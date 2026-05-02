#include "negotiation.h"
#include <iostream>
#include <algorithm>

namespace opendsp {

NegotiationManager::NegotiationManager(const std::string& callsign,
                                        ACMEngine&         acm,
                                        SendCallback       on_send,
                                        TierChangeCallback on_tier_change)
    : callsign_(callsign)
    , acm_(acm)
    , on_send_(std::move(on_send))
    , on_tier_change_(std::move(on_tier_change))
{}

// ── Initiate probe ────────────────────────────────────────────────────────────

void NegotiationManager::initiate_probe() {
    if (state_ != NegState::IDLE) return;

    Frame probe = framer_.make_probe(callsign_, acm_.num_tiers() - 1);
    on_send_(probe);
    state_        = NegState::PROBING;
    probe_retries_ = 0;
    last_tx_time_  = Clock::now();

    std::cout << "[NEG] Sent PROBE (our max tier: "
              << acm_.num_tiers() - 1 << ")\n";
}

// ── Frame received ────────────────────────────────────────────────────────────

void NegotiationManager::on_frame_received(const Frame& f) {
    if (!f.crc_ok) {
        std::cerr << "[NEG] Frame from " << f.callsign << " CRC FAIL — ignored\n";
        return;
    }
    switch (f.header.frame_type) {
    case FrameType::PROBE:   handle_probe(f);   break;
    case FrameType::ACK:     handle_ack(f);     break;
    case FrameType::RETUNE:  handle_retune(f);  break;
    case FrameType::NAK:     handle_nak(f);     break;
    default: break;
    }
}

// ── SNR update ────────────────────────────────────────────────────────────────

void NegotiationManager::on_snr_update(double snr_db) {
    last_snr_db_ = snr_db;
    if (state_ != NegState::ACTIVE) return;

    // If SNR has changed enough to warrant a different tier, send RETUNE
    bool changed = acm_.update(snr_db, acm_.state().current_ber);
    if (changed && state_ == NegState::ACTIVE) {
        int new_tier = acm_.state().tier_index;
        if (new_tier != agreed_tier_) {
            Frame retune = framer_.make_retune(callsign_, new_tier);
            on_send_(retune);
            state_ = NegState::RETUNING;
            last_tx_time_ = Clock::now();
            std::cout << "[NEG] Sent RETUNE → tier " << new_tier
                      << " (" << acm_.tier(new_tier).description << ")\n";
        }
    }
}

// ── Tick (call at ~1 Hz) ──────────────────────────────────────────────────────

void NegotiationManager::tick() {
    if (state_ != NegState::PROBING && state_ != NegState::RETUNING) return;

    auto elapsed = std::chrono::duration<double>(Clock::now() - last_tx_time_).count();

    if (elapsed > PROBE_TIMEOUT_SEC) {
        if (state_ == NegState::PROBING) {
            if (probe_retries_ < MAX_PROBE_RETRIES) {
                probe_retries_++;
                Frame probe = framer_.make_probe(callsign_, acm_.num_tiers() - 1);
                on_send_(probe);
                last_tx_time_ = Clock::now();
                std::cout << "[NEG] PROBE retry " << probe_retries_ << "\n";
            } else {
                std::cout << "[NEG] PROBE timed out — reverting to IDLE\n";
                state_ = NegState::IDLE;
            }
        } else if (state_ == NegState::RETUNING) {
            // Peer didn't ACK retune — fall back to agreed tier
            std::cout << "[NEG] RETUNE timed out — staying on tier " << agreed_tier_ << "\n";
            acm_.force_tier(agreed_tier_);
            state_ = NegState::ACTIVE;
        }
    }
}

// ── Handlers ──────────────────────────────────────────────────────────────────

void NegotiationManager::handle_probe(const Frame& f) {
    int peer_max = f.header.max_bw_code;
    int tier     = negotiate_tier(peer_max);

    std::cout << "[NEG] Received PROBE from " << f.callsign
              << " (max tier " << peer_max << ") → agreeing tier " << tier << "\n";

    Frame ack = framer_.make_ack(callsign_, tier, acm_.num_tiers() - 1);
    on_send_(ack);

    agreed_tier_ = tier;
    acm_.force_tier(tier);
    state_ = NegState::ACTIVE;
    on_tier_change_(tier);
}

void NegotiationManager::handle_ack(const Frame& f) {
    if (state_ != NegState::PROBING && state_ != NegState::RETUNING) return;

    int agreed = static_cast<int>(f.header.tier_index);
    // Sanity: take the lower of what peer says and what we support
    agreed = std::min(agreed, acm_.num_tiers() - 1);

    std::cout << "[NEG] ACK from " << f.callsign
              << " — agreed tier " << agreed
              << " (" << acm_.tier(agreed).description << ")\n";

    agreed_tier_ = agreed;
    acm_.force_tier(agreed);
    state_ = NegState::ACTIVE;
    on_tier_change_(agreed);
}

void NegotiationManager::handle_retune(const Frame& f) {
    int requested = static_cast<int>(f.header.tier_index);
    requested = std::min(requested, acm_.num_tiers() - 1);

    std::cout << "[NEG] RETUNE request from " << f.callsign
              << " → tier " << requested
              << " (" << acm_.tier(requested).description << ")\n";

    // Accept if we support it; else ACK with our max
    int accepted = std::min(requested, acm_.num_tiers() - 1);
    Frame ack = framer_.make_ack(callsign_, accepted, acm_.num_tiers() - 1);
    on_send_(ack);

    agreed_tier_ = accepted;
    acm_.force_tier(accepted);
    state_ = NegState::ACTIVE;
    on_tier_change_(accepted);
}

void NegotiationManager::handle_nak(const Frame& f) {
    std::cout << "[NEG] NAK from " << f.callsign << " — falling back to tier 0\n";
    agreed_tier_ = 0;
    acm_.force_tier(0);
    state_ = NegState::IDLE;
    on_tier_change_(0);
}

// ── Tier negotiation logic ────────────────────────────────────────────────────

int NegotiationManager::negotiate_tier(int peer_max_tier) const {
    double snr  = acm_.state().current_snr_db;
    int    upper = std::min(peer_max_tier, acm_.num_tiers() - 1);

    int best = 0;
    for (int i = upper; i >= 0; i--) {
        if (snr >= acm_.tier(i).snr_threshold_db) {
            best = i;
            break;
        }
    }
    return best;
}

std::string NegotiationManager::state_str() const {
    switch (state_) {
    case NegState::IDLE:         return "IDLE";
    case NegState::PROBING:      return "PROBING";
    case NegState::ACKNOWLEDGED: return "ACKNOWLEDGED";
    case NegState::ACTIVE:       return "ACTIVE";
    case NegState::RETUNING:     return "RETUNING";
    default:                     return "UNKNOWN";
    }
}

} // namespace opendsp
