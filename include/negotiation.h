#pragma once
#include <string>
#include <functional>
#include <chrono>
#include "framer.h"
#include "acm.h"

namespace opendsp {

// ── Negotiation states ────────────────────────────────────────────────────────

enum class NegState {
    IDLE,           // Not in a QSO
    PROBING,        // Sent PROBE, waiting for ACK
    ACKNOWLEDGED,   // Both sides agreed on a tier
    ACTIVE,         // Data transfer in progress
    RETUNING,       // Mid-QSO retune requested, waiting for ACK
};

// ── Negotiation result ────────────────────────────────────────────────────────

struct NegResult {
    bool    success;
    int     agreed_tier;
    double  agreed_bw_hz;
    std::string message;
};

// ── NegotiationManager ────────────────────────────────────────────────────────
//
// Implements the PROBE → ACK → ACTIVE → [RETUNE → ACK]* handshake.
//
// The caller wires up two callbacks:
//   on_send_frame  — called when the manager wants to transmit a frame
//   on_tier_change — called when an agreed tier is established
//
// Usage:
//   1. Calling station: call initiate_probe()
//   2. On frame RX:     call on_frame_received()
//   3. ACM engine feeds SNR updates via on_snr_update()

class NegotiationManager {
public:
    using SendCallback       = std::function<void(const Frame&)>;
    using TierChangeCallback = std::function<void(int tier_index)>;

    NegotiationManager(const std::string& our_callsign,
                       ACMEngine&         acm,
                       SendCallback       on_send,
                       TierChangeCallback on_tier_change);

    // ── Initiator side ────────────────────────────────────────────────────────

    // Start negotiation: transmit a PROBE at minimum tier.
    void initiate_probe();

    // ── Both sides ────────────────────────────────────────────────────────────

    // Feed a received frame into the state machine.
    void on_frame_received(const Frame& f);

    // Called by ACM engine when SNR changes significantly (≥ 2 dB).
    // If in ACTIVE state and a tier change is warranted, sends RETUNE.
    void on_snr_update(double snr_db);

    // Periodic tick — call at ~1 Hz to handle timeouts / retries.
    void tick();

    NegState    state()    const { return state_; }
    std::string state_str() const;
    int         agreed_tier() const { return agreed_tier_; }

private:
    std::string        callsign_;
    ACMEngine&         acm_;
    SendCallback       on_send_;
    TierChangeCallback on_tier_change_;

    NegState    state_         = NegState::IDLE;
    int         agreed_tier_   = 0;
    int         probe_retries_ = 0;
    double      last_snr_db_   = -20.0;

    using Clock = std::chrono::steady_clock;
    Clock::time_point last_tx_time_;

    static constexpr int    MAX_PROBE_RETRIES  = 3;
    static constexpr double PROBE_TIMEOUT_SEC  = 16.0;  // 1 FT8 frame window
    static constexpr double RETUNE_HYSTERESIS  = 3.0;   // dB swing before requesting retune

    Framer framer_;

    void handle_probe(const Frame& f);
    void handle_ack(const Frame& f);
    void handle_retune(const Frame& f);
    void handle_nak(const Frame& f);

    // Pick the best mutually-supported tier given current SNR and peer max tier.
    int negotiate_tier(int peer_max_tier) const;
};

} // namespace opendsp
