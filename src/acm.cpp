#include "acm.h"
#include <cmath>
#include <sstream>
#include <iomanip>
#include <stdexcept>

namespace opendsp {

// ── ACM tier table ────────────────────────────────────────────────────────────
//
// Key HF realities baked in:
//  - Sub-spacing stays at 6.25 Hz until SNR ≥ 0 dB (Doppler tolerance)
//  - 64-QAM removed — unreliable on real HF channels
//  - 16-QAM only above +6 dB where phase noise is manageable
//  - BW grows gradually so interference footprint scales with channel quality
//
// Tier 0 (survival) uses a 50 Hz channel (FT8-equivalent bandwidth) with
// H_256_768_22 (rate 1/3) and 4-pass LLR soft combining, giving an effective
// decode threshold of ≈ −8 dB channel SNR (≈ −25 dB in 2500 Hz reference BW).
// Code rates in the table match the actual codec2 LDPC codes used.

void ACMEngine::build_tier_table() {
    tiers_ = {
        //   SNR_thresh  BW_hz    spacing  mod             rate      window  passes  desc
        { -20.0,   50.0,  6.25,  ModOrder::BPSK,  {1,3},  90.0, 4, "Survival: 50Hz BPSK 1/3×4p" },
        { -12.0,  500.0,  6.25,  ModOrder::BPSK,  {1,3},  15.0, 1, "Weak: 500Hz BPSK 1/3"       },
        {  -8.0, 1000.0,  6.25,  ModOrder::BPSK,  {1,2},  12.0, 1, "Weak+: 1kHz BPSK 1/2"       },
        {  -5.0, 1500.0,  6.25,  ModOrder::QPSK,  {1,2},  10.0, 1, "Fair: 1.5kHz QPSK 1/2"      },
        {  -2.0, 2000.0,  6.25,  ModOrder::QPSK,  {1,2},  10.0, 1, "Fair+: 2kHz QPSK 1/2"       },
        {   1.0, 2500.0,  6.25,  ModOrder::QPSK,  {1,2},   7.5, 1, "Good: 2.5kHz QPSK 1/2"      },
        {   4.0, 3000.0, 12.5,   ModOrder::PSK8,  {3,4},   7.5, 1, "Good+: 3kHz 8PSK 3/4"       },
        {   7.0, 3200.0, 12.5,   ModOrder::PSK8,  {3,4},   7.5, 1, "Exc: 3.2kHz 8PSK 3/4"       },
        {  10.0, 3500.0, 25.0,   ModOrder::QAM16, {3,4},   7.5, 1, "Max: 3.5kHz 16QAM 3/4"      },
    };
}

// ── Constructor ───────────────────────────────────────────────────────────────

ACMEngine::ACMEngine() {
    build_tier_table();
    state_.tier_index       = 0;
    state_.current_snr_db   = -20.0;
    state_.current_ber      = 0.5;
    state_.frames_at_snr    = 0;
    state_.downgrade_pending = false;
    state_.estimated_bps    = compute_throughput(tiers_[0]);
}

// ── Throughput estimator ──────────────────────────────────────────────────────
//
// throughput = bw × (bits_per_sym / sub_spacing) × code_rate × (1 − overhead)
// overhead accounts for: pilots (1/pilot_interval), cyclic prefix, sync, guard

double ACMEngine::compute_throughput(const ACMTier& t) const {
    int bps = static_cast<int>(t.mod);
    int num_sc = static_cast<int>(std::floor(t.bw_hz / t.sub_spacing_hz));

    // Pilot overhead: every 4th subcarrier
    double pilot_frac    = 1.0 / 4.0;
    double data_sc_frac  = 1.0 - pilot_frac;

    // Symbol duration including CP
    double sym_dur_sec = (1.0 / t.sub_spacing_hz) * (1.0 + CP_RATIO);

    // Symbols per second
    double syms_per_sec = 1.0 / sym_dur_sec;

    // Raw bits per second
    double raw_bps = num_sc * data_sc_frac * bps * syms_per_sec;

    // Apply code rate
    double coded_bps = raw_bps * t.rate.value();

    // Frame overhead: sync word (~0.5 s), ACM header, CRC → ~15% total
    double frame_overhead = 0.15;

    // Multi-pass accumulation reduces net data rate proportionally
    return coded_bps * (1.0 - frame_overhead) / t.passes;
}

// ── update ────────────────────────────────────────────────────────────────────

bool ACMEngine::update(double snr_db, double ber) {
    state_.current_snr_db = snr_db;
    state_.current_ber    = ber;

    int old_tier = state_.tier_index;

    // Immediate downgrade on BER or CRC failure
    if (ber > DOWNGRADE_BER_LIMIT && state_.tier_index > 0) {
        state_.tier_index--;
        state_.frames_at_snr = 0;
        state_.downgrade_pending = false;
        state_.estimated_bps = compute_throughput(tiers_[state_.tier_index]);
        return true;
    }

    // Consider upgrade (with hysteresis + hold-off)
    int next_tier = state_.tier_index + 1;
    if (next_tier < static_cast<int>(tiers_.size())) {
        double upgrade_threshold = tiers_[next_tier].snr_threshold_db + UPGRADE_HYSTERESIS;
        if (snr_db >= upgrade_threshold) {
            state_.frames_at_snr++;
            if (state_.frames_at_snr >= UPGRADE_HOLD_FRAMES) {
                state_.tier_index++;
                state_.frames_at_snr = 0;
                state_.estimated_bps = compute_throughput(tiers_[state_.tier_index]);
                return true;
            }
        } else {
            state_.frames_at_snr = 0;
        }
    }

    // Consider downgrade (no hysteresis — safety first)
    if (state_.tier_index > 0) {
        double cur_threshold = tiers_[state_.tier_index].snr_threshold_db;
        if (snr_db < cur_threshold - 1.0) {
            state_.tier_index--;
            state_.frames_at_snr = 0;
            state_.estimated_bps = compute_throughput(tiers_[state_.tier_index]);
            return true;
        }
    }

    state_.estimated_bps = compute_throughput(tiers_[state_.tier_index]);
    return (state_.tier_index != old_tier);
}

// ── Accessors ─────────────────────────────────────────────────────────────────

OFDMParams ACMEngine::current_params() const {
    const auto& t = tiers_[state_.tier_index];
    return make_params(t.bw_hz, t.sub_spacing_hz, static_cast<int>(t.mod));
}

const ACMTier& ACMEngine::current_tier() const {
    return tiers_[state_.tier_index];
}

double ACMEngine::estimated_throughput_bps() const {
    return state_.estimated_bps;
}

void ACMEngine::force_tier(int index) {
    if (index < 0 || index >= static_cast<int>(tiers_.size()))
        throw std::out_of_range("Tier index out of range");
    state_.tier_index = index;
    state_.frames_at_snr = 0;
    state_.estimated_bps = compute_throughput(tiers_[index]);
}

int ACMEngine::num_tiers() const {
    return static_cast<int>(tiers_.size());
}

const ACMTier& ACMEngine::tier(int i) const {
    return tiers_.at(i);
}

std::string ACMEngine::status_string() const {
    const auto& t = current_tier();
    static const char* mod_names[] = {"?","BPSK","QPSK","8PSK","16QAM"};
    int bps = static_cast<int>(t.mod);

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1);
    oss << "[ACM tier " << state_.tier_index << "] "
        << t.bw_hz/1000.0 << " kHz  "
        << mod_names[bps] << "  R=" << t.rate.str() << "  "
        << "SNR=" << state_.current_snr_db << " dB  "
        << "BER=" << std::scientific << std::setprecision(2) << state_.current_ber << "  "
        << std::fixed << std::setprecision(0)
        << "~" << state_.estimated_bps << " bps";
    return oss.str();
}

} // namespace opendsp
