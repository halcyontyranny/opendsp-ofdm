#pragma once
#include <cstdint>
#include <string>
#include "ofdm_core.h"

namespace opendsp {

// ── Modulation / coding tier ──────────────────────────────────────────────────

enum class ModOrder : uint8_t {
    BPSK   = 1,
    QPSK   = 2,
    PSK8   = 3,
    QAM16  = 4,
};

struct CodeRate {
    int num, den;           // e.g. num=1 den=4 → rate 1/4
    double value() const { return static_cast<double>(num) / den; }
    std::string str() const { return std::to_string(num)+"/"+std::to_string(den); }
};

struct ACMTier {
    double   snr_threshold_db;   // Minimum SNR to use this tier
    double   bw_hz;              // Occupied bandwidth
    double   sub_spacing_hz;     // Subcarrier spacing
    ModOrder mod;
    CodeRate rate;
    double   frame_window_sec;   // TX frame duration (total, including all passes)
    int      passes;             // LLR accumulation passes (1 for all tiers except 0)
    std::string description;
};

// ── ACM state ─────────────────────────────────────────────────────────────────

struct ACMState {
    int      tier_index;         // Current tier (index into tier table)
    double   current_snr_db;     // Last measured SNR
    double   current_ber;        // Last measured BER (post-decode)
    int      frames_at_snr;      // Consecutive frames at or above next tier threshold
    bool     downgrade_pending;  // Immediate downgrade flagged
    double   estimated_bps;      // Current estimated throughput
};

// ── ACM engine ────────────────────────────────────────────────────────────────

class ACMEngine {
public:
    ACMEngine();

    // Feed new SNR and BER measurements; returns true if tier changed.
    bool update(double snr_db, double ber);

    // Build OFDMParams for the current tier.
    OFDMParams current_params() const;

    // Current tier descriptor.
    const ACMTier& current_tier() const;

    // Estimated throughput in bits/sec for current tier.
    double estimated_throughput_bps() const;

    // Human-readable status line.
    std::string status_string() const;

    // Force a specific tier (for negotiation / testing).
    void force_tier(int index);

    int num_tiers() const;
    const ACMTier& tier(int i) const;

    const ACMState& state() const { return state_; }

private:
    std::vector<ACMTier> tiers_;
    ACMState             state_;

    static constexpr int    UPGRADE_HOLD_FRAMES = 3;    // frames above threshold before upgrade
    static constexpr double UPGRADE_HYSTERESIS  = 1.5;  // dB above threshold to upgrade
    static constexpr double DOWNGRADE_BER_LIMIT = 0.01; // immediate downgrade if BER exceeds this

    void build_tier_table();
    double compute_throughput(const ACMTier& t) const;
};

} // namespace opendsp
