// sim_test.cpp — Offline simulation: AWGN channel, BER vs SNR curves
// No radio or soundcard required.
//
// Usage:
//   ./sim_test               # run BER sweep across all tiers
//   ./sim_test --snr 3.5     # test at a specific SNR
//   ./sim_test --tier 5      # force a specific ACM tier

#include <iostream>
#include <iomanip>
#include <vector>
#include <random>
#include <cmath>
#include <string>
#include <sstream>
#include "ofdm_core.h"
#include "acm.h"
#include "framer.h"
#include "negotiation.h"

using namespace opendsp;

// ── AWGN channel ──────────────────────────────────────────────────────────────

static std::mt19937_64 rng(42);

// Add AWGN to a real sample vector.  snr_db is per-subcarrier SNR.
RealVec add_awgn(const RealVec& signal, double snr_db) {
    double signal_power = 0.0;
    for (auto s : signal) signal_power += s * s;
    signal_power /= signal.size();

    double snr_linear = std::pow(10.0, snr_db / 10.0);
    double noise_std  = std::sqrt(signal_power / (snr_linear + 1e-12));

    std::normal_distribution<double> noise(0.0, noise_std);
    RealVec out(signal.size());
    for (size_t i = 0; i < signal.size(); i++) out[i] = signal[i] + noise(rng);
    return out;
}

// ── BER test for one tier ─────────────────────────────────────────────────────

struct BERResult {
    int    tier;
    double snr_db;
    double ber;
    int    bits_tested;
    int    bit_errors;
    double throughput_bps;
};

BERResult run_ber_test(int tier_idx, double snr_db, int num_symbols = 500) {
    ACMEngine acm;
    acm.force_tier(tier_idx);
    OFDMParams p = acm.current_params();

    OFDMModulator   mod(p);
    OFDMDemodulator demod(p);
    Framer          framer;

    int  total_bits  = 0;
    int  bit_errors  = 0;
    ChannelEstimate  ch_est;

    std::uniform_int_distribution<int> bit_dist(0, 1);

    for (int sym = 0; sym < num_symbols; sym++) {
        // Generate random bits for data subcarriers (skip pilots)
        std::vector<uint8_t> tx_bits;
        for (int i = 0; i < p.num_subcarriers; i++) {
            if (i % p.pilot_interval == 0) continue;
            for (int b = 0; b < p.bits_per_symbol; b++)
                tx_bits.push_back(bit_dist(rng));
        }

        // Build subcarrier vector
        CxVec tx_sc(p.num_subcarriers);
        int bit_idx = 0;
        for (int i = 0; i < p.num_subcarriers; i++) {
            if (i % p.pilot_interval == 0) {
                // Pilot — modulator will overwrite
                tx_sc[i] = cx(1, 0);
            } else {
                uint8_t sym_bits = 0;
                for (int b = 0; b < p.bits_per_symbol; b++)
                    sym_bits |= (tx_bits[bit_idx++] & 1) << (p.bits_per_symbol - 1 - b);
                tx_sc[i] = map_symbol(sym_bits, p.bits_per_symbol);
            }
        }
        mod.insert_pilots(tx_sc, sym);

        // Modulate → AWGN → Demodulate
        RealVec tx_samples = mod.modulate_symbol(tx_sc);
        RealVec rx_samples = add_awgn(tx_samples, snr_db);
        CxVec   rx_sc      = demod.demodulate_symbol(rx_samples);

        // Channel estimate
        demod.update_channel_estimate(rx_sc, sym, ch_est);

        // Equalise and demap
        auto llr = demod.equalise_and_demap(rx_sc, ch_est);

        // Compare bits (hard decision)
        int llr_idx = 0;
        for (int i = 0; i < p.num_subcarriers; i++) {
            if (i % p.pilot_interval == 0) continue;
            for (int b = 0; b < p.bits_per_symbol; b++) {
                int ref_bit_idx = (i - i/p.pilot_interval) * p.bits_per_symbol + b;
                // Guard against going out of bounds
                if (ref_bit_idx >= static_cast<int>(tx_bits.size())) break;
                if (llr_idx >= static_cast<int>(llr.size())) break;
                int decoded_bit = (llr[llr_idx++] < 0.0) ? 1 : 0;
                if (decoded_bit != tx_bits[ref_bit_idx]) bit_errors++;
                total_bits++;
            }
        }
    }

    BERResult r;
    r.tier          = tier_idx;
    r.snr_db        = snr_db;
    r.bits_tested   = total_bits;
    r.bit_errors    = bit_errors;
    r.ber           = total_bits > 0 ? static_cast<double>(bit_errors) / total_bits : 0.5;
    r.throughput_bps = acm.estimated_throughput_bps();
    return r;
}

// ── Negotiation smoke test ────────────────────────────────────────────────────

void run_negotiation_test() {
    std::cout << "\n── Negotiation smoke test ────────────────────────────────\n";

    ACMEngine acm_a, acm_b;
    acm_a.update(-3.0, 0.01);  // Station A: fair channel
    acm_b.update(-3.0, 0.01);  // Station B: same

    // Shared "air" — frames passed directly
    std::vector<Frame> ab_wire, ba_wire;
    Framer framer;

    auto send_a = [&](const Frame& f) {
        auto raw = framer.build(f);
        ab_wire.push_back(framer.parse(raw));
    };
    auto send_b = [&](const Frame& f) {
        auto raw = framer.build(f);
        ba_wire.push_back(framer.parse(raw));
    };

    auto tier_a = [](int t) { std::cout << "  [A] Tier changed → " << t << "\n"; };
    auto tier_b = [](int t) { std::cout << "  [B] Tier changed → " << t << "\n"; };

    NegotiationManager neg_a("W1AW",   acm_a, send_a, tier_a);
    NegotiationManager neg_b("VK2XYZ", acm_b, send_b, tier_b);

    // A initiates
    neg_a.initiate_probe();

    // Deliver A's probe to B
    for (auto& f : ab_wire) neg_b.on_frame_received(f);
    ab_wire.clear();

    // Deliver B's ACK to A
    for (auto& f : ba_wire) neg_a.on_frame_received(f);
    ba_wire.clear();

    std::cout << "  [A] state=" << neg_a.state_str()
              << " agreed_tier=" << neg_a.agreed_tier() << "\n";
    std::cout << "  [B] state=" << neg_b.state_str()
              << " agreed_tier=" << neg_b.agreed_tier() << "\n";

    // Simulate SNR degradation → B sends RETUNE
    acm_b.update(-9.0, 0.05);
    neg_b.on_snr_update(-9.0);
    for (auto& f : ba_wire) neg_a.on_frame_received(f);
    ba_wire.clear();
    for (auto& f : ab_wire) neg_b.on_frame_received(f);
    ab_wire.clear();

    std::cout << "  After retune: [A] tier=" << neg_a.agreed_tier()
              << "  [B] tier=" << neg_b.agreed_tier() << "\n";
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    double force_snr  = -999;
    int    force_tier = -1;

    for (int i = 1; i < argc; i++) {
        std::string arg(argv[i]);
        if (arg == "--snr"  && i+1 < argc) force_snr  = std::stod(argv[++i]);
        if (arg == "--tier" && i+1 < argc) force_tier = std::stoi(argv[++i]);
    }

    ACMEngine acm_probe;

    std::cout << "OpenDSP-OFDM Simulation Test\n";
    std::cout << "============================\n\n";

    if (force_tier >= 0) {
        // Single tier test
        double snr = (force_snr > -900) ? force_snr
                   : acm_probe.tier(force_tier).snr_threshold_db + 2.0;
        auto r = run_ber_test(force_tier, snr, 1000);
        std::cout << "Tier " << r.tier << " @ SNR=" << snr << " dB\n";
        std::cout << "  BER=" << std::scientific << r.ber
                  << "  errors=" << r.bit_errors << "/" << r.bits_tested
                  << "  throughput~" << std::fixed << (int)r.throughput_bps << " bps\n";
    } else {
        // Full BER sweep
        std::cout << std::left
                  << std::setw(6)  << "Tier"
                  << std::setw(22) << "Description"
                  << std::setw(10) << "SNR(dB)"
                  << std::setw(12) << "BER"
                  << std::setw(14) << "Throughput"
                  << "\n";
        std::cout << std::string(64, '-') << "\n";

        for (int t = 0; t < acm_probe.num_tiers(); t++) {
            const auto& tier = acm_probe.tier(t);
            // Test at threshold + 2 dB (middle of operational range)
            double snr = tier.snr_threshold_db + 2.0;
            auto r = run_ber_test(t, snr, 300);

            std::ostringstream bps_str;
            if (r.throughput_bps >= 1000)
                bps_str << std::fixed << std::setprecision(1) << r.throughput_bps/1000.0 << " kbps";
            else
                bps_str << (int)r.throughput_bps << " bps";

            std::cout << std::left
                      << std::setw(6)  << t
                      << std::setw(22) << tier.description.substr(0,21)
                      << std::setw(10) << std::fixed << std::setprecision(1) << snr
                      << std::setw(12) << std::scientific << std::setprecision(2) << r.ber
                      << std::setw(14) << bps_str.str()
                      << "\n";
        }

        run_negotiation_test();
    }

    std::cout << "\nDone.\n";
    return 0;
}
