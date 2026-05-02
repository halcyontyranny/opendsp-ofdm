// sim_test.cpp — Offline simulation: AWGN channel, BER vs SNR curves
// No radio or soundcard required.
//
// Usage:
//   ./sim_test                        # BER sweep: each tier tested at its own threshold SNR
//   ./sim_test --channel-tier 7       # BER sweep: all tiers tested at tier-7 channel SNR
//   ./sim_test --tier 5               # single tier at its own threshold SNR
//   ./sim_test --tier 5 --channel-tier 7  # tier-5 modem under tier-7 channel conditions
//   ./sim_test --snr 3.5              # explicit SNR (overrides --channel-tier)
//   ./sim_test --seed 42              # fix RNG seed for reproducibility

#include <iostream>
#include <iomanip>
#include <vector>
#include <random>
#include <cmath>
#include <string>
#include <sstream>
#include <fstream>
#include <cstdint>
#include "ofdm_core.h"
#include "acm.h"
#include "framer.h"
#include "negotiation.h"
#ifdef HAVE_CODEC2
#include "fec.h"
#endif

using namespace opendsp;

// ── AWGN channel ──────────────────────────────────────────────────────────────

static std::mt19937_64 rng;

static uint64_t read_urandom_seed() {
    uint64_t seed = 0;
    std::ifstream f("/dev/urandom", std::ios::binary);
    f.read(reinterpret_cast<char*>(&seed), sizeof(seed));
    return seed;
}

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
                int ref_bit_idx = (i - (i + p.pilot_interval - 1) / p.pilot_interval) * p.bits_per_symbol + b;
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

// ── FEC-aware BER test (uses FECAccumulator; handles 1-pass and N-pass) ───────

#ifdef HAVE_CODEC2
struct FECBERResult {
    int    tier;
    double snr_db;
    double raw_ber;    // pre-FEC hard-decision BER (per coded bit, per pass)
    double fec_ber;    // post-FEC BER on recovered data bits
    int    num_passes; // accumulation passes used
    int    messages_tested;
};

// Transmit a single FEC block (cb coded bits) over syms_per_block OFDM symbols
// and return the collected LLR vector (length = cb after truncation).
static std::vector<float> tx_one_pass(
        const std::vector<uint8_t>& coded,  // padded to bits_capacity
        int cb, int syms_per_block, int sym_idx_offset,
        OFDMModulator& mod, OFDMDemodulator& demod,
        ChannelEstimate& ch_est,
        const OFDMParams& p, double snr_db)
{
    std::vector<float> pass_llrs;
    pass_llrs.reserve(syms_per_block * p.num_subcarriers * p.bits_per_symbol);
    int bit_offset = 0;
    for (int sym = 0; sym < syms_per_block; sym++) {
        CxVec tx_sc(p.num_subcarriers);
        for (int i = 0; i < p.num_subcarriers; i++) {
            if (i % p.pilot_interval == 0) {
                tx_sc[i] = cx(1, 0);
            } else {
                uint8_t sym_bits = 0;
                for (int b = 0; b < p.bits_per_symbol; b++) {
                    int bidx = bit_offset + b;
                    uint8_t bit = (bidx < static_cast<int>(coded.size())) ? coded[bidx] : 0;
                    sym_bits |= (bit & 1) << (p.bits_per_symbol - 1 - b);
                }
                tx_sc[i] = map_symbol(sym_bits, p.bits_per_symbol);
                bit_offset += p.bits_per_symbol;
            }
        }
        mod.insert_pilots(tx_sc, sym_idx_offset + sym);
        RealVec tx_samples = mod.modulate_symbol(tx_sc);
        RealVec rx_samples = add_awgn(tx_samples, snr_db);
        CxVec   rx_sc      = demod.demodulate_symbol(rx_samples);
        demod.update_channel_estimate(rx_sc, sym_idx_offset + sym, ch_est);
        auto llr = demod.equalise_and_demap(rx_sc, ch_est);
        for (auto v : llr) pass_llrs.push_back(static_cast<float>(v));
    }
    pass_llrs.resize(cb);
    return pass_llrs;
}

FECBERResult run_fec_ber_test(int tier_idx, double snr_db, int num_messages = 50) {
    ACMEngine acm;
    acm.force_tier(tier_idx);
    OFDMParams p = acm.current_params();

    int num_passes = passes_for_tier(tier_idx);

    FECCodec        fec(fec_code_for_tier(tier_idx));   // encoder reference
    FECAccumulator  accum(fec_code_for_tier(tier_idx), num_passes);
    OFDMModulator   mod(p);
    OFDMDemodulator demod(p);

    int db = fec.data_bits();
    int cb = fec.coded_bits();

    int data_sc = 0;
    for (int i = 0; i < p.num_subcarriers; i++)
        if (i % p.pilot_interval != 0) data_sc++;
    int bits_per_sym  = data_sc * p.bits_per_symbol;
    int syms_per_block = (cb + bits_per_sym - 1) / bits_per_sym;
    int bits_capacity  = syms_per_block * bits_per_sym;

    int raw_bits = 0, raw_errors = 0;
    int fec_bits = 0, fec_errors = 0;

    std::uniform_int_distribution<int> bit_dist(0, 1);

    for (int msg = 0; msg < num_messages; msg++) {
        std::vector<uint8_t> payload(db);
        for (auto& b : payload) b = bit_dist(rng);

        auto coded     = fec.encode(payload);
        auto coded_ref = coded;
        coded.resize(bits_capacity, 0);

        accum.reset();

        for (int pass = 0; pass < num_passes; pass++) {
            ChannelEstimate ch_est;   // fresh estimate per pass (mimics per-pass preamble)
            auto pass_llrs = tx_one_pass(coded, cb, syms_per_block, pass * syms_per_block,
                                         mod, demod, ch_est, p, snr_db);
            for (int i = 0; i < cb; i++) {
                if (((pass_llrs[i] < 0.0f) ? 1 : 0) != coded_ref[i]) raw_errors++;
                raw_bits++;
            }
            accum.add_pass(pass_llrs);
        }

        std::vector<uint8_t> decoded;
        accum.decode(decoded);
        for (int i = 0; i < db; i++) {
            if (decoded[i] != payload[i]) fec_errors++;
            fec_bits++;
        }
    }

    FECBERResult r;
    r.tier            = tier_idx;
    r.snr_db          = snr_db;
    r.raw_ber         = raw_bits > 0 ? static_cast<double>(raw_errors) / raw_bits : 0.5;
    r.fec_ber         = fec_bits > 0 ? static_cast<double>(fec_errors) / fec_bits : 0.5;
    r.num_passes      = num_passes;
    r.messages_tested = num_messages;
    return r;
}
#endif // HAVE_CODEC2

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
    double   force_snr      = -999;
    int      force_tier     = -1;
    int      channel_tier   = -1;
    int64_t  force_seed     = -1;
    bool     no_fec         = false;
    bool     survival_sweep = false;

    for (int i = 1; i < argc; i++) {
        std::string arg(argv[i]);
        if (arg == "--snr"          && i+1 < argc) force_snr      = std::stod(argv[++i]);
        if (arg == "--tier"         && i+1 < argc) force_tier     = std::stoi(argv[++i]);
        if (arg == "--channel-tier" && i+1 < argc) channel_tier   = std::stoi(argv[++i]);
        if (arg == "--seed"         && i+1 < argc) force_seed     = std::stoll(argv[++i]);
        if (arg == "--no-fec")                     no_fec         = true;
        if (arg == "--survival")                   survival_sweep = true;
        if (arg == "--help") {
            std::cout <<
"sim_test — OpenDSP-OFDM offline BER/SNR simulator\n"
"\n"
"Usage:\n"
"  sim_test [options]\n"
"\n"
"Options:\n"
"  --tier N          Test a single modem tier (0–8) instead of the full sweep\n"
"  --channel-tier N  Set channel SNR to tier N's nominal operating point\n"
"                    (threshold + 2 dB). Ignored if --snr is also given.\n"
"  --snr F           Set channel SNR to exactly F dB for all tested tiers\n"
"  --seed N          Fix RNG seed for reproducible results\n"
"                    (default: seeded from /dev/urandom)\n"
"  --no-fec          Show raw (pre-FEC) BER only; skip codec2 LDPC decoding\n"
"  --survival        Run survival-mode SNR sweep (tier 0, 4-pass accumulation)\n"
"  --help            Show this help\n"
"\n"
"Channel tiers and their nominal SNR operating points:\n"
"  Tier  Description                 SNR threshold  Test SNR (+2 dB)\n"
"  0     Survival: 50 Hz BPSK 1/3×4  -20 dB        -18 dB (4-pass threshold ≈ -8 dB)\n"
"  1     Weak: 500 Hz BPSK 1/3       -12 dB        -10 dB\n"
"  2     Weak+: 1 kHz BPSK 1/2        -8 dB         -6 dB\n"
"  3     Fair: 1.5 kHz QPSK 1/2       -5 dB         -3 dB\n"
"  4     Fair+: 2 kHz QPSK 1/2        -2 dB          0 dB\n"
"  5     Good: 2.5 kHz QPSK 1/2       +1 dB         +3 dB\n"
"  6     Good+: 3 kHz 8-PSK 3/4       +4 dB         +6 dB\n"
"  7     Exc: 3.2 kHz 8-PSK 3/4       +7 dB         +9 dB\n"
"  8     Max: 3.5 kHz 16-QAM 3/4     +10 dB        +12 dB\n"
"\n"
"Examples:\n"
"  sim_test                              Full sweep, each tier at its own SNR\n"
"  sim_test --survival                   Survival tier 0 SNR sweep (FT8 comparison)\n"
"  sim_test --channel-tier 7             Full sweep at tier-7 conditions (9 dB)\n"
"  sim_test --tier 7                     Tier-7 modem at tier-7 channel SNR\n"
"  sim_test --tier 8 --channel-tier 7   Tier-8 modem under tier-7 conditions\n"
"  sim_test --snr 5.0 --tier 6          Tier-6 modem at exactly 5.0 dB\n"
"  sim_test --seed 42                    Reproducible run with fixed seed\n";
            return 0;
        }
    }

    uint64_t seed = (force_seed >= 0) ? static_cast<uint64_t>(force_seed)
                                      : read_urandom_seed();
    rng.seed(seed);

    ACMEngine acm_probe;

    // Resolve channel SNR: explicit --snr > --channel-tier > per-tier default
    auto channel_snr = [&](int modem_tier) -> double {
        if (force_snr > -900)  return force_snr;
        if (channel_tier >= 0) return acm_probe.tier(channel_tier).snr_threshold_db + 2.0;
        return acm_probe.tier(modem_tier).snr_threshold_db + 2.0;
    };

    std::cout << "OpenDSP-OFDM Simulation Test\n";
    std::cout << "============================\n";
    std::cout << "seed: " << seed << "\n";
    if (channel_tier >= 0)
        std::cout << "channel: tier " << channel_tier
                  << " (" << acm_probe.tier(channel_tier).description
                  << ", SNR=" << std::fixed << std::setprecision(1)
                  << acm_probe.tier(channel_tier).snr_threshold_db + 2.0 << " dB)\n";
    std::cout << "\n";

#ifdef HAVE_CODEC2
    if (survival_sweep) {
        // Survival-mode SNR sweep: tier 0, 4-pass LLR accumulation
        // Channel SNR is per-carrier; 2500 Hz reference adds +17 dB (10*log10(2500/50)).
        std::cout << "\nSurvival mode: tier 0, 50 Hz BPSK, "
                  << fec_code_for_tier(0) << " (rate 1/3), "
                  << passes_for_tier(0) << "-pass LLR soft combining\n";
        std::cout << "FT8 reference threshold: -21 dB (2500 Hz BW)\n"
                  << "4-pass design target:    -23 dB (2500 Hz BW)\n\n";
        std::cout << std::left
                  << std::setw(13) << "SNR(ch dB)"
                  << std::setw(14) << "SNR(2500Hz)"
                  << std::setw(13) << "BER(raw)"
                  << std::setw(13) << "BER(4pass)"
                  << "Status\n"
                  << std::string(62, '-') << "\n";
        static const double snr_pts[] = {-12,-10,-8,-6,-4,-2,0,2};
        for (double snr : snr_pts) {
            double snr_2500 = snr - 17.0;
            auto rf = run_fec_ber_test(0, snr, 40);
            std::string status = (rf.fec_ber < 0.01) ? "DECODE OK"
                               : (rf.fec_ber < 0.10) ? "marginal"
                               : "FAIL";
            std::cout << std::left
                      << std::setw(13) << std::fixed << std::setprecision(1) << snr
                      << std::setw(14) << snr_2500
                      << std::setw(13) << std::scientific << std::setprecision(2) << rf.raw_ber
                      << std::setw(13) << rf.fec_ber
                      << status << "\n";
        }
        std::cout << "\nDone.\n";
        return 0;
    }
#endif

    // Format a bit rate as "X bps" or "X.X kbps"
    auto fmt_bps = [](double bps) -> std::string {
        std::ostringstream oss;
        if (bps >= 1000) oss << std::fixed << std::setprecision(1) << bps/1000.0 << " kbps";
        else             oss << static_cast<int>(bps) << " bps";
        return oss.str();
    };

    if (force_tier >= 0) {
        // Single modem-tier test
        double snr = channel_snr(force_tier);
        auto r = run_ber_test(force_tier, snr, 1000);
        double code_rate = acm_probe.tier(force_tier).rate.value();
        double gross_bps = code_rate > 0.0 ? r.throughput_bps / code_rate : r.throughput_bps;
        std::cout << "Tier " << r.tier << " @ SNR=" << snr << " dB\n";
        std::cout << "  BER(raw)=" << std::scientific << r.ber
                  << "  errors=" << r.bit_errors << "/" << r.bits_tested << "\n";
        std::cout << "  gross=" << fmt_bps(gross_bps)
                  << "  net=" << fmt_bps(r.throughput_bps) << "\n";
#ifdef HAVE_CODEC2
        if (!no_fec) {
            auto rf = run_fec_ber_test(force_tier, snr, 200);
            std::cout << "  BER(FEC)=" << std::scientific << rf.fec_ber
                      << "  code=" << fec_code_for_tier(force_tier)
                      << "  passes=" << rf.num_passes << "\n";
        }
#endif
    } else {
        // Full BER sweep
#ifdef HAVE_CODEC2
        bool show_fec = !no_fec;
#else
        bool show_fec = false;
#endif
        if (show_fec) {
            std::cout << std::left
                      << std::setw(6)  << "Tier"
                      << std::setw(22) << "Description"
                      << std::setw(10) << "SNR(dB)"
                      << std::setw(12) << "BER(raw)"
                      << std::setw(12) << "BER(FEC)"
                      << std::setw(12) << "Gross bps"
                      << std::setw(11) << "Net bps"
                      << "\n";
            std::cout << std::string(85, '-') << "\n";
        } else {
            std::cout << std::left
                      << std::setw(6)  << "Tier"
                      << std::setw(22) << "Description"
                      << std::setw(10) << "SNR(dB)"
                      << std::setw(12) << "BER"
                      << std::setw(12) << "Gross bps"
                      << std::setw(11) << "Net bps"
                      << "\n";
            std::cout << std::string(73, '-') << "\n";
        }

        for (int t = 0; t < acm_probe.num_tiers(); t++) {
            const auto& tier = acm_probe.tier(t);
            double snr = channel_snr(t);
            auto r = run_ber_test(t, snr, 300);

            double code_rate = tier.rate.value();
            double gross_bps = code_rate > 0.0 ? r.throughput_bps / code_rate : r.throughput_bps;

            std::cout << std::left
                      << std::setw(6)  << t
                      << std::setw(22) << tier.description.substr(0,21)
                      << std::setw(10) << std::fixed << std::setprecision(1) << snr
                      << std::setw(12) << std::scientific << std::setprecision(2) << r.ber;

#ifdef HAVE_CODEC2
            if (show_fec) {
                auto rf = run_fec_ber_test(t, snr, 50);
                std::cout << std::setw(12) << std::scientific << std::setprecision(2) << rf.fec_ber;
            }
#endif
            std::cout << std::setw(12) << fmt_bps(gross_bps)
                      << std::setw(11) << fmt_bps(r.throughput_bps) << "\n";
        }

        run_negotiation_test();
    }

    std::cout << "\nDone.\n";
    return 0;
}
