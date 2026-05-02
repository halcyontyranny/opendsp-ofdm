#include "ofdm_core.h"
#include <cmath>
#include <cassert>
#include <stdexcept>
#include <numeric>
#include <algorithm>

namespace opendsp {

// ── Helpers ───────────────────────────────────────────────────────────────────

static constexpr double PI = M_PI;
static constexpr double TWO_PI = 2.0 * M_PI;

// ── make_params ───────────────────────────────────────────────────────────────

OFDMParams make_params(double bw_hz, double sub_spacing_hz, int bits_per_symbol) {
    OFDMParams p;
    p.bw_hz           = bw_hz;
    p.sub_spacing_hz  = sub_spacing_hz;
    p.num_subcarriers = static_cast<int>(std::floor(bw_hz / sub_spacing_hz));
    p.fft_size        = static_cast<int>(std::round(SAMPLE_RATE / sub_spacing_hz));
    p.cp_len          = static_cast<int>(std::round(p.fft_size * CP_RATIO));
    p.bits_per_symbol = bits_per_symbol;
    p.pilot_interval  = 4;  // every 4th subcarrier is a pilot
    return p;
}

// ── Constellation mappers ─────────────────────────────────────────────────────

cx map_symbol(uint8_t bits, int bps) {
    switch (bps) {
    case 1: { // BPSK: 0→+1, 1→-1
        return (bits & 1) ? cx(-1, 0) : cx(1, 0);
    }
    case 2: { // QPSK Gray coded, unit power
        static const cx qpsk[4] = {
            {  M_SQRT1_2,  M_SQRT1_2 },
            { -M_SQRT1_2,  M_SQRT1_2 },
            {  M_SQRT1_2, -M_SQRT1_2 },
            { -M_SQRT1_2, -M_SQRT1_2 },
        };
        return qpsk[bits & 3];
    }
    case 3: { // 8-PSK Gray coded
        double angle = (bits & 7) * TWO_PI / 8.0;
        // Gray-coded angle permutation: 0,1,3,2,7,6,4,5
        static const int gray8[8] = {0,1,3,2,7,6,4,5};
        angle = gray8[bits & 7] * TWO_PI / 8.0;
        return cx(std::cos(angle), std::sin(angle));
    }
    case 4: { // 16-QAM Gray coded, normalised to unit power
        // Real and imaginary each take 2 bits: {-3,-1,+1,+3}/sqrt(10)
        static const double lev[4] = {-3, -1, 1, 3};
        static const int gray2[4]  = {0, 1, 3, 2};
        double re = lev[gray2[(bits >> 2) & 3]];
        double im = lev[gray2[ bits       & 3]];
        return cx(re, im) / std::sqrt(10.0);
    }
    default:
        throw std::invalid_argument("Unsupported bits_per_symbol");
    }
}

// Minimum-distance soft LLR computation (exact for BPSK/QPSK; approx for higher orders)
std::vector<double> demap_symbol_llr(cx rx, int bps, double noise_var) {
    std::vector<double> llr(bps);
    double inv_2var = 1.0 / (2.0 * noise_var + 1e-12);

    if (bps == 1) {
        // LLR = 2*re(rx) / noise_var  (coherent BPSK)
        llr[0] = 2.0 * rx.real() * inv_2var * noise_var; // simplify: 2*re/var
        llr[0] = 2.0 * rx.real() / (noise_var + 1e-12);
        return llr;
    }
    if (bps == 2) {
        // QPSK: I branch → bit0, Q branch → bit1
        llr[0] = 2.0 * rx.real() / (noise_var + 1e-12);
        llr[1] = 2.0 * rx.imag() / (noise_var + 1e-12);
        return llr;
    }

    // For 8-PSK / 16-QAM: exhaustive max-log approximation
    int M = 1 << bps;
    std::vector<double> dist(M);
    for (int s = 0; s < M; s++) {
        cx ref = map_symbol(static_cast<uint8_t>(s), bps);
        cx diff = rx - ref;
        dist[s] = std::norm(diff);  // |rx - ref|^2
    }
    for (int b = 0; b < bps; b++) {
        double min0 = std::numeric_limits<double>::max();
        double min1 = std::numeric_limits<double>::max();
        for (int s = 0; s < M; s++) {
            if ((s >> (bps - 1 - b)) & 1)
                min1 = std::min(min1, dist[s]);
            else
                min0 = std::min(min0, dist[s]);
        }
        llr[b] = (min1 - min0) * inv_2var;
    }
    return llr;
}

uint8_t llr_to_bits(const std::vector<double>& llr) {
    uint8_t bits = 0;
    for (size_t i = 0; i < llr.size(); i++) {
        if (llr[i] < 0.0)
            bits |= (1 << (llr.size() - 1 - i));
    }
    return bits;
}

// ── Zadoff-Chu ────────────────────────────────────────────────────────────────

CxVec zadoff_chu(int n, int u) {
    CxVec zc(n);
    for (int k = 0; k < n; k++) {
        double phase = -PI * u * k * (k + 1) / n;
        zc[k] = cx(std::cos(phase), std::sin(phase));
    }
    return zc;
}

// ── OFDMModulator ─────────────────────────────────────────────────────────────

OFDMModulator::OFDMModulator(const OFDMParams& p) : p_(p) {
    fft_in_  = fftw_alloc_complex(p_.fft_size);
    fft_out_ = fftw_alloc_complex(p_.fft_size);
    // IFFT: sign = +1 in FFTW convention is FFTW_BACKWARD
    ifft_plan_ = fftw_plan_dft_1d(p_.fft_size, fft_in_, fft_out_,
                                   FFTW_BACKWARD, FFTW_MEASURE);
}

OFDMModulator::~OFDMModulator() {
    fftw_destroy_plan(ifft_plan_);
    fftw_free(fft_in_);
    fftw_free(fft_out_);
}

void OFDMModulator::insert_pilots(CxVec& sc, int sym_idx) const {
    // BPSK pilots derived from a PN sequence seeded by symbol index
    uint32_t seed = static_cast<uint32_t>(sym_idx) * 1664525u + 1013904223u;
    for (int i = 0; i < p_.num_subcarriers; i += p_.pilot_interval) {
        seed = seed * 1664525u + 1013904223u;
        sc[i] = ((seed >> 16) & 1) ? cx(1, 0) : cx(-1, 0);
    }
}

RealVec OFDMModulator::modulate_symbol(const CxVec& subcarriers) const {
    assert(static_cast<int>(subcarriers.size()) == p_.num_subcarriers);

    // Zero the FFT input
    for (int i = 0; i < p_.fft_size; i++) {
        fft_in_[i][0] = 0.0;
        fft_in_[i][1] = 0.0;
    }

    // Map subcarriers to FFT bins starting at bin corresponding to BASE_FREQ
    int base_bin = static_cast<int>(std::round(BASE_FREQ / p_.sub_spacing_hz));
    for (int i = 0; i < p_.num_subcarriers; i++) {
        int bin = base_bin + i;
        fft_in_[bin][0] = subcarriers[i].real();
        fft_in_[bin][1] = subcarriers[i].imag();
    }

    fftw_execute(ifft_plan_);

    // Normalise
    double norm = 1.0 / p_.fft_size;

    // Build output: CP (tail of IFFT) + IFFT output
    RealVec out(p_.fft_size + p_.cp_len);
    for (int i = 0; i < p_.cp_len; i++) {
        int src = p_.fft_size - p_.cp_len + i;
        out[i] = fft_out_[src][0] * norm;
    }
    for (int i = 0; i < p_.fft_size; i++) {
        out[p_.cp_len + i] = fft_out_[i][0] * norm;
    }
    return out;
}

RealVec OFDMModulator::modulate_bits(const std::vector<uint8_t>& bits) const {
    // Pack bits into subcarrier symbols
    CxVec sc(p_.num_subcarriers);
    int bit_idx = 0;
    for (int i = 0; i < p_.num_subcarriers; i++) {
        uint8_t sym_bits = 0;
        for (int b = 0; b < p_.bits_per_symbol; b++) {
            if (bit_idx < static_cast<int>(bits.size()))
                sym_bits |= (bits[bit_idx++] & 1) << (p_.bits_per_symbol - 1 - b);
        }
        sc[i] = map_symbol(sym_bits, p_.bits_per_symbol);
    }
    return modulate_symbol(sc);
}

// ── OFDMDemodulator ───────────────────────────────────────────────────────────

OFDMDemodulator::OFDMDemodulator(const OFDMParams& p) : p_(p) {
    fft_in_  = fftw_alloc_complex(p_.fft_size);
    fft_out_ = fftw_alloc_complex(p_.fft_size);
    fft_plan_ = fftw_plan_dft_1d(p_.fft_size, fft_in_, fft_out_,
                                  FFTW_FORWARD, FFTW_MEASURE);
    generate_pilot_refs();
}

OFDMDemodulator::~OFDMDemodulator() {
    fftw_destroy_plan(fft_plan_);
    fftw_free(fft_in_);
    fftw_free(fft_out_);
}

void OFDMDemodulator::generate_pilot_refs() {
    pilot_refs_.resize(p_.num_subcarriers / p_.pilot_interval + 1);
    uint32_t seed = 0;
    for (int i = 0, pi = 0; i < p_.num_subcarriers; i += p_.pilot_interval, pi++) {
        seed = seed * 1664525u + 1013904223u;
        pilot_refs_[pi] = ((seed >> 16) & 1) ? cx(1, 0) : cx(-1, 0);
    }
}

CxVec OFDMDemodulator::demodulate_symbol(const RealVec& samples) const {
    assert(static_cast<int>(samples.size()) == p_.fft_size + p_.cp_len);

    // Skip cyclic prefix
    for (int i = 0; i < p_.fft_size; i++) {
        fft_in_[i][0] = samples[p_.cp_len + i];
        fft_in_[i][1] = 0.0;
    }
    fftw_execute(fft_plan_);

    int base_bin = static_cast<int>(std::round(BASE_FREQ / p_.sub_spacing_hz));
    CxVec sc(p_.num_subcarriers);
    for (int i = 0; i < p_.num_subcarriers; i++) {
        int bin = base_bin + i;
        sc[i] = cx(fft_out_[bin][0], fft_out_[bin][1]);
    }
    return sc;
}

void OFDMDemodulator::update_channel_estimate(const CxVec& rx_sc,
                                               int sym_idx,
                                               ChannelEstimate& est) const {
    if (est.h.size() != static_cast<size_t>(p_.num_subcarriers))
        est.h.assign(p_.num_subcarriers, cx(1, 0));

    // Least-squares estimate at pilot positions
    uint32_t seed = static_cast<uint32_t>(sym_idx) * 1664525u + 1013904223u;
    std::vector<std::pair<int,cx>> pilots; // {subcarrier_index, h_estimate}
    for (int i = 0; i < p_.num_subcarriers; i += p_.pilot_interval) {
        seed = seed * 1664525u + 1013904223u;
        cx pilot_ref = ((seed >> 16) & 1) ? cx(1, 0) : cx(-1, 0);
        cx h_ls = rx_sc[i] / pilot_ref;
        pilots.push_back({i, h_ls});
    }

    // Linear interpolation between pilots
    for (size_t pi = 0; pi + 1 < pilots.size(); pi++) {
        int   i0 = pilots[pi].first,   i1 = pilots[pi+1].first;
        cx    h0 = pilots[pi].second,  h1 = pilots[pi+1].second;
        for (int i = i0; i < i1; i++) {
            double t = static_cast<double>(i - i0) / (i1 - i0);
            est.h[i] = h0 * (1.0 - t) + h1 * t;
        }
    }
    // Extrapolate last pilot to end
    if (!pilots.empty())
        for (int i = pilots.back().first; i < p_.num_subcarriers; i++)
            est.h[i] = pilots.back().second;

    // Noise variance: residual at pilot positions after division
    double noise_sum = 0.0;
    int    noise_cnt = 0;
    for (auto& [idx, h_est] : pilots) {
        cx residual = rx_sc[idx] / (h_est + cx(1e-12, 0)) - cx(1, 0); // should be ≈0
        noise_sum += std::norm(residual);
        noise_cnt++;
    }
    est.noise_var = noise_cnt > 0 ? noise_sum / noise_cnt : 1e-3;

    // Signal power (mean over pilots)
    double sig_pwr = 0.0;
    for (auto& [idx, h_est] : pilots)
        sig_pwr += std::norm(h_est);
    sig_pwr = noise_cnt > 0 ? sig_pwr / noise_cnt : 1.0;

    double snr_linear = sig_pwr / (est.noise_var + 1e-12);
    est.snr_db = 10.0 * std::log10(snr_linear + 1e-12);
}

std::vector<double> OFDMDemodulator::equalise_and_demap(const CxVec& rx_sc,
                                                          const ChannelEstimate& est) const {
    std::vector<double> all_llr;
    all_llr.reserve(p_.num_subcarriers * p_.bits_per_symbol);

    for (int i = 0; i < p_.num_subcarriers; i++) {
        // Skip pilot subcarriers
        if (i % p_.pilot_interval == 0) continue;

        // MMSE equalisation: ĥ* / (|ĥ|² + noise_var)
        cx h = est.h[i];
        double h_sq = std::norm(h);
        cx eq_coeff = std::conj(h) / (h_sq + est.noise_var + 1e-12);
        cx eq_sym   = rx_sc[i] * eq_coeff;

        // Effective noise variance after equalisation
        double eq_noise = est.noise_var / (h_sq + 1e-12);

        auto llr = demap_symbol_llr(eq_sym, p_.bits_per_symbol, eq_noise);
        for (auto v : llr) all_llr.push_back(v);
    }
    return all_llr;
}

int OFDMDemodulator::find_sync(const RealVec& samples, const CxVec& zc_ref) const {
    int zc_len = static_cast<int>(zc_ref.size());
    if (static_cast<int>(samples.size()) < zc_len) return -1;

    double best_corr = 0.0;
    int    best_pos  = -1;

    // Compute |cross-correlation|² at each lag
    for (int lag = 0; lag <= static_cast<int>(samples.size()) - zc_len; lag++) {
        cx corr = 0;
        for (int k = 0; k < zc_len; k++)
            corr += samples[lag + k] * std::conj(zc_ref[k]);
        double c2 = std::norm(corr);
        if (c2 > best_corr) {
            best_corr = c2;
            best_pos  = lag;
        }
    }

    // Threshold: peak must be significantly above average
    double avg = 0.0;
    for (auto s : samples) avg += s * s;
    avg /= samples.size();
    double threshold = avg * zc_len * 4.0;  // empirical

    return (best_corr > threshold) ? best_pos : -1;
}

} // namespace opendsp
