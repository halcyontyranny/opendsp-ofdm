#pragma once
#include <vector>
#include <complex>
#include <cstdint>
#include <fftw3.h>

namespace opendsp {

// ── Constants ─────────────────────────────────────────────────────────────────

static constexpr double SAMPLE_RATE     = 8000.0;   // Hz — fits in SSB passband
static constexpr double BASE_FREQ       = 500.0;    // Hz — lower guard edge
static constexpr double MAX_FREQ        = 3500.0;   // Hz — upper guard edge
static constexpr double MIN_SUB_SPACING = 6.25;     // Hz — FT8-proven Doppler tolerance
static constexpr double CP_RATIO        = 0.20;     // 20% cyclic prefix

using cx = std::complex<double>;
using CxVec = std::vector<cx>;
using RealVec = std::vector<double>;

// ── ACM tier parameters (set by ACM engine, consumed by OFDM core) ────────────

struct OFDMParams {
    double  bw_hz;           // Occupied bandwidth (500–3500 Hz)
    double  sub_spacing_hz;  // Subcarrier spacing (6.25, 12.5, or 25 Hz)
    int     num_subcarriers; // = floor(bw_hz / sub_spacing_hz)
    int     fft_size;        // = round(SAMPLE_RATE / sub_spacing_hz)
    int     cp_len;          // = round(fft_size * CP_RATIO)
    int     bits_per_symbol; // 1=BPSK, 2=QPSK, 3=8PSK, 4=16QAM
    int     pilot_interval;  // Insert pilot every N subcarriers
};

OFDMParams make_params(double bw_hz, double sub_spacing_hz, int bits_per_symbol);

// ── Constellation mappers ─────────────────────────────────────────────────────

// Map `bits_per_symbol` bits to a unit-power IQ symbol
cx  map_symbol(uint8_t bits, int bits_per_symbol);

// Soft-decision: return LLR for each bit position
// rx_sym: received (equalised) symbol, noise_var: per-sample noise variance
std::vector<double> demap_symbol_llr(cx rx_sym, int bits_per_symbol, double noise_var);

// Hard decision from LLR
uint8_t llr_to_bits(const std::vector<double>& llr);

// ── Zadoff-Chu sync word ──────────────────────────────────────────────────────

// Generate a Zadoff-Chu sequence of length `n` with root `u`
CxVec zadoff_chu(int n, int u = 25);

// ── OFDMModulator ─────────────────────────────────────────────────────────────

class OFDMModulator {
public:
    explicit OFDMModulator(const OFDMParams& p);
    ~OFDMModulator();
    OFDMModulator(OFDMModulator&&) noexcept;
    OFDMModulator& operator=(OFDMModulator&&) noexcept;
    OFDMModulator(const OFDMModulator&) = delete;
    OFDMModulator& operator=(const OFDMModulator&) = delete;

    // Modulate one OFDM symbol from a vector of complex subcarrier values.
    // Returns time-domain samples (FFT_size + CP_len samples).
    RealVec modulate_symbol(const CxVec& subcarriers) const;

    // Convenience: modulate a full frame of bits.
    // bits[] is packed: bits_per_symbol bits per subcarrier, num_subcarriers subcarriers.
    RealVec modulate_bits(const std::vector<uint8_t>& bits) const;

    // Inject pilot subcarriers into a subcarrier vector (modifies in-place)
    void insert_pilots(CxVec& subcarriers, int symbol_index) const;

    const OFDMParams& params() const { return p_; }

private:
    OFDMParams   p_;
    fftw_plan    ifft_plan_;
    fftw_complex *fft_in_;
    fftw_complex *fft_out_;
};

// ── OFDMDemodulator ───────────────────────────────────────────────────────────

struct ChannelEstimate {
    CxVec   h;           // Per-subcarrier complex channel coefficients
    double  noise_var;   // Estimated noise variance
    double  snr_db;      // Estimated SNR in dB
};

class OFDMDemodulator {
public:
    explicit OFDMDemodulator(const OFDMParams& p);
    ~OFDMDemodulator();
    OFDMDemodulator(OFDMDemodulator&&) noexcept;
    OFDMDemodulator& operator=(OFDMDemodulator&&) noexcept;
    OFDMDemodulator(const OFDMDemodulator&) = delete;
    OFDMDemodulator& operator=(const OFDMDemodulator&) = delete;

    // Remove cyclic prefix and run FFT.
    // input: exactly (fft_size + cp_len) real samples.
    // Returns frequency-domain subcarrier values.
    CxVec demodulate_symbol(const RealVec& samples) const;

    // Extract pilots from a demodulated symbol, update channel estimate.
    void update_channel_estimate(const CxVec& rx_subcarriers,
                                 int symbol_index,
                                 ChannelEstimate& est) const;

    // Equalise and soft-demap a symbol given channel estimate.
    // Returns concatenated LLRs: bits_per_symbol LLRs per subcarrier.
    std::vector<double> equalise_and_demap(const CxVec& rx_subcarriers,
                                           const ChannelEstimate& est) const;

    // Timing sync: search for Zadoff-Chu preamble in a sample buffer.
    // Returns sample offset of start-of-frame, or -1 if not found.
    int find_sync(const RealVec& samples, const CxVec& zc_ref) const;

    const OFDMParams& params() const { return p_; }

private:
    OFDMParams   p_;
    fftw_plan    fft_plan_;
    fftw_complex *fft_in_;
    fftw_complex *fft_out_;

    // Pilot reference values (known at receiver)
    CxVec pilot_refs_;
    void  generate_pilot_refs();
};

} // namespace opendsp
