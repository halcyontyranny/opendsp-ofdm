// opendsp_ofdm — Adaptive HF OFDM Modem
// Usage: opendsp_ofdm [options]
//   --tx TEXT        Transmit a text message
//   --rx             Receive mode (listen until Ctrl-C)
//   --call SIGN      Your callsign (required)
//   --list-devices   List audio devices
//   --in  N          Audio input  device index (default: system default)
//   --out N          Audio output device index (default: system default)
//   --snr FLOAT      Simulate a specific SNR (offline test, no audio)
//   --tier N         Force an ACM tier (0–8)
//   --verbose        Extra diagnostic output

#include <iostream>
#include <iomanip>
#include <string>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include "ofdm_core.h"
#include "acm.h"
#include "framer.h"
#include "negotiation.h"
#include "audio_io.h"
#ifdef HAVE_CODEC2
#include "fec.h"
#include <memory>
#endif

using namespace opendsp;

static std::atomic<bool> g_running{true};

void sig_handler(int) { g_running = false; }

// ── RX pipeline ───────────────────────────────────────────────────────────────

class RXPipeline {
public:
    RXPipeline(ACMEngine& acm, bool verbose)
        : acm_(acm), verbose_(verbose) {
        update_params();
        zc_ref_ = zadoff_chu(params_.fft_size / 4);
#ifdef HAVE_CODEC2
        fec_ = std::make_unique<FECCodec>(fec_code_for_tier(acm_.state().tier_index));
#endif
    }

    void on_samples(const std::vector<double>& samples) {
        // Accumulate samples
        for (auto s : samples) sample_buf_.push_back(s);

        int sym_len = params_.fft_size + params_.cp_len;

        // Try to find sync once we have enough samples
        int search_len = sym_len * 4;
        while (static_cast<int>(sample_buf_.size()) >= search_len) {
            RealVec window(sample_buf_.begin(),
                           sample_buf_.begin() + search_len);

            // Build ZC time-domain signal for correlation
            // (In a real system the ZC is OFDM-modulated; here we use it directly
            //  as a simplified timing reference)
            RealVec zc_real(zc_ref_.size());
            for (size_t i = 0; i < zc_ref_.size(); i++) zc_real[i] = zc_ref_[i].real();

            int sync_pos = demod_.find_sync(window, zc_ref_);
            if (sync_pos < 0) {
                // No sync — discard half a symbol to slide window
                sample_buf_.erase(sample_buf_.begin(),
                                  sample_buf_.begin() + sym_len / 2);
                return;
            }

            // Found sync — extract one OFDM symbol after sync word
            int data_start = sync_pos + static_cast<int>(zc_ref_.size());
            if (data_start + sym_len > static_cast<int>(sample_buf_.size())) return;

            RealVec sym_samples(sample_buf_.begin() + data_start,
                                sample_buf_.begin() + data_start + sym_len);

            // Demodulate
            CxVec rx_sc = demod_.demodulate_symbol(sym_samples);
            demod_.update_channel_estimate(rx_sc, sym_count_, ch_est_);
            auto llr = demod_.equalise_and_demap(rx_sc, ch_est_);

#ifdef HAVE_CODEC2
            // Accumulate soft LLRs; decode complete FEC blocks as they arrive
            for (auto v : llr) llr_buf_.push_back(static_cast<float>(v));
            while (fec_ && static_cast<int>(llr_buf_.size()) >= fec_->coded_bits()) {
                std::vector<float> block_llr(llr_buf_.begin(),
                                             llr_buf_.begin() + fec_->coded_bits());
                llr_buf_.erase(llr_buf_.begin(),
                               llr_buf_.begin() + fec_->coded_bits());
                std::vector<uint8_t> decoded_bits;
                bool parity_ok = fec_->decode(block_llr, decoded_bits);
                auto decoded_bytes = bits_to_bytes(decoded_bits);
                Framer framer;
                Frame  frame = framer.parse(decoded_bytes);
                if (frame.crc_ok) {
                    std::cout << "\n[RX] " << frame.callsign << ": ";
                    for (auto b : frame.payload)
                        std::cout << static_cast<char>(b);
                    std::cout << " [FEC:" << (parity_ok ? "OK" : "err") << "]\n";
                }
            }
#else
            // Hard decision bits (no FEC build)
            std::vector<uint8_t> bits;
            for (auto l : llr) bits.push_back(l < 0.0 ? 1 : 0);
            (void)bits;
#endif

            // Feed SNR to ACM
            bool tier_changed = acm_.update(ch_est_.snr_db, 0.0 /* BER TBD */);
            if (tier_changed) {
                std::cout << "\n[ACM] " << acm_.status_string() << "\n";
                update_params();
            }

            if (verbose_)
                std::cout << "\r[RX] sym=" << sym_count_
                          << " SNR=" << std::fixed << std::setprecision(1)
                          << ch_est_.snr_db << " dB  "
                          << acm_.status_string() << std::flush;

            sym_count_++;
            sample_buf_.erase(sample_buf_.begin(),
                              sample_buf_.begin() + data_start + sym_len);
        }
    }

private:
    ACMEngine&      acm_;
    bool            verbose_;
    OFDMParams      params_;
    OFDMDemodulator demod_{params_};
    ChannelEstimate ch_est_;
    CxVec           zc_ref_;
    RealVec         sample_buf_;
    int             sym_count_ = 0;
#ifdef HAVE_CODEC2
    std::unique_ptr<FECCodec> fec_;
    std::vector<float>        llr_buf_;
#endif

    void update_params() {
        params_ = acm_.current_params();
        demod_  = OFDMDemodulator(params_);
        zc_ref_ = zadoff_chu(params_.fft_size / 4);
#ifdef HAVE_CODEC2
        fec_ = std::make_unique<FECCodec>(fec_code_for_tier(acm_.state().tier_index));
        llr_buf_.clear();
#endif
    }
};

// ── TX pipeline ───────────────────────────────────────────────────────────────

std::vector<double> build_tx_frame(const std::string& text,
                                    const std::string& callsign,
                                    ACMEngine& acm) {
    OFDMParams    params = acm.current_params();
    OFDMModulator mod(params);
    Framer        framer;

    // Build frame
    Frame f;
    f.header.tier_index  = static_cast<uint8_t>(acm.state().tier_index);
    f.header.max_bw_code = static_cast<uint8_t>(acm.num_tiers() - 1);
    f.header.frame_type  = FrameType::DATA;
    f.callsign           = callsign;
    for (char c : text) f.payload.push_back(static_cast<uint8_t>(c));

    auto raw_bytes = framer.build(f);

    // Convert bytes to bits (MSB first)
    std::vector<uint8_t> bits;
#ifdef HAVE_CODEC2
    bits = bytes_to_bits(raw_bytes);

    // FEC encode: pad to block boundary then encode each block
    FECCodec fec(fec_code_for_tier(acm.state().tier_index));
    auto padded = fec.pad_to_block(bits);
    bits.clear();
    for (int off = 0; off < static_cast<int>(padded.size()); off += fec.data_bits()) {
        std::vector<uint8_t> block(padded.begin() + off,
                                    padded.begin() + off + fec.data_bits());
        auto codeword = fec.encode(block);
        bits.insert(bits.end(), codeword.begin(), codeword.end());
    }
#else
    for (uint8_t byte : raw_bytes)
        for (int b = 7; b >= 0; b--)
            bits.push_back((byte >> b) & 1);
#endif

    // Pad coded bit stream to fill an integer number of OFDM symbols
    int data_sc = params.num_subcarriers - params.num_subcarriers / params.pilot_interval;
    int bits_per_sym = data_sc * params.bits_per_symbol;
    while (static_cast<int>(bits.size()) % bits_per_sym != 0) bits.push_back(0);

    // Preamble: Zadoff-Chu
    CxVec zc = zadoff_chu(params.fft_size / 4);
    RealVec output;
    for (auto& s : zc) output.push_back(s.real() * 0.5);
    // Zero-pad ZC to FFT+CP length
    while (static_cast<int>(output.size()) < params.fft_size + params.cp_len)
        output.push_back(0.0);

    // Pilot OFDM symbol (first symbol is all-pilots for channel estimation)
    CxVec pilot_sc(params.num_subcarriers);
    mod.insert_pilots(pilot_sc, 0);
    auto pilot_samples = mod.modulate_symbol(pilot_sc);
    for (auto s : pilot_samples) output.push_back(s);

    // Data symbols
    int bit_idx = 0;
    while (bit_idx < static_cast<int>(bits.size())) {
        CxVec sc(params.num_subcarriers);
        int local_bit = bit_idx;
        for (int i = 0; i < params.num_subcarriers; i++) {
            if (i % params.pilot_interval == 0) { sc[i] = cx(1,0); continue; }
            uint8_t sym_bits = 0;
            for (int b = 0; b < params.bits_per_symbol; b++) {
                if (local_bit < static_cast<int>(bits.size()))
                    sym_bits |= (bits[local_bit++] & 1) << (params.bits_per_symbol-1-b);
            }
            sc[i] = map_symbol(sym_bits, params.bits_per_symbol);
        }
        mod.insert_pilots(sc, static_cast<int>(output.size() / (params.fft_size + params.cp_len)));
        auto sym_samples = mod.modulate_symbol(sc);
        for (auto s : sym_samples) output.push_back(s);
        bit_idx = local_bit;
    }

    // Normalise to ±0.9
    double peak = 0.0;
    for (auto s : output) peak = std::max(peak, std::abs(s));
    if (peak > 0.0) for (auto& s : output) s *= 0.9 / peak;

    return output;
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    std::signal(SIGINT, sig_handler);

    // Parse args
    std::string callsign = "N0CALL";
    std::string tx_text;
    bool do_rx = false, do_list = false, verbose = false;
    int  in_dev = -1, out_dev = -1, force_tier = -1;
    double force_snr = -999;

    for (int i = 1; i < argc; i++) {
        std::string a(argv[i]);
        if (a == "--call"         && i+1 < argc) callsign   = argv[++i];
        else if (a == "--tx"      && i+1 < argc) tx_text    = argv[++i];
        else if (a == "--rx")                    do_rx      = true;
        else if (a == "--list-devices")          do_list    = true;
        else if (a == "--in"      && i+1 < argc) in_dev     = std::stoi(argv[++i]);
        else if (a == "--out"     && i+1 < argc) out_dev    = std::stoi(argv[++i]);
        else if (a == "--snr"     && i+1 < argc) force_snr  = std::stod(argv[++i]);
        else if (a == "--tier"    && i+1 < argc) force_tier = std::stoi(argv[++i]);
        else if (a == "--verbose")               verbose    = true;
        else if (a == "--help") {
            std::cout <<
"opendsp_ofdm — Adaptive HF OFDM Modem\n"
"\n"
"Usage:\n"
"  opendsp_ofdm --call SIGN --tx TEXT   Transmit a message\n"
"  opendsp_ofdm --call SIGN --rx        Receive mode (Ctrl-C to stop)\n"
"  opendsp_ofdm --list-devices          List audio device indices\n"
"\n"
"Options:\n"
"  --call SIGN      Callsign to embed in frames (default: N0CALL)\n"
"  --tx TEXT        Text message to transmit\n"
"  --rx             Enter receive / decode mode\n"
"  --list-devices   Print available audio input/output device indices\n"
"  --in  N          Audio input device index  (default: system default)\n"
"  --out N          Audio output device index (default: system default)\n"
"  --tier N         Lock ACM to tier N (0–8) instead of adapting\n"
"  --snr F          Offline mode: simulate channel at F dB, no audio I/O\n"
"  --verbose        Print per-symbol SNR and ACM status during RX\n"
"\n"
"ACM tiers  (0 = most robust / lowest rate, 8 = fastest / highest SNR required)\n"
"  Tier  Modulation     BW      Code rate  ~Rate    Min SNR\n"
"  0     BPSK           500 Hz  1/8        30 bps   -20 dB\n"
"  1     BPSK           500 Hz  1/4        60 bps   -12 dB\n"
"  2     BPSK           1 kHz   1/3       177 bps    -8 dB\n"
"  3     QPSK           1.5 kHz 1/3       531 bps    -5 dB\n"
"  4     QPSK           2 kHz   1/2       1.1 kbps   -2 dB\n"
"  5     QPSK           2.5 kHz 2/3       1.8 kbps   +1 dB\n"
"  6     8-PSK          3 kHz   2/3       3.2 kbps   +4 dB\n"
"  7     8-PSK          3.2 kHz 3/4       3.8 kbps   +7 dB\n"
"  8     16-QAM         3.5 kHz 3/4       5.6 kbps  +10 dB\n"
"\n"
"Examples:\n"
"  opendsp_ofdm --call W1AW --tx \"Hello\"\n"
"  opendsp_ofdm --call W1AW --rx --verbose\n"
"  opendsp_ofdm --call W1AW --tx \"Hello\" --snr 5.0   # offline preview\n"
"  opendsp_ofdm --list-devices\n";
            return 0;
        }
    }

    if (do_list) { AudioIO::list_devices(); return 0; }

    ACMEngine acm;
    if (force_tier >= 0) acm.force_tier(force_tier);

    // ── Offline SNR simulation (no audio) ────────────────────────────────────
    if (force_snr > -900) {
        acm.update(force_snr, 0.01);
        std::cout << "Offline mode — " << acm.status_string() << "\n";
        if (!tx_text.empty()) {
            auto samples = build_tx_frame(tx_text, callsign, acm);
            std::cout << "TX frame: " << samples.size() << " samples ("
                      << std::fixed << std::setprecision(2)
                      << static_cast<double>(samples.size()) / SAMPLE_RATE << " s)\n";
            std::cout << "Message: \"" << tx_text << "\" → "
                      << samples.size() * 8 / tx_text.size() << " bits/char overhead\n";
        }
        return 0;
    }

    // ── Live audio ────────────────────────────────────────────────────────────
    AudioIO audio;
    if (!audio.open(in_dev, out_dev)) {
        std::cerr << "Failed to open audio device\n"; return 1;
    }

    if (!tx_text.empty()) {
        // TX mode
        std::cout << "[TX] " << callsign << ": \"" << tx_text << "\"\n";
        std::cout << "[ACM] " << acm.status_string() << "\n";
        auto samples = build_tx_frame(tx_text, callsign, acm);
        audio.set_ptt(true);
        audio.start();
        audio.queue_tx(samples);
        // Wait for TX queue to drain
        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                static_cast<int>(samples.size() / SAMPLE_RATE * 1100)));
        audio.set_ptt(false);
        audio.stop();
        std::cout << "[TX] Done.\n";
    } else if (do_rx) {
        // RX mode
        RXPipeline rx(acm, verbose);
        audio.set_rx_callback([&](const std::vector<double>& s){ rx.on_samples(s); });
        audio.start();
        std::cout << "[RX] Listening — Ctrl-C to stop\n";
        while (g_running)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        audio.stop();
        std::cout << "\n[RX] Stopped.\n";
    } else {
        std::cout << "No mode specified. Use --tx TEXT, --rx, or --list-devices.\n";
        std::cout << "Run with --help for usage.\n";
    }

    return 0;
}
