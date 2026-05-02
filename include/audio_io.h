#pragma once
#include <vector>
#include <functional>
#include <atomic>
#include <mutex>
#include <deque>
#include <portaudio.h>

namespace opendsp {

// ── AGC ───────────────────────────────────────────────────────────────────────

class AGC {
public:
    explicit AGC(double target_rms = 0.1, double attack = 0.01, double release = 0.001);
    // Process a block of samples in-place; returns current gain.
    double process(std::vector<double>& samples);
private:
    double target_rms_;
    double attack_;
    double release_;
    double gain_ = 1.0;
};

// ── DC block ─────────────────────────────────────────────────────────────────

class DCBlock {
public:
    double process(double x);
private:
    double x_prev_ = 0.0;
    double y_prev_ = 0.0;
    static constexpr double R = 0.9995;
};

// ── AudioIO ───────────────────────────────────────────────────────────────────
//
// Wraps PortAudio for full-duplex soundcard access at SAMPLE_RATE.
// RX samples arrive via on_rx_block callback (called from audio thread).
// TX samples are queued via queue_tx().

class AudioIO {
public:
    using RxCallback = std::function<void(const std::vector<double>& samples)>;

    AudioIO();
    ~AudioIO();

    // List available devices to stdout.
    static void list_devices();

    // Open the default (or specified) device.
    // block_size: samples per callback (e.g. 256 or 512).
    bool open(int input_device  = -1,
              int output_device = -1,
              int block_size    = 256);

    bool start();
    void stop();
    bool is_running() const { return running_; }

    // Register RX callback (called from audio thread — keep it fast).
    void set_rx_callback(RxCallback cb) { rx_cb_ = std::move(cb); }

    // Queue samples for TX. Thread-safe.
    void queue_tx(const std::vector<double>& samples);

    // PTT control (if Hamlib is available, wraps rig_set_ptt).
    // Without Hamlib, toggles a VOX-style software flag.
    void set_ptt(bool tx);
    bool ptt_active() const { return ptt_; }

    double sample_rate() const { return sample_rate_; }

private:
    PaStream*          stream_   = nullptr;
    std::atomic<bool>  running_  = false;
    std::atomic<bool>  ptt_      = false;
    double             sample_rate_;
    int                block_size_;

    RxCallback         rx_cb_;

    std::mutex              tx_mutex_;
    std::deque<double>      tx_queue_;

    AGC      agc_;
    DCBlock  dc_block_;

    // PortAudio callback (static trampoline)
    static int pa_callback(const void* in, void* out,
                           unsigned long frames,
                           const PaStreamCallbackTimeInfo*,
                           PaStreamCallbackFlags,
                           void* user_data);

    int handle_callback(const float* in, float* out, unsigned long frames);
};

#ifdef HAVE_HAMLIB
// ── HamlibPTT ─────────────────────────────────────────────────────────────────
#include <hamlib/rig.h>
class HamlibPTT {
public:
    HamlibPTT();
    ~HamlibPTT();
    bool open(rig_model_t model, const std::string& port, int baud = 9600);
    void set_ptt(bool tx);
    void close();
private:
    RIG* rig_ = nullptr;
};
#endif

} // namespace opendsp
