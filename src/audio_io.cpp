#include "audio_io.h"
#include "ofdm_core.h"
#include <iostream>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace opendsp {

// ── AGC ───────────────────────────────────────────────────────────────────────

AGC::AGC(double target_rms, double attack, double release)
    : target_rms_(target_rms), attack_(attack), release_(release) {}

double AGC::process(std::vector<double>& samples) {
    if (samples.empty()) return gain_;
    // Measure RMS
    double sum_sq = 0.0;
    for (auto s : samples) sum_sq += s * s;
    double rms = std::sqrt(sum_sq / samples.size()) + 1e-12;

    // Desired gain
    double desired = target_rms_ / rms;
    double alpha   = (desired < gain_) ? attack_ : release_;
    gain_ = gain_ * (1.0 - alpha) + desired * alpha;
    gain_ = std::clamp(gain_, 0.01, 100.0);

    for (auto& s : samples) s *= gain_;
    return gain_;
}

// ── DC block ──────────────────────────────────────────────────────────────────

double DCBlock::process(double x) {
    double y = x - x_prev_ + R * y_prev_;
    x_prev_ = x;
    y_prev_ = y;
    return y;
}

// ── AudioIO ───────────────────────────────────────────────────────────────────

AudioIO::AudioIO() : sample_rate_(SAMPLE_RATE), block_size_(256) {
    PaError err = Pa_Initialize();
    if (err != paNoError)
        throw std::runtime_error(std::string("PortAudio init failed: ") + Pa_GetErrorText(err));
}

AudioIO::~AudioIO() {
    stop();
    if (stream_) Pa_CloseStream(stream_);
    Pa_Terminate();
}

void AudioIO::list_devices() {
    int n = Pa_GetDeviceCount();
    std::cout << "PortAudio devices:\n";
    for (int i = 0; i < n; i++) {
        const PaDeviceInfo* info = Pa_GetDeviceInfo(i);
        std::cout << "  [" << i << "] " << info->name
                  << "  in=" << info->maxInputChannels
                  << " out=" << info->maxOutputChannels
                  << " sr=" << info->defaultSampleRate << "\n";
    }
}

bool AudioIO::open(int input_device, int output_device, int block_size) {
    block_size_ = block_size;

    PaStreamParameters in_params{}, out_params{};

    in_params.device                    = (input_device < 0)  ? Pa_GetDefaultInputDevice()  : input_device;
    in_params.channelCount              = 1;
    in_params.sampleFormat              = paFloat32;
    in_params.suggestedLatency          = Pa_GetDeviceInfo(in_params.device)->defaultLowInputLatency;
    in_params.hostApiSpecificStreamInfo = nullptr;

    out_params.device                    = (output_device < 0) ? Pa_GetDefaultOutputDevice() : output_device;
    out_params.channelCount              = 1;
    out_params.sampleFormat              = paFloat32;
    out_params.suggestedLatency          = Pa_GetDeviceInfo(out_params.device)->defaultLowOutputLatency;
    out_params.hostApiSpecificStreamInfo = nullptr;

    PaError err = Pa_OpenStream(
        &stream_,
        &in_params, &out_params,
        sample_rate_,
        static_cast<unsigned long>(block_size_),
        paClipOff,
        &AudioIO::pa_callback,
        this
    );
    if (err != paNoError) {
        std::cerr << "Pa_OpenStream failed: " << Pa_GetErrorText(err) << "\n";
        return false;
    }
    return true;
}

bool AudioIO::start() {
    if (!stream_) return false;
    PaError err = Pa_StartStream(stream_);
    if (err != paNoError) {
        std::cerr << "Pa_StartStream failed: " << Pa_GetErrorText(err) << "\n";
        return false;
    }
    running_ = true;
    return true;
}

void AudioIO::stop() {
    if (running_ && stream_) {
        Pa_StopStream(stream_);
        running_ = false;
    }
}

void AudioIO::queue_tx(const std::vector<double>& samples) {
    std::lock_guard<std::mutex> lock(tx_mutex_);
    for (double s : samples) tx_queue_.push_back(s);
}

void AudioIO::set_ptt(bool tx) {
    ptt_ = tx;
}

// ── PortAudio callback ────────────────────────────────────────────────────────

int AudioIO::pa_callback(const void* in, void* out,
                          unsigned long frames,
                          const PaStreamCallbackTimeInfo*,
                          PaStreamCallbackFlags,
                          void* user_data) {
    return static_cast<AudioIO*>(user_data)->handle_callback(
        static_cast<const float*>(in),
        static_cast<float*>(out),
        frames);
}

int AudioIO::handle_callback(const float* in, float* out, unsigned long frames) {
    // ── RX path ──────────────────────────────────────────────────────────────
    if (in && rx_cb_ && !ptt_) {
        std::vector<double> rx_samples(frames);
        for (unsigned long i = 0; i < frames; i++) {
            rx_samples[i] = dc_block_.process(static_cast<double>(in[i]));
        }
        agc_.process(rx_samples);
        rx_cb_(rx_samples);
    }

    // ── TX path ──────────────────────────────────────────────────────────────
    if (out) {
        std::memset(out, 0, frames * sizeof(float));
        if (ptt_) {
            std::lock_guard<std::mutex> lock(tx_mutex_);
            for (unsigned long i = 0; i < frames; i++) {
                if (!tx_queue_.empty()) {
                    out[i] = static_cast<float>(tx_queue_.front());
                    tx_queue_.pop_front();
                }
            }
        }
    }
    return paContinue;
}

// ── HamlibPTT ─────────────────────────────────────────────────────────────────

#ifdef HAVE_HAMLIB
HamlibPTT::HamlibPTT() {}

HamlibPTT::~HamlibPTT() { close(); }

bool HamlibPTT::open(rig_model_t model, const std::string& port, int baud) {
    rig_ = rig_init(model);
    if (!rig_) { std::cerr << "rig_init failed\n"; return false; }
    strncpy(rig_->state.rigport.pathname, port.c_str(), FILPATHLEN - 1);
    rig_->state.rigport.parm.serial.rate = baud;
    if (rig_open(rig_) != RIG_OK) {
        std::cerr << "rig_open failed\n";
        rig_cleanup(rig_); rig_ = nullptr; return false;
    }
    return true;
}

void HamlibPTT::set_ptt(bool tx) {
    if (rig_) rig_set_ptt(rig_, RIG_VFO_CURR, tx ? RIG_PTT_ON : RIG_PTT_OFF);
}

void HamlibPTT::close() {
    if (rig_) { rig_close(rig_); rig_cleanup(rig_); rig_ = nullptr; }
}
#endif

} // namespace opendsp
