#include "audio_io/alsa_sink.hpp"

#include <alsa/asoundlib.h>

#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>

namespace audio_io {
    namespace {

        [[nodiscard]] std::string alsa_error(const std::string& action, const int code) {
            return action + ": " + snd_strerror(code);
        }

    } // namespace

    AlsaSink::AlsaSink(std::string device, const uint32_t sample_rate,
                       const uint16_t channels, const uint32_t period_ms,
                       const uint32_t buffer_ms) :
        device_(std::move(device)),
        sample_rate_(sample_rate),
        channels_(channels),
        period_ms_(period_ms),
        buffer_ms_(buffer_ms) {
    }

    AlsaSink::~AlsaSink() {
        close();
    }

    bool AlsaSink::open(std::string& error) {
        close();
        if (device_.empty()) {
            error = "playback device is empty";
            return false;
        }

        snd_pcm_t* pcm = nullptr;
        int code = snd_pcm_open(&pcm, device_.c_str(), SND_PCM_STREAM_PLAYBACK, 0);
        if (code < 0) {
            error = alsa_error("snd_pcm_open(" + device_ + ")", code);
            return false;
        }

        // hw_params 描述硬件真正要跑的格式。这里请求 S16_LE、单声道、48 kHz；
        // `plughw:` 会在硬件不支持时自动插入 plug 转换层。
        snd_pcm_hw_params_t* params = nullptr;
        snd_pcm_hw_params_alloca(&params);
        code = snd_pcm_hw_params_any(pcm, params);
        if (code < 0) {
            error = alsa_error("snd_pcm_hw_params_any", code);
            snd_pcm_close(pcm);
            return false;
        }

        code = snd_pcm_hw_params_set_access(pcm, params, SND_PCM_ACCESS_RW_INTERLEAVED);
        if (code >= 0) {
            code = snd_pcm_hw_params_set_format(pcm, params, SND_PCM_FORMAT_S16_LE);
        }
        if (code >= 0) {
            code = snd_pcm_hw_params_set_channels(pcm, params, channels_);
        }
        if (code >= 0) {
            unsigned int rate = sample_rate_;
            code = snd_pcm_hw_params_set_rate_near(pcm, params, &rate, nullptr);
        }
        if (code >= 0) {
            // period 是 ALSA 一次中断/唤醒处理的最小块；buffer 是环形缓冲总长度。
            // period 越大越省 CPU、延迟越高；buffer 太小会更容易 underrun。
            auto period = static_cast<snd_pcm_uframes_t>(
                static_cast<uint64_t>(sample_rate_) * period_ms_ / 1000U);
            code = snd_pcm_hw_params_set_period_size_near(pcm, params, &period, nullptr);
        }
        if (code >= 0) {
            auto buffer = static_cast<snd_pcm_uframes_t>(
                static_cast<uint64_t>(sample_rate_) * buffer_ms_ / 1000U);
            code = snd_pcm_hw_params_set_buffer_size_near(pcm, params, &buffer);
        }
        if (code >= 0) {
            code = snd_pcm_hw_params(pcm, params);
        }
        if (code < 0) {
            error = alsa_error("configure ALSA playback device " + device_, code);
            snd_pcm_close(pcm);
            return false;
        }

        code = snd_pcm_prepare(pcm);
        if (code < 0) {
            error = alsa_error("snd_pcm_prepare", code);
            snd_pcm_close(pcm);
            return false;
        }

        pcm_ = pcm;
        return true;
    }

    void AlsaSink::close() {
        if (pcm_ == nullptr) {
            return;
        }
        auto* pcm = static_cast<snd_pcm_t*>(pcm_);
        // drop 立即丢弃尚未播放的样本，避免关闭时阻塞在 drain 上。
        snd_pcm_drop(pcm);
        snd_pcm_close(pcm);
        pcm_ = nullptr;
    }

    bool AlsaSink::is_open() const noexcept {
        return pcm_ != nullptr;
    }

    bool AlsaSink::write(const int16_t* samples, const std::size_t frames, std::string& error) {
        auto* pcm = static_cast<snd_pcm_t*>(pcm_);
        if (pcm == nullptr) {
            error = "ALSA playback device is not open";
            return false;
        }

        std::size_t written = 0;
        while (written < frames) {
            // samples 是交错 PCM：第 written 帧从 samples[written * channels] 开始。
            const snd_pcm_sframes_t result = snd_pcm_writei(
                pcm, samples + written * channels_, frames - written);
            if (result < 0) {
                // EPIPE 表示 underrun：应用写得太慢，设备环形缓冲已经空了。
                // snd_pcm_recover 会 prepare 设备，让我们可以继续写。
                if (const int recovered = snd_pcm_recover(pcm, static_cast<int>(result), 1); recovered < 0) {
                    error = alsa_error("snd_pcm_writei", static_cast<int>(result));
                    return false;
                }
                continue;
            }
            if (result == 0) {
                snd_pcm_wait(pcm, 100);
                continue;
            }
            written += static_cast<std::size_t>(result);
        }
        return true;
    }

    std::string AlsaSink::name() const {
        return device_;
    }

    NullSink::NullSink(const uint32_t sample_rate) :
        sample_rate_(sample_rate) {
    }

    bool NullSink::open(std::string& error) {
        open_ = true;
        error.clear();
        return true;
    }

    void NullSink::close() {
        open_ = false;
    }

    bool NullSink::is_open() const noexcept {
        return open_;
    }

    bool NullSink::write(const int16_t* /*samples*/, const std::size_t frames,
                         std::string& error) {
        if (!open_) {
            error = "null sink is not open";
            return false;
        }
        if (frames > 0 && sample_rate_ > 0) {
            const double seconds = static_cast<double>(frames) /
                static_cast<double>(sample_rate_);
            std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
        }
        return true;
    }

    std::string NullSink::name() const {
        return "null";
    }

} // namespace audio_io
