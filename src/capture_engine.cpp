#include "audio_io/capture_engine.hpp"

#include <alsa/asoundlib.h>

#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>

#include "audio_io_interface/msg/audio_event.hpp"
#include "common_cpp/logging/log_interface/log_macro.hpp"

namespace audio_io {
    namespace {

        using EventMsg = audio_io_interface::msg::AudioEvent;

        [[nodiscard]] std::string alsa_error(const std::string& action, const int code) {
            return action + ": " + snd_strerror(code);
        }

    } // namespace

    CaptureEngine::CaptureEngine(std::string device, const uint32_t sample_rate,
                                 const uint16_t channels, const uint32_t period_ms,
                                 const uint32_t buffer_ms) :
        device_(std::move(device)),
        sample_rate_(sample_rate),
        channels_(channels),
        period_ms_(period_ms),
        buffer_ms_(buffer_ms) {
    }

    CaptureEngine::~CaptureEngine() {
        stop();
    }

    void CaptureEngine::start() {
        if (running_.exchange(true)) {
            return;
        }
        thread_ = std::thread([this] {
            run();
        });
    }

    void CaptureEngine::stop() {
        if (!running_.exchange(false)) {
            return;
        }
        if (thread_.joinable()) {
            thread_.join();
        }
        close_device();
    }

    bool CaptureEngine::ok() const noexcept {
        return ok_.load();
    }

    std::string CaptureEngine::error() const {
        std::scoped_lock lock(mutex_);
        return error_;
    }

    bool CaptureEngine::open_device(std::string& error) {
        close_device();
        if (device_.empty()) {
            error = "capture device is empty";
            return false;
        }

        snd_pcm_t* pcm = nullptr;
        int code = snd_pcm_open(&pcm, device_.c_str(), SND_PCM_STREAM_CAPTURE, 0);
        if (code < 0) {
            error = alsa_error("snd_pcm_open(" + device_ + ")", code);
            return false;
        }

        snd_pcm_hw_params_t* params = nullptr;
        snd_pcm_hw_params_alloca(&params);
        code = snd_pcm_hw_params_any(pcm, params);
        if (code >= 0) {
            code = snd_pcm_hw_params_set_access(pcm, params, SND_PCM_ACCESS_RW_INTERLEAVED);
        }
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
            error = alsa_error("configure ALSA capture device " + device_, code);
            snd_pcm_close(pcm);
            return false;
        }
        code = snd_pcm_prepare(pcm);
        if (code < 0) {
            error = alsa_error("snd_pcm_prepare(capture)", code);
            snd_pcm_close(pcm);
            return false;
        }

        pcm_ = pcm;
        return true;
    }

    void CaptureEngine::close_device() {
        if (pcm_ == nullptr) {
            return;
        }
        auto* pcm = static_cast<snd_pcm_t*>(pcm_);
        snd_pcm_drop(pcm);
        snd_pcm_close(pcm);
        pcm_ = nullptr;
    }

    void CaptureEngine::run() {
        const auto frames = static_cast<std::size_t>(
            std::max<uint64_t>(1U, static_cast<uint64_t>(sample_rate_) * period_ms_ / 1000U));
        bool lost_reported = false;

        while (running_.load()) {
            if (pcm_ == nullptr) {
                if (std::string error; !open_device(error)) {
                    ok_.store(false);
                    {
                        std::scoped_lock lock(mutex_);
                        error_ = error;
                        if (!lost_reported) {
                            events_.push_back(
                                AudioIoEvent{.request_id = {},
                                             .type = EventMsg::EVENT_DEVICE_LOST,
                                             .detail = error});
                            LOG_ERROR("audio_io capture device open failed: {}", error);
                            lost_reported = true;
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    continue;
                }
                ok_.store(true);
                {
                    std::scoped_lock lock(mutex_);
                    error_.clear();
                    if (lost_reported) {
                        events_.push_back(
                            AudioIoEvent{.request_id = {},
                                         .type = EventMsg::EVENT_DEVICE_RECOVERED,
                                         .detail = device_});
                        lost_reported = false;
                    }
                }
                LOG_INFO("audio_io capture device opened: {}", device_);
            }

            std::vector<int16_t> chunk(frames);
            auto* pcm = static_cast<snd_pcm_t*>(pcm_);
            const snd_pcm_sframes_t read = snd_pcm_readi(pcm, chunk.data(), frames);
            if (read < 0) {
                if (const int recovered = snd_pcm_recover(pcm, static_cast<int>(read), 1); recovered < 0) {
                    const std::string error = alsa_error("snd_pcm_readi", static_cast<int>(read));
                    ok_.store(false);
                    close_device();
                    std::scoped_lock lock(mutex_);
                    error_ = error;
                    events_.push_back(
                        AudioIoEvent{.request_id = {},
                                     .type = EventMsg::EVENT_DEVICE_LOST,
                                     .detail = error});
                    lost_reported = true;
                    continue;
                }
                continue;
            }
            chunk.resize(static_cast<std::size_t>(read));
            if (chunk.empty()) {
                continue;
            }

            std::scoped_lock lock(mutex_);
            // 只保留最近约 1 秒的数据，避免没有消费者时无限增长。
            constexpr std::size_t kMaxChunks = 100;
            chunks_.push_back(std::move(chunk));
            while (chunks_.size() > kMaxChunks) {
                chunks_.pop_front();
            }
        }
    }

    std::vector<std::vector<int16_t>> CaptureEngine::take_chunks() {
        std::scoped_lock lock(mutex_);
        std::vector<std::vector<int16_t>> result;
        result.reserve(chunks_.size());
        while (!chunks_.empty()) {
            result.push_back(std::move(chunks_.front()));
            chunks_.pop_front();
        }
        return result;
    }

    std::vector<AudioIoEvent> CaptureEngine::take_events() {
        std::scoped_lock lock(mutex_);
        std::vector<AudioIoEvent> result(events_.begin(), events_.end());
        events_.clear();
        return result;
    }

} // namespace audio_io
