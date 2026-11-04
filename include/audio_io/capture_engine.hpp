#pragma once

#include <atomic>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "audio_io/playback_engine.hpp"

namespace audio_io {

    /// 麦克风采集引擎。
    ///
    /// 采集线程只做 ALSA readi 和入队，不发布 ROS 消息；节点定时器再取走 chunk
    /// 并发布 AudioStream。这样 DDS 反压不会阻塞 ALSA 读取。
    class CaptureEngine {
    public:
        CaptureEngine(std::string device, uint32_t sample_rate, uint16_t channels,
                      uint32_t period_ms, uint32_t buffer_ms);
        ~CaptureEngine();

        CaptureEngine(const CaptureEngine&) = delete;
        CaptureEngine& operator=(const CaptureEngine&) = delete;

        void start();
        void stop();

        [[nodiscard]] bool ok() const noexcept;
        [[nodiscard]] std::string error() const;

        /// 取走自上次调用以来采集到的 PCM 块，每块为 S16_LE 单声道。
        [[nodiscard]] std::vector<std::vector<int16_t>> take_chunks();

        /// 取走采集设备事件（DEVICE_LOST / DEVICE_RECOVERED）。
        [[nodiscard]] std::vector<AudioIoEvent> take_events();

    private:
        void run();
        bool open_device(std::string& error);
        void close_device();

        std::string device_;
        uint32_t sample_rate_;
        uint16_t channels_;
        uint32_t period_ms_;
        uint32_t buffer_ms_;

        void* pcm_{nullptr};
        std::atomic<bool> running_{false};
        std::atomic<bool> ok_{false};
        std::thread thread_;

        mutable std::mutex mutex_;
        std::deque<std::vector<int16_t>> chunks_;
        std::deque<AudioIoEvent> events_;
        std::string error_;
    };

} // namespace audio_io
