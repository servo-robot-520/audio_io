#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "audio_io/alsa_sink.hpp"

namespace audio_io {

    /// 一段文件音频的播放请求。字段与 audio_io_interface/msg/AudioClip 对应。
    struct ClipRequest {
        std::string request_id;
        std::string uri;
        float gain{1.0F};
        bool loop{false};
        uint8_t priority{0};
        uint8_t policy{0};
        std::string group;
        double start_offset{0.0};
        double duration{0.0};
    };

    /// 一段实时 PCM 流的请求。字段与 audio_io_interface/msg/AudioStream 对应。
    struct StreamRequest {
        std::string stream_id;
        std::string encoding;
        uint32_t sample_rate{0};
        uint16_t channels{0};
        uint64_t sequence{0};
        bool eos{false};
        std::vector<uint8_t> data;
        uint8_t priority{0};
        uint8_t policy{0};
        std::string group;
    };

    /// 入队结果。accepted=false 时 error 说明原因（例如 WAV 解码失败）。
    struct ApiResult {
        bool accepted{false};
        std::string request_id;
        std::string error;
    };

    /// 某个 request_id 的播放状态，供 action 反馈与结果使用。
    struct RequestStatus {
        bool found{false};
        bool active{false};
        bool finished{false};
        bool success{true};
        std::string message;
        float position{0.0F};
        float duration{0.0F};
    };

    /// 播放事件，字段与 audio_io_interface/msg/AudioEvent 对应。
    struct AudioIoEvent {
        std::string request_id;
        uint8_t type{0};
        std::string detail;
    };

    /// 播放器状态快照。
    struct PlaybackState {
        bool playback_ok{false};
        bool has_active{false};
        std::string request_id;
        std::string uri;
        float position{0.0F};
        float duration{0.0F};
        uint32_t queue_depth{0};
        uint32_t underruns{0};
    };

    /// 单设备播放引擎。
    ///
    /// 由独立的音频线程按 period_ms 生成 PCM，并写入 AudioSink。ROS 回调只负责
    /// 入队/停止/查询，不直接写 ALSA。这里用一个互斥锁保护 source 列表：
    /// 对 v1 来说临界区只做内存拷贝，远小于 10 ms period；后续若要做硬实时，
    /// 可替换为无锁命令队列。
    class PlaybackEngine {
    public:
        PlaybackEngine(std::unique_ptr<AudioSink> sink, uint32_t sample_rate,
                       uint16_t channels, uint32_t period_ms,
                       uint32_t stream_timeout_ms);
        ~PlaybackEngine();

        PlaybackEngine(const PlaybackEngine&) = delete;
        PlaybackEngine& operator=(const PlaybackEngine&) = delete;

        void start();
        void stop();

        [[nodiscard]] ApiResult enqueue_clip(const ClipRequest& request);
        [[nodiscard]] ApiResult enqueue_stream(const StreamRequest& request);

        /// 取消某个 request_id 的播放。
        void cancel(std::string_view request_id);

        /// 按焦点组或全部停止，返回实际停止的源数量。
        std::uint32_t stop_group(std::string_view group, bool all);

        /// 设置主音量与静音。volume 会被夹到 [0, 1]。
        void set_volume(float volume, bool mute);

        [[nodiscard]] RequestStatus status(std::string_view request_id) const;

        /// 释放一个已完成请求占用的记录。
        void acknowledge(std::string_view request_id);

        [[nodiscard]] std::vector<AudioIoEvent> take_events();

        [[nodiscard]] PlaybackState state() const;

    private:
        struct Source;
        using SourcePtr = std::shared_ptr<Source>;

        void run();
        void mix_block(int16_t* output, std::size_t frames);
        void apply_policy_locked(const Source& incoming);
        void finish_locked(Source& source, bool success, std::string message,
                           uint8_t event_type);
        void push_event_locked(std::string request_id, uint8_t type, std::string detail);

        [[nodiscard]] std::string make_request_id();
        [[nodiscard]] bool decode_clip(const ClipRequest& request, Source& source,
                                       std::string& error) const;
        [[nodiscard]] bool convert_stream(const StreamRequest& request, Source& source,
                                          std::string& error) const;
        [[nodiscard]] static std::string resolve_uri(const std::string& uri,
                                                     std::string& error);

        std::unique_ptr<AudioSink> sink_;
        uint32_t sample_rate_;
        uint16_t channels_;
        uint32_t period_ms_;
        uint32_t stream_timeout_ms_;

        mutable std::mutex mutex_;
        std::vector<SourcePtr> sources_;
        std::deque<AudioIoEvent> events_;
        std::atomic<float> volume_{1.0F};
        std::atomic<bool> mute_{false};
        std::atomic<bool> playback_ok_{false};
        std::atomic<uint32_t> underruns_{0};
        std::atomic<bool> running_{false};
        std::atomic<uint64_t> next_request_id_{1};
        std::thread thread_;
    };

} // namespace audio_io
