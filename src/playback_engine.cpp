#include "audio_io/playback_engine.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <format>
#include <limits>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "audio_io/audio_decoder.hpp"
#include "audio_io/audio_format.hpp"
#include "audio_io_interface/msg/audio_clip.hpp"
#include "audio_io_interface/msg/audio_event.hpp"
#include "common_cpp/logging/log_interface/log_macro.hpp"

namespace audio_io {
    namespace {

        using ClipMsg = audio_io_interface::msg::AudioClip;
        using EventMsg = audio_io_interface::msg::AudioEvent;

        [[nodiscard]] uint16_t read_u16(const uint8_t* data) noexcept {
            return static_cast<uint16_t>(data[0]) |
                   static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
        }

        [[nodiscard]] uint32_t read_u32(const uint8_t* data) noexcept {
            return static_cast<uint32_t>(data[0]) |
                   (static_cast<uint32_t>(data[1]) << 8U) |
                   (static_cast<uint32_t>(data[2]) << 16U) |
                   (static_cast<uint32_t>(data[3]) << 24U);
        }

        [[nodiscard]] float clamp_gain(const float value) noexcept {
            return std::clamp(value, 0.0F, 4.0F);
        }

    } // namespace

    struct PlaybackEngine::Source {
        std::string request_id;
        std::string uri;
        std::string group;
        uint8_t priority{ClipMsg::PRIORITY_IDLE};
        uint8_t policy{ClipMsg::POLICY_QUEUE};
        float gain{1.0F};
        bool loop{false};

        // 文件音频：48 kHz 单声道 float；cursor 是当前读取位置。
        std::vector<float> samples;
        std::size_t start_frame{0};
        std::size_t end_frame{0};
        std::size_t cursor{0};

        // 实时流：ROS 回调追加 float 样本，音频线程消费。
        bool stream{false};
        std::deque<float> stream_samples;
        bool eos{false};
        std::chrono::steady_clock::time_point last_data{};

        bool active{true};
        bool started{false};
        bool finished{false};
        bool success{true};
        std::string message;
        uint64_t played_frames{0};
        float duration{0.0F};
    };

    PlaybackEngine::PlaybackEngine(std::unique_ptr<AudioSink> sink,
                                   const uint32_t sample_rate, const uint16_t channels,
                                   const uint32_t period_ms,
                                   const uint32_t stream_timeout_ms) :
        sink_(std::move(sink)),
        sample_rate_(sample_rate),
        channels_(channels),
        period_ms_(period_ms),
        stream_timeout_ms_(stream_timeout_ms) {}

    PlaybackEngine::~PlaybackEngine() {
        stop();
    }

    void PlaybackEngine::start() {
        if (running_.exchange(true)) {
            return;
        }
        thread_ = std::thread([this] { run(); });
    }

    void PlaybackEngine::stop() {
        if (!running_.exchange(false)) {
            return;
        }
        if (thread_.joinable()) {
            thread_.join();
        }
        sink_->close();
        playback_ok_.store(false);
    }

    std::string PlaybackEngine::make_request_id() {
        return std::format("req-{}", next_request_id_.fetch_add(1));
    }

    std::string PlaybackEngine::resolve_uri(const std::string& uri, std::string& error) {
        constexpr std::string_view file_prefix = "file://";
        constexpr std::string_view package_prefix = "package://";

        if (uri.starts_with(file_prefix)) {
            return uri.substr(file_prefix.size());
        }
        if (uri.starts_with(package_prefix)) {
            const std::string rest = uri.substr(package_prefix.size());
            const auto slash = rest.find('/');
            if (slash == std::string::npos || slash == 0 || slash + 1 >= rest.size()) {
                error = "package:// URI must be package://<package>/<path>";
                return {};
            }
            try {
                const auto share = ament_index_cpp::get_package_share_directory(
                    rest.substr(0, slash));
                return (std::filesystem::path(share) / rest.substr(slash + 1)).string();
            }
            catch (const std::exception& exception) {
                error = std::string{"cannot resolve package:// URI: "} + exception.what();
                return {};
            }
        }
        return uri;
    }

    bool PlaybackEngine::decode_clip(const ClipRequest& request, Source& source,
                                     std::string& error) const {
        const std::string path = resolve_uri(request.uri, error);
        if (path.empty()) {
            return false;
        }
        auto decoded = AudioDecoder::decode_file(path);
        if (!decoded) {
            error = decoded.error();
            return false;
        }
        source.samples = to_mono_48k(decoded.value());
        if (source.samples.empty()) {
            error = "decoded audio has no samples";
            return false;
        }
        return true;
    }

    bool PlaybackEngine::convert_stream(const StreamRequest& request, Source& source,
                                         std::string& error) const {
        const std::size_t bytes_per_sample =
            request.encoding == "s16le" ? 2U :
            request.encoding == "f32le" ? 4U : 0U;
        if (bytes_per_sample == 0U) {
            error = "unsupported stream encoding: " + request.encoding;
            return false;
        }
        if (request.channels == 0U || request.sample_rate == 0U) {
            error = "stream channels/sample_rate must be non-zero";
            return false;
        }
        const std::size_t frame_bytes = bytes_per_sample * request.channels;
        if (request.data.size() % frame_bytes != 0U) {
            error = "stream data size is not a whole number of frames";
            return false;
        }

        AudioBuffer buffer;
        buffer.sample_rate = request.sample_rate;
        buffer.channels = request.channels;
        const std::size_t frames = request.data.size() / frame_bytes;
        buffer.samples.resize(frames * request.channels);

        for (std::size_t frame = 0; frame < frames; ++frame) {
            for (std::size_t channel = 0; channel < request.channels; ++channel) {
                const std::size_t offset = frame * frame_bytes +
                                           channel * bytes_per_sample;
                float sample = 0.0F;
                if (request.encoding == "s16le") {
                    sample = static_cast<float>(
                        static_cast<int16_t>(read_u16(request.data.data() + offset))) /
                             32768.0F;
                } else {
                    sample = std::bit_cast<float>(read_u32(request.data.data() + offset));
                }
                buffer.samples[frame * request.channels + channel] = sample;
            }
        }

        const auto mono = to_mono_48k(buffer);
        source.stream_samples.insert(source.stream_samples.end(), mono.begin(), mono.end());
        return true;
    }

    ApiResult PlaybackEngine::enqueue_clip(const ClipRequest& request) {
        const std::string request_id =
            request.request_id.empty() ? make_request_id() : request.request_id;
        auto source = std::make_shared<Source>();
        source->request_id = request_id;
        source->uri = request.uri;
        source->group = request.group;
        source->priority = request.priority;
        source->policy = request.policy;
        source->gain = clamp_gain(request.gain);
        source->loop = request.loop;

        std::string error;
        if (!decode_clip(request, *source, error)) {
            std::scoped_lock lock(mutex_);
            push_event_locked(request_id, EventMsg::EVENT_DECODE_FAILED, error);
            return ApiResult{false, request_id, error};
        }

        source->start_frame = std::min(
            source->samples.size(),
            static_cast<std::size_t>(std::max(0.0, request.start_offset) *
                                     static_cast<double>(sample_rate_)));
        source->cursor = source->start_frame;
        source->end_frame = source->samples.size();
        if (request.duration > 0.0) {
            const auto requested_end = source->start_frame + static_cast<std::size_t>(
                request.duration * static_cast<double>(sample_rate_));
            source->end_frame = std::min(source->end_frame, requested_end);
        }
        if (source->start_frame >= source->end_frame) {
            std::scoped_lock lock(mutex_);
            push_event_locked(request_id, EventMsg::EVENT_DECODE_FAILED,
                              "clip start_offset/duration leaves no samples");
            return ApiResult{false, request_id, "clip start_offset/duration leaves no samples"};
        }
        source->duration = static_cast<float>(source->end_frame - source->start_frame) /
                           static_cast<float>(sample_rate_);

        {
            std::scoped_lock lock(mutex_);
            apply_policy_locked(*source);
            sources_.push_back(std::move(source));
        }
        return ApiResult{true, request_id, {}};
    }

    ApiResult PlaybackEngine::enqueue_stream(const StreamRequest& request) {
        const std::string request_id =
            request.stream_id.empty() ? make_request_id() : request.stream_id;

        Source temp;
        temp.request_id = request_id;
        std::string error;
        if (!convert_stream(request, temp, error)) {
            std::scoped_lock lock(mutex_);
            push_event_locked(request_id, EventMsg::EVENT_DECODE_FAILED, error);
            return ApiResult{false, request_id, error};
        }

        std::scoped_lock lock(mutex_);
        // 同一个 stream_id 结束后再次出现，视为一条新流；否则 TTS 连续复用
        // 固定 stream_id 时会永远追加到已经结束的源上。
        std::erase_if(sources_, [&](const SourcePtr& source) {
            return source->request_id == request_id && source->finished;
        });
        const auto existing = std::find_if(
            sources_.begin(), sources_.end(),
            [&](const SourcePtr& source) { return source->request_id == request_id; });
        if (existing != sources_.end()) {
            auto& source = **existing;
            source.stream_samples.insert(source.stream_samples.end(),
                                         temp.stream_samples.begin(),
                                         temp.stream_samples.end());
            source.eos = request.eos;
            source.last_data = std::chrono::steady_clock::now();
            if (request.eos && source.stream_samples.empty()) {
                finish_locked(source, true, "stream ended", EventMsg::EVENT_FINISHED);
            }
            return ApiResult{true, request_id, {}};
        }

        auto source = std::make_shared<Source>();
        source->request_id = request_id;
        source->group = request.group;
        source->priority = request.priority;
        source->policy = request.policy;
        source->stream = true;
        source->stream_samples = std::move(temp.stream_samples);
        source->eos = request.eos;
        source->last_data = std::chrono::steady_clock::now();
        apply_policy_locked(*source);
        sources_.push_back(std::move(source));
        if (request.eos && sources_.back()->stream_samples.empty()) {
            finish_locked(*sources_.back(), true, "stream ended",
                          EventMsg::EVENT_FINISHED);
        }
        return ApiResult{true, request_id, {}};
    }

    void PlaybackEngine::cancel(const std::string_view request_id) {
        std::scoped_lock lock(mutex_);
        for (const auto& source : sources_) {
            if (source->active && source->request_id == request_id) {
                finish_locked(*source, false, "canceled", EventMsg::EVENT_PREEMPTED);
            }
        }
    }

    std::uint32_t PlaybackEngine::stop_group(const std::string_view group, const bool all) {
        std::scoped_lock lock(mutex_);
        std::uint32_t stopped = 0;
        for (const auto& source : sources_) {
            if (!source->active) {
                continue;
            }
            if (all || (!group.empty() && source->group == group)) {
                finish_locked(*source, false, "stopped", EventMsg::EVENT_FINISHED);
                ++stopped;
            }
        }
        return stopped;
    }

    void PlaybackEngine::set_volume(const float volume, const bool mute) {
        volume_.store(std::clamp(volume, 0.0F, 1.0F));
        mute_.store(mute);
    }

    RequestStatus PlaybackEngine::status(const std::string_view request_id) const {
        std::scoped_lock lock(mutex_);
        for (const auto& source : sources_) {
            if (source->request_id != request_id) {
                continue;
            }
            RequestStatus status;
            status.found = true;
            status.active = source->active;
            status.finished = source->finished;
            status.success = source->success;
            status.message = source->message;
            status.position = static_cast<float>(source->played_frames) /
                              static_cast<float>(sample_rate_);
            status.duration = source->duration;
            return status;
        }
        return {};
    }

    void PlaybackEngine::acknowledge(const std::string_view request_id) {
        std::scoped_lock lock(mutex_);
        std::erase_if(sources_, [&](const SourcePtr& source) {
            return source->request_id == request_id && source->finished;
        });
    }

    std::vector<AudioIoEvent> PlaybackEngine::take_events() {
        std::scoped_lock lock(mutex_);
        std::vector<AudioIoEvent> result(events_.begin(), events_.end());
        events_.clear();
        return result;
    }

    PlaybackState PlaybackEngine::state() const {
        std::scoped_lock lock(mutex_);
        PlaybackState state;
        state.playback_ok = playback_ok_.load();
        state.underruns = underruns_.load();
        for (const auto& source : sources_) {
            if (!source->active) {
                continue;
            }
            if (!source->started) {
                ++state.queue_depth;
                continue;
            }
            if (!state.has_active) {
                state.has_active = true;
                state.request_id = source->request_id;
                state.uri = source->uri;
                state.position = static_cast<float>(source->played_frames) /
                                 static_cast<float>(sample_rate_);
                state.duration = source->duration;
            }
        }
        return state;
    }

    void PlaybackEngine::apply_policy_locked(const Source& incoming) {
        for (const auto& source : sources_) {
            if (!source->active) {
                continue;
            }
            if (incoming.policy == ClipMsg::POLICY_PREEMPT &&
                source->priority <= incoming.priority) {
                finish_locked(*source, false, "preempted", EventMsg::EVENT_PREEMPTED);
            } else if (incoming.policy == ClipMsg::POLICY_REPLACE_SAME_GROUP &&
                       !incoming.group.empty() && source->group == incoming.group) {
                finish_locked(*source, false, "replaced", EventMsg::EVENT_PREEMPTED);
            }
        }
    }

    void PlaybackEngine::finish_locked(Source& source, const bool success,
                                       std::string message, const uint8_t event_type) {
        if (!source.active) {
            return;
        }
        source.active = false;
        source.finished = true;
        source.success = success;
        source.message = std::move(message);
        push_event_locked(source.request_id, event_type, source.message);
    }

    void PlaybackEngine::push_event_locked(std::string request_id, const uint8_t type,
                                           std::string detail) {
        events_.push_back(AudioIoEvent{std::move(request_id), type, std::move(detail)});
        // 事件只是诊断/状态通知，丢旧事件比无限增长更安全。
        constexpr std::size_t kMaxEvents = 256;
        while (events_.size() > kMaxEvents) {
            events_.pop_front();
        }
    }

    void PlaybackEngine::mix_block(int16_t* output, const std::size_t frames) {
        std::scoped_lock lock(mutex_);
        // output 是交错 PCM：每帧有 channels_ 个样本。内部混音始终是单声道，
        // channels_ > 1 时把同一个样本复制到每个输出声道（dual mono），
        // 这不等于立体声内容。
        std::fill(output, output + frames * channels_, static_cast<int16_t>(0));

        bool any_started = false;
        for (const auto& source : sources_) {
            if (source->active && source->started) {
                any_started = true;
                break;
            }
        }

        std::vector<float> mix(frames, 0.0F);
        const auto now = std::chrono::steady_clock::now();

        for (const auto& source : sources_) {
            if (!source->active) {
                continue;
            }

            if (!source->started) {
                // QUEUE 源必须等当前所有已启动源结束；MIX/PREEMPT/REPLACE 立即启动。
                if (source->policy == ClipMsg::POLICY_QUEUE && any_started) {
                    continue;
                }
                source->started = true;
                any_started = true;
                push_event_locked(source->request_id, EventMsg::EVENT_STARTED, "started");
            }

            bool starved = false;
            for (std::size_t i = 0; i < frames; ++i) {
                float sample = 0.0F;
                if (source->stream) {
                    if (!source->stream_samples.empty()) {
                        sample = source->stream_samples.front();
                        source->stream_samples.pop_front();
                    } else if (source->eos) {
                        finish_locked(*source, true, "stream ended",
                                      EventMsg::EVENT_FINISHED);
                        break;
                    } else {
                        // 流暂时没有数据：补静音，但一个 block 只计一次欠载。
                        starved = true;
                    }
                } else {
                    if (source->cursor >= source->end_frame) {
                        if (source->loop && source->end_frame > source->start_frame) {
                            source->cursor = source->start_frame;
                        } else {
                            finish_locked(*source, true, "finished",
                                          EventMsg::EVENT_FINISHED);
                            break;
                        }
                    }
                    sample = source->samples[source->cursor++];
                }
                mix[i] += sample * source->gain;
                ++source->played_frames;
            }
            if (starved) {
                underruns_.fetch_add(1);
            }

            if (source->stream && !source->eos && source->stream_samples.empty() &&
                now - source->last_data >
                    std::chrono::milliseconds(stream_timeout_ms_)) {
                finish_locked(*source, false, "stream timeout",
                              EventMsg::EVENT_FAILED);
            }
        }

        const float master = mute_.load() ? 0.0F : volume_.load();
        for (std::size_t i = 0; i < frames; ++i) {
            const float clipped = std::clamp(mix[i] * master, -1.0F, 1.0F);
            const auto pcm = static_cast<int16_t>(std::lround(clipped * 32767.0F));
            // 单声道源复制到所有输出声道；不是左右独立内容。
            for (std::size_t channel = 0; channel < channels_; ++channel) {
                output[i * channels_ + channel] = pcm;
            }
        }
    }

    void PlaybackEngine::run() {
        const auto frames = static_cast<std::size_t>(
            std::max<uint64_t>(1U, static_cast<uint64_t>(sample_rate_) * period_ms_ / 1000U));
        std::vector<int16_t> block(frames * channels_, 0);
        bool lost_reported = false;

        while (running_.load()) {
            if (!sink_->is_open()) {
                std::string error;
                if (!sink_->open(error)) {
                    if (!lost_reported) {
                        LOG_ERROR("audio_io playback device open failed: {}", error);
                        lost_reported = true;
                    }
                    playback_ok_.store(false);
                    std::scoped_lock lock(mutex_);
                    push_event_locked({}, EventMsg::EVENT_DEVICE_LOST, error);
                    std::this_thread::sleep_for(std::chrono::milliseconds(250));
                    continue;
                }
                playback_ok_.store(true);
                if (lost_reported) {
                    std::scoped_lock lock(mutex_);
                    push_event_locked({}, EventMsg::EVENT_DEVICE_RECOVERED,
                                      sink_->name());
                    lost_reported = false;
                }
                LOG_INFO("audio_io playback device opened: {}", sink_->name());
            }

            mix_block(block.data(), frames);

            std::string error;
            if (!sink_->write(block.data(), frames, error)) {
                LOG_ERROR("audio_io playback write failed: {}", error);
                playback_ok_.store(false);
                sink_->close();
                std::scoped_lock lock(mutex_);
                push_event_locked({}, EventMsg::EVENT_DEVICE_LOST, error);
                lost_reported = true;
            }
        }
    }

} // namespace audio_io
