#include "audio_io/audio_io_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <format>
#include <stdexcept>
#include <utility>
#include <vector>

#include "audio_io_interface/ros_msg.hpp"
#include "common_cpp/logging/log_interface/log_macro.hpp"

namespace audio_io {
    namespace {

        using AudioClipMsg = audio_io_interface::msg::AudioClip;
        using AudioEventMsg = audio_io_interface::msg::AudioEvent;
        using AudioStateMsg = audio_io_interface::msg::AudioState;
        using AudioStreamMsg = audio_io_interface::msg::AudioStream;

        template <typename T>
        [[nodiscard]] T get_or_declare_parameter(const rclcpp::Node::SharedPtr& node,
                                                 const std::string& name,
                                                 const T& default_value) {
            if (!node->has_parameter(name)) {
                node->declare_parameter<T>(name, default_value);
            }
            return node->get_parameter(name).get_value<T>();
        }

    } // namespace

    AudioIoNode::AudioIoNode(rclcpp::Node::SharedPtr node) :
        node_(std::move(node)) {
        if (!node_) {
            throw std::runtime_error("AudioIoNode requires a non-null rclcpp::Node");
        }

        declare_parameters();

        if (backend_ == "null") {
            capture_enabled_ = false;
            playback_device_ = "null";
            playback_ = std::make_unique<PlaybackEngine>(
                std::make_unique<NullSink>(playback_rate_),
                playback_rate_, playback_channels_, playback_period_ms_,
                stream_timeout_ms_);
        }
        else {
            playback_ = std::make_unique<PlaybackEngine>(
                std::make_unique<AlsaSink>(playback_device_, playback_rate_,
                                           playback_channels_, playback_period_ms_,
                                           playback_buffer_ms_),
                playback_rate_, playback_channels_, playback_period_ms_,
                stream_timeout_ms_);
        }

        if (capture_enabled_) {
            if (capture_device_.empty()) {
                capture_device_ = playback_device_;
            }
            capture_ = std::make_unique<CaptureEngine>(
                capture_device_, capture_rate_, capture_channels_,
                capture_period_ms_, capture_buffer_ms_);
        }

        playback_->set_volume(
            static_cast<float>(get_or_declare_parameter<double>(
                node_, "volume", 1.0)),
            get_or_declare_parameter<bool>(node_, "mute", false));

        create_interfaces();
        LOG_INFO("audio_io initialized: backend={} playback={} capture={}",
                 backend_, playback_device_,
                 capture_ ? capture_device_ : std::string{"disabled"});
    }

    AudioIoNode::~AudioIoNode() {
        stop();
        // 外部节点可能比本对象活得更久；析构前移除所有引用 this 的回调。
        play_subscription_.reset();
        stream_subscription_.reset();
        stop_service_.reset();
        volume_service_.reset();
        action_server_.reset();
        state_publisher_.reset();
        event_publisher_.reset();
        capture_publisher_.reset();
    }

    void AudioIoNode::declare_parameters() {
        backend_ = get_or_declare_parameter<std::string>(node_, "backend", "alsa");
        if (backend_ != "alsa" && backend_ != "null") {
            throw std::runtime_error("audio_io backend must be 'alsa' or 'null'");
        }

        playback_device_ = get_or_declare_parameter<std::string>(
            node_, "playback.device", "plughw:aic3104");
        playback_rate_ = static_cast<uint32_t>(get_or_declare_parameter<int>(
            node_, "playback.rate", 48000));
        // 这是 ALSA 输出声道数，不是“内容是否立体声”。内部混音始终是单声道：
        // channels=2 表示把同一个单声道复制到 L/R（dual mono），不是立体声；
        // channels=1 只是以单声道 PCM 打开设备，最终是否两边喇叭都响由 ALSA
        // plughw、codec 路由和接线决定，不能理解为只左/右。
        playback_channels_ = static_cast<uint16_t>(get_or_declare_parameter<int>(
            node_, "playback.channels", 1));
        playback_period_ms_ = static_cast<uint32_t>(get_or_declare_parameter<int>(
            node_, "playback.period_ms", 10));
        playback_buffer_ms_ = static_cast<uint32_t>(get_or_declare_parameter<int>(
            node_, "playback.buffer_ms", 40));

        capture_enabled_ = get_or_declare_parameter<bool>(
            node_, "capture.enabled", true);
        capture_device_ = get_or_declare_parameter<std::string>(
            node_, "capture.device", "");
        capture_rate_ = static_cast<uint32_t>(get_or_declare_parameter<int>(
            node_, "capture.rate", 48000));
        capture_channels_ = static_cast<uint16_t>(get_or_declare_parameter<int>(
            node_, "capture.channels", 1));
        capture_period_ms_ = static_cast<uint32_t>(get_or_declare_parameter<int>(
            node_, "capture.period_ms", 10));
        capture_buffer_ms_ = static_cast<uint32_t>(get_or_declare_parameter<int>(
            node_, "capture.buffer_ms", 40));

        stream_timeout_ms_ = static_cast<uint32_t>(get_or_declare_parameter<int>(
            node_, "stream.timeout_ms", 200));
        frame_id_ = get_or_declare_parameter<std::string>(node_, "frame_id", "audio");

        if (playback_rate_ < 8000U || playback_rate_ > 192000U) {
            throw std::runtime_error("playback.rate must be in [8000, 192000]");
        }
        if (capture_rate_ < 8000U || capture_rate_ > 192000U) {
            throw std::runtime_error("capture.rate must be in [8000, 192000]");
        }
        if (playback_channels_ == 0U || playback_channels_ > 2U ||
            capture_channels_ == 0U || capture_channels_ > 2U) {
            throw std::runtime_error("audio channels must be 1 or 2");
        }
        if (playback_period_ms_ == 0U || playback_buffer_ms_ < playback_period_ms_ ||
            capture_period_ms_ == 0U || capture_buffer_ms_ < capture_period_ms_) {
            throw std::runtime_error(
                "audio buffer_ms must be >= period_ms and both must be non-zero");
        }
        if (stream_timeout_ms_ < 50U || stream_timeout_ms_ > 5000U) {
            throw std::runtime_error("stream.timeout_ms must be in [50, 5000]");
        }

        if (const auto volume = get_or_declare_parameter<double>(node_, "volume", 1.0);
            volume < 0.0 || volume > 1.0) {
            throw std::runtime_error("volume must be in [0.0, 1.0]");
        }
    }

    void AudioIoNode::create_interfaces() {
        const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        const auto event_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
        const auto play_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        const auto stream_qos = rclcpp::QoS(rclcpp::KeepLast(2)).best_effort();
        const auto capture_qos = rclcpp::QoS(rclcpp::KeepLast(2)).best_effort();

        state_publisher_ = node_->create_publisher<AudioStateMsg>(
            std::string{audio_io_interface::kPrivateTopicState}, state_qos);
        event_publisher_ = node_->create_publisher<AudioEventMsg>(
            std::string{audio_io_interface::kPrivateTopicEvent}, event_qos);
        capture_publisher_ = node_->create_publisher<AudioStreamMsg>(
            std::string{audio_io_interface::kPrivateTopicCapture}, capture_qos);

        play_subscription_ = node_->create_subscription<AudioClipMsg>(
            std::string{audio_io_interface::kPrivateTopicPlay}, play_qos,
            [this](const AudioClipMsg::SharedPtr message) {
                handle_play(message);
            });
        stream_subscription_ = node_->create_subscription<AudioStreamMsg>(
            std::string{audio_io_interface::kPrivateTopicStream}, stream_qos,
            [this](const AudioStreamMsg::SharedPtr message) {
                handle_stream(message);
            });

        stop_service_ = node_->create_service<audio_io_interface::srv::StopAudio>(
            std::string{audio_io_interface::kPrivateServiceStop},
            [this](const std::shared_ptr<audio_io_interface::srv::StopAudio::Request>& request,
                   const std::shared_ptr<audio_io_interface::srv::StopAudio::Response>& response) {
                handle_stop(request, response);
            });
        volume_service_ = node_->create_service<audio_io_interface::srv::SetVolume>(
            std::string{audio_io_interface::kPrivateServiceSetVolume},
            [this](const std::shared_ptr<audio_io_interface::srv::SetVolume::Request>& request,
                   const std::shared_ptr<audio_io_interface::srv::SetVolume::Response>& response) {
                handle_set_volume(request, response);
            });

        action_server_ = rclcpp_action::create_server<PlayClipAction>(
            node_, std::string{audio_io_interface::kPrivateActionPlayClip},
            [this](const rclcpp_action::GoalUUID& uuid,
                   const std::shared_ptr<const PlayClipAction::Goal>& goal) {
                return handle_goal(uuid, goal);
            },
            [this](const std::shared_ptr<GoalHandlePlayClip>& goal_handle) {
                return handle_cancel(goal_handle);
            },
            [this](const std::shared_ptr<GoalHandlePlayClip>& goal_handle) {
                handle_accepted(goal_handle);
            });
    }

    void AudioIoNode::start() {
        if (started_) {
            return;
        }
        playback_->start();
        if (capture_) {
            capture_->start();
        }
        timer_ = node_->create_wall_timer(std::chrono::milliseconds(100),
                                          [this] {
                                              poll();
                                          });
        // 采集 chunk 通常每 10~20 ms 到达；独立定时器发布，避免状态定时器把
        // 多个 chunk 攒到一次 burst 里，给下游增加无谓延迟。
        capture_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                                  [this] {
                                                      publish_capture();
                                                  });
        started_ = true;
        LOG_INFO("audio_io started");
    }

    void AudioIoNode::stop() {
        if (!started_) {
            return;
        }
        publish_capture_eos();
        if (timer_) {
            timer_->cancel();
            timer_.reset();
        }
        if (capture_timer_) {
            capture_timer_->cancel();
            capture_timer_.reset();
        }
        if (capture_) {
            capture_->stop();
        }
        playback_->stop();
        started_ = false;
        LOG_INFO("audio_io stopped");
    }

    void AudioIoNode::handle_play(const AudioClipMsg::SharedPtr& message) const {
        ClipRequest request;
        request.request_id = message->request_id;
        request.uri = message->uri;
        request.gain = message->gain;
        request.loop = message->loop;
        request.priority = message->priority;
        request.policy = message->policy;
        request.group = message->group;
        request.start_offset = message->start_offset;
        request.duration = message->duration;

        if (const auto result = playback_->enqueue_clip(request); !result.accepted) {
            LOG_ERROR("audio_io play request rejected: {}", result.error);
        }
    }

    void AudioIoNode::handle_stream(const AudioStreamMsg::SharedPtr& message) const {
        LOG_DEBUG("audio_io stream received: id={} bytes={} eos={}",
                  message->stream_id, message->data.size(), message->eos);
        StreamRequest request;
        request.stream_id = message->stream_id.empty() ? "default" : message->stream_id;
        request.encoding = message->encoding;
        request.sample_rate = message->sample_rate;
        request.channels = message->channels;
        request.sequence = message->sequence;
        request.eos = message->eos;
        request.data = message->data;
        // AudioStream 没有优先级字段：实时流默认与当前声音混音。
        request.priority = AudioClipMsg::PRIORITY_INTERACTION;
        request.policy = AudioClipMsg::POLICY_MIX;
        request.group = "stream";

        if (const auto result = playback_->enqueue_stream(request); !result.accepted) {
            LOG_ERROR("audio_io stream request rejected: {}", result.error);
        }
    }

    void AudioIoNode::handle_stop(
        const std::shared_ptr<audio_io_interface::srv::StopAudio::Request>& request,
        const std::shared_ptr<audio_io_interface::srv::StopAudio::Response>& response) const {
        response->stopped = playback_->stop_group(request->group, request->all);
        response->success = true;
        response->message = response->stopped > 0 ? "stopped" : "nothing to stop";
    }

    void AudioIoNode::handle_set_volume(
        const std::shared_ptr<audio_io_interface::srv::SetVolume::Request>& request,
        const std::shared_ptr<audio_io_interface::srv::SetVolume::Response>& response) const {
        const float volume = std::clamp(request->volume, 0.0F, 1.0F);
        playback_->set_volume(volume, request->mute);
        response->success = true;
        response->volume = volume;
        response->mute = request->mute;
        response->message = "volume updated";
    }

    rclcpp_action::GoalResponse AudioIoNode::handle_goal(const rclcpp_action::GoalUUID& /*uuid*/,
                                                         const std::shared_ptr<const PlayClipAction::Goal>& goal) {
        if (goal->clip.uri.empty()) {
            LOG_ERROR("audio_io play_clip goal rejected: empty uri");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse AudioIoNode::handle_cancel(
        const std::shared_ptr<GoalHandlePlayClip>& goal_handle) {
        std::string request_id;
        {
            std::scoped_lock lock(goals_mutex_);
            for (const auto& [id, handle] : goals_) {
                if (handle.get() == goal_handle.get()) {
                    request_id = id;
                    break;
                }
            }
        }
        if (request_id.empty()) {
            return rclcpp_action::CancelResponse::REJECT;
        }
        playback_->cancel(request_id);
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void AudioIoNode::handle_accepted(
        const std::shared_ptr<GoalHandlePlayClip>& goal_handle) {
        const std::string request_id = std::format(
            "goal-{}", next_goal_id_.fetch_add(1));
        const auto goal = goal_handle->get_goal();

        ClipRequest request;
        request.request_id = request_id;
        request.uri = goal->clip.uri;
        request.gain = goal->clip.gain;
        request.loop = goal->clip.loop;
        request.priority = goal->clip.priority;
        request.policy = goal->clip.policy;
        request.group = goal->clip.group;
        request.start_offset = goal->clip.start_offset;
        request.duration = goal->clip.duration;

        if (const auto result = playback_->enqueue_clip(request); !result.accepted) {
            const auto action_result = std::make_shared<PlayClipAction::Result>();
            action_result->request_id = request_id;
            action_result->success = false;
            action_result->message = result.error;
            goal_handle->abort(action_result);
            LOG_ERROR("audio_io play_clip rejected: {}", result.error);
            return;
        }

        {
            std::scoped_lock lock(goals_mutex_);
            goals_.emplace(request_id, goal_handle);
        }
        LOG_INFO("audio_io play_clip accepted: {}", request_id);
    }

    void AudioIoNode::poll() {
        publish_events();
        publish_state();
        poll_actions();
    }

    void AudioIoNode::publish_state() {
        const auto [playback_ok, has_active, request_id, uri, position, duration, queue_depth, underruns] = playback_->state();
        AudioStateMsg message;
        if (!playback_ok) {
            message.state = "error";
        }
        else if (capture_ && !capture_->ok()) {
            message.state = "degraded";
        }
        else if (has_active) {
            message.state = "playing";
        }
        else {
            message.state = "idle";
        }
        message.playback_device = playback_device_;
        message.capture_device = capture_ ? capture_device_ : std::string{};
        message.request_id = request_id;
        message.uri = uri;
        message.position = position;
        message.duration = duration;
        message.queue_depth = queue_depth;
        message.underruns = underruns;
        message.capture_ok = capture_ && capture_->ok();
        message.capture_error = capture_ ? capture_->error() : "capture disabled";
        state_publisher_->publish(message);
    }

    void AudioIoNode::publish_events() const {
        const auto publish = [this](const std::vector<AudioIoEvent>& events) {
            for (const auto& [request_id, type, detail] : events) {
                AudioEventMsg message;
                message.header.stamp = node_->now();
                message.header.frame_id = frame_id_;
                message.request_id = request_id;
                message.event = type;
                message.detail = detail;
                event_publisher_->publish(message);
            }
        };
        publish(playback_->take_events());
        if (capture_) {
            publish(capture_->take_events());
        }
    }

    void AudioIoNode::publish_capture() {
        if (!capture_) {
            return;
        }
        for (const auto& chunk : capture_->take_chunks()) {
            AudioStreamMsg message;
            message.header.stamp = node_->now();
            message.header.frame_id = frame_id_;
            message.stream_id = "mic";
            message.encoding = "s16le";
            message.sample_rate = capture_rate_;
            message.channels = capture_channels_;
            message.sequence = capture_sequence_.fetch_add(1);
            message.eos = false;
            message.data.resize(chunk.size() * sizeof(int16_t));
            std::memcpy(message.data.data(), chunk.data(), message.data.size());
            capture_publisher_->publish(message);
        }
    }

    void AudioIoNode::publish_capture_eos() {
        if (!capture_ || !capture_publisher_) {
            return;
        }
        AudioStreamMsg message;
        message.header.stamp = node_->now();
        message.header.frame_id = frame_id_;
        message.stream_id = "mic";
        message.encoding = "s16le";
        message.sample_rate = capture_rate_;
        message.channels = capture_channels_;
        message.sequence = capture_sequence_.fetch_add(1);
        message.eos = true;
        capture_publisher_->publish(message);
    }

    void AudioIoNode::poll_actions() {
        std::vector<std::pair<std::string, std::shared_ptr<GoalHandlePlayClip>>> goals;
        {
            std::scoped_lock lock(goals_mutex_);
            goals.reserve(goals_.size());
            for (const auto& entry : goals_) {
                goals.emplace_back(entry);
            }
        }

        for (const auto& [request_id, goal_handle] : goals) {
            const auto [found, active, finished, success, message, position, duration] = playback_->status(request_id);

            if (goal_handle->is_canceling()) {
                const auto result = std::make_shared<PlayClipAction::Result>();
                result->request_id = request_id;
                result->success = false;
                result->message = "canceled";
                goal_handle->canceled(result);
                {
                    std::scoped_lock lock(goals_mutex_);
                    goals_.erase(request_id);
                }
                playback_->acknowledge(request_id);
                continue;
            }

            if (!found) {
                const auto result = std::make_shared<PlayClipAction::Result>();
                result->request_id = request_id;
                result->success = false;
                result->message = "request not found";
                goal_handle->abort(result);
                {
                    std::scoped_lock lock(goals_mutex_);
                    goals_.erase(request_id);
                }
                continue;
            }

            if (finished) {
                const auto result = std::make_shared<PlayClipAction::Result>();
                result->request_id = request_id;
                result->success = success;
                result->message = message;
                if (success) {
                    goal_handle->succeed(result);
                }
                else {
                    goal_handle->abort(result);
                }
                {
                    std::scoped_lock lock(goals_mutex_);
                    goals_.erase(request_id);
                }
                playback_->acknowledge(request_id);
                continue;
            }

            if (active) {
                const auto feedback = std::make_shared<PlayClipAction::Feedback>();
                feedback->position = position;
                feedback->progress = duration > 0.0F
                    ? std::clamp(position / duration, 0.0F, 1.0F)
                    : 0.0F;
                goal_handle->publish_feedback(feedback);
            }
        }
    }

} // namespace audio_io
