#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "audio_io/capture_engine.hpp"
#include "audio_io/playback_engine.hpp"
#include "audio_io_interface/action/play_clip.hpp"
#include "audio_io_interface/msg/audio_clip.hpp"
#include "audio_io_interface/msg/audio_event.hpp"
#include "audio_io_interface/msg/audio_state.hpp"
#include "audio_io_interface/msg/audio_stream.hpp"
#include "audio_io_interface/srv/set_volume.hpp"
#include "audio_io_interface/srv/stop_audio.hpp"

namespace audio_io {

    /// audio_io 的组合式节点。
    ///
    /// 本类不继承 rclcpp::Node，而是持有一个外部传入的节点。其他功能包可以这样复用：
    ///
    /// @code
    /// auto node = std::make_shared<rclcpp::Node>("system");
    /// auto audio = std::make_shared<audio_io::AudioIoNode>(node);
    /// audio->start();
    /// @endcode
    ///
    /// 所有参数都声明在传入的节点上；本类只负责音频设备、播放引擎和 ROS 接口。
    class AudioIoNode {
    public:
        using PlayClipAction = audio_io_interface::action::PlayClip;
        using GoalHandlePlayClip = rclcpp_action::ServerGoalHandle<PlayClipAction>;

        explicit AudioIoNode(rclcpp::Node::SharedPtr node);
        ~AudioIoNode();

        AudioIoNode(const AudioIoNode&) = delete;
        AudioIoNode& operator=(const AudioIoNode&) = delete;

        [[nodiscard]] const rclcpp::Node::SharedPtr& node() const noexcept {
            return node_;
        }

        /// 启动播放/采集线程和状态定时器；重复调用是安全的。
        void start();

        /// 停止播放/采集线程并发布采集流的 eos；重复调用是安全的。
        void stop();

    private:
        void declare_parameters();
        void create_interfaces();

        void handle_play(const audio_io_interface::msg::AudioClip::SharedPtr message);
        void handle_stream(const audio_io_interface::msg::AudioStream::SharedPtr message);
        void handle_stop(const std::shared_ptr<audio_io_interface::srv::StopAudio::Request> request,
                         std::shared_ptr<audio_io_interface::srv::StopAudio::Response> response);
        void handle_set_volume(
            const std::shared_ptr<audio_io_interface::srv::SetVolume::Request> request,
            std::shared_ptr<audio_io_interface::srv::SetVolume::Response> response);

        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const PlayClipAction::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandlePlayClip>& goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandlePlayClip>& goal_handle);

        void poll();
        void publish_state();
        void publish_events();
        void publish_capture();
        void poll_actions();

        void publish_capture_eos();

        rclcpp::Node::SharedPtr node_;
        std::string backend_;
        std::string playback_device_;
        std::string capture_device_;
        uint32_t playback_rate_{48000};
        uint16_t playback_channels_{1};
        uint32_t playback_period_ms_{10};
        uint32_t playback_buffer_ms_{40};
        uint32_t capture_rate_{48000};
        uint16_t capture_channels_{1};
        uint32_t capture_period_ms_{10};
        uint32_t capture_buffer_ms_{40};
        uint32_t stream_timeout_ms_{200};
        std::string frame_id_{"audio"};
        bool capture_enabled_{false};

        std::unique_ptr<PlaybackEngine> playback_;
        std::unique_ptr<CaptureEngine> capture_;

        rclcpp::Publisher<audio_io_interface::msg::AudioState>::SharedPtr state_publisher_;
        rclcpp::Publisher<audio_io_interface::msg::AudioEvent>::SharedPtr event_publisher_;
        rclcpp::Publisher<audio_io_interface::msg::AudioStream>::SharedPtr capture_publisher_;
        rclcpp::Subscription<audio_io_interface::msg::AudioClip>::SharedPtr play_subscription_;
        rclcpp::Subscription<audio_io_interface::msg::AudioStream>::SharedPtr stream_subscription_;
        rclcpp::Service<audio_io_interface::srv::StopAudio>::SharedPtr stop_service_;
        rclcpp::Service<audio_io_interface::srv::SetVolume>::SharedPtr volume_service_;
        rclcpp_action::Server<PlayClipAction>::SharedPtr action_server_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr capture_timer_;

        std::atomic<uint64_t> capture_sequence_{0};
        std::atomic<uint64_t> next_goal_id_{1};

        std::mutex goals_mutex_;
        std::unordered_map<std::string, std::shared_ptr<GoalHandlePlayClip>> goals_;

        bool started_{false};
    };

} // namespace audio_io
