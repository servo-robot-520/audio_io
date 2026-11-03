#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "audio_io/audio_io_node.hpp"
#include "common_cpp/logging/rclcpp_log_handler.hpp"

int main(const int argc, char** argv) {
    rclcpp::init(argc, argv);

    const auto node = std::make_shared<rclcpp::Node>("audio_io");
    // rcutils 输出处理器是进程级对象，只能在可执行文件入口安装一次。
    if (!logging::ros::RclcppLogHandler::try_init_from_node(node)) {
        return 1;
    }

    try {
        const auto audio = std::make_shared<audio_io::AudioIoNode>(node);
        audio->start();

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
        executor.remove_node(node);

        audio->stop();
    }
    catch (const std::exception& exception) {
        RCLCPP_FATAL(node->get_logger(), "audio_io terminated: %s", exception.what());
        logging::ros::RclcppLogHandler::shutdown();
        rclcpp::shutdown();
        return 1;
    }
    catch (...) {
        RCLCPP_FATAL(node->get_logger(), "audio_io terminated with an unknown error");
        logging::ros::RclcppLogHandler::shutdown();
        rclcpp::shutdown();
        return 1;
    }

    logging::ros::RclcppLogHandler::shutdown();
    rclcpp::shutdown();
    return 0;
}
