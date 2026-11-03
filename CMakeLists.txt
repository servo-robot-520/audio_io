cmake_minimum_required(VERSION 3.22)
project(audio_io VERSION 0.1.0)

option(BUILD_EXE "Build node executable file" ON)
option(BUILD_TESTING "Build unit tests" ON)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (NOT SKIP_CXX23_STDLIB_CHECK AND NOT _CXX23_STDLIB_VERIFIED)
    include(CheckCXXSourceCompiles)
    set(CMAKE_REQUIRED_QUIET TRUE)
    string(APPEND CMAKE_REQUIRED_FLAGS " -std=c++23")

    string(TOUPPER "${PROJECT_NAME}" _PRJ_UPPER)

    check_cxx_source_compiles([[
        #include <expected>
        #include <format>
        #include <ranges>
        int main() {
            std::expected<int, int> result{42};
            const auto text = std::format("{}", result.value());
            const auto values = std::views::iota(0, 1);
            return static_cast<int>(text.size() + *values.begin());
        }
    ]] ${_PRJ_UPPER}_HAS_REQUIRED_CXX23_STDLIB)

    if (NOT ${_PRJ_UPPER}_HAS_REQUIRED_CXX23_STDLIB)
        message(FATAL_ERROR
                "${PROJECT_NAME} requires a C++23 standard library with <expected>, "
                "<format>, and ranges support. Use GCC 13+ or an equivalent compiler.")
    endif ()
endif ()

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ALSA REQUIRED alsa)

# 普通 colcon/独立 CMake 构建时，依赖包已经安装；CLion 源码聚合构建则直接
# 使用本次构建树中的 target。
if (NOT ROS2_WS_SOURCE_BUILD)
    find_package(common_cpp REQUIRED)
    find_package(audio_io_interface REQUIRED)
endif ()

# 可复用的音频引擎库。其他功能包可以链接 audio_io，并传入自己的 rclcpp::Node。
add_library(audio_io STATIC
        include/audio_io/alsa_sink.hpp
        include/audio_io/audio_format.hpp
        include/audio_io/audio_io_node.hpp
        include/audio_io/capture_engine.hpp
        include/audio_io/playback_engine.hpp
        include/audio_io/wav_decoder.hpp
        src/alsa_sink.cpp
        src/audio_io_node.cpp
        src/capture_engine.cpp
        src/playback_engine.cpp
        src/wav_decoder.cpp
)
target_compile_features(audio_io PUBLIC cxx_std_23)
set_target_properties(audio_io PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(audio_io PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_include_directories(audio_io PRIVATE ${ALSA_INCLUDE_DIRS})
target_link_libraries(audio_io PUBLIC ${ALSA_LIBRARIES})
ament_target_dependencies(audio_io PUBLIC
        rclcpp
        rclcpp_action
        std_msgs
        ament_index_cpp
)

# common_cpp 只提供日志宏/格式化接口；ROS 日志桥接由可执行文件安装。
if (ROS2_WS_SOURCE_BUILD)
    if (NOT TARGET common_cpp_logging_interface)
        message(FATAL_ERROR
                "CLion workspace configuration requires the in-tree common_cpp_logging_interface target.")
    endif ()
    target_link_libraries(audio_io PUBLIC common_cpp_logging_interface)
else ()
    target_link_libraries(audio_io PUBLIC common_cpp::common_cpp_logging_interface)
endif ()

# audio_io_interface 提供消息类型与 endpoints；源码聚合构建使用其 CMake helper。
if (ROS2_WS_SOURCE_BUILD)
    if (NOT TARGET audio_io_interface)
        message(FATAL_ERROR
                "CLion workspace configuration requires the in-tree audio_io_interface target.")
    endif ()
    include("${audio_io_interface_SOURCE_DIR}/cmake_include.cmake")
else ()
    ament_target_dependencies(audio_io PUBLIC audio_io_interface)
endif ()

if (BUILD_EXE)
    add_executable(audio_io_node src/main.cpp)
    target_compile_features(audio_io_node PRIVATE cxx_std_23)
    target_link_libraries(audio_io_node audio_io)
    ament_target_dependencies(audio_io_node rclcpp)

    # rcutils 输出处理器是进程全局对象，只能由 main.cpp 安装。
    if (ROS2_WS_SOURCE_BUILD)
        if (NOT TARGET common_cpp_ros_logging)
            message(FATAL_ERROR
                    "CLion workspace configuration requires the in-tree common_cpp_ros_logging target.")
        endif ()
        target_link_libraries(audio_io_node common_cpp_ros_logging)
    else ()
        target_link_libraries(audio_io_node common_cpp::common_cpp_ros_logging)
    endif ()
endif ()

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_wav_decoder test/test_wav_decoder.cpp)
    target_link_libraries(test_wav_decoder audio_io)
endif ()

install(TARGETS audio_io
        EXPORT audio_ioTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

if (BUILD_EXE)
    install(TARGETS audio_io_node
            DESTINATION lib/${PROJECT_NAME}
    )
    install(DIRECTORY config launch
            DESTINATION share/${PROJECT_NAME}
            PATTERN "__pycache__" EXCLUDE
            PATTERN "*.pyc" EXCLUDE
    )
endif ()

install(DIRECTORY include/
        DESTINATION include
)

install(FILES README.md
        DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_action std_msgs ament_index_cpp audio_io_interface common_cpp)
ament_export_targets(audio_ioTargets HAS_LIBRARY_TARGET)
ament_package()
