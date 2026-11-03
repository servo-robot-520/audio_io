# audio_io

## 简介

`audio_io` 是一个通用的 ROS 2 音频 I/O 功能包：让**一个进程独占音频设备**，其他节点只发送“播放什么、采集什么”的请求，不再各自打开 ALSA/PulseAudio/PipeWire。

它不依赖 microduck 的鸭子语义，可以用于任何需要播放音频文件、播放实时 PCM、采集麦克风的 ROS 2 系统。典型用途：

- 机器人提示音、语音播报、TTS 输出；
- 实时合成音频流（乐器、歌声、传感器驱动的音效）；
- 麦克风采集与下游语音/音频分析；
- 多路声音的混音、优先级、抢占与音频焦点管理。

本包提供两个 target：

| target | 类型 | 作用 |
| --- | --- | --- |
| `audio_io` | 静态库 | 播放/采集引擎、WAV/MP3 解码、混音、优先级、ALSA 抽象 |
| `audio_io_node` | 可执行文件 | 组合 `audio_io::AudioIoNode`，对外提供 ROS 2 话题、服务与 action |

### 核心设计

- **单一设备所有者**：播放和采集都由 `audio_io` 打开；其他节点通过 `AudioClip` / `AudioStream` 提交数据，避免多个进程争抢同一张声卡。
- **播放请求有优先级**：`QUEUE`、`PREEMPT`、`MIX`、`REPLACE_SAME_GROUP` 四种策略，配 `group` 实现“语音可被警报抢占、提示音不打断 TTS”等行为。
- **文件与实时流统一**：静态文件解码成 PCM 后进入同一个混音器；`AudioStream` 的实时 PCM 也走同一条路径。
- **格式统一**：内部使用 48 kHz 单声道 float 混音；输入文件/流的采样率、声道数不同时自动下混和线性重采样。
- **组合式节点**：`AudioIoNode` 不继承 `rclcpp::Node`，构造函数接收外部 `Node::SharedPtr`，其他功能包可以把本包嵌入自己的节点并共享执行器。
- **无硬件可运行**：`backend=null` 使用 `NullSink`，便于在没有声卡的 CI、容器和开发机上验证节点与接口。

### 包含与不包含

包含：

- WAV：PCM 8/16/24/32-bit、float 32/64-bit、WAVE_FORMAT_EXTENSIBLE；
- MP3：基于 `libmpg123`；
- `AudioClip` 文件播放：增益、循环、起始偏移、时长限制；
- `AudioStream` 实时 PCM：`s16le` / `f32le`，单/多声道，自动重采样到 48 kHz 单声道；
- ALSA 播放/采集、欠载恢复、设备丢失/恢复事件；
- `/audio/io/capture` 麦克风 PCM 发布；
- 播放状态、事件、停止、音量服务与 `PlayClip` action。

不包含：

- TTS 引擎、鸭子叫声语义、摸头检测模型（分别在 TTS / `duck_audio` / `duck_ear` 包中）；
- WebRTC 传输；
- FLAC/OGG 解码（当前只内置 WAV 和 MP3）；
- 内置 PipeWire 客户端后端（当前使用 ALSA；PipeWire 环境可通过 ALSA 的 pulse/pipewire 插件使用）。

### 内部结构

```text
audio_io_node
  ├── PlaybackEngine（独立音频线程）
  │     ├── AudioDecoder: WAV / MP3 → 48 kHz mono float
  │     ├── 混音器 / 优先级 / 抢占
  │     └── AudioSink: AlsaSink / NullSink
  ├── CaptureEngine（独立采集线程）
  │     └── ALSA capture → 有界队列 → ~/capture
  └── ROS 接口
        ├── ~/play, ~/stream, ~/capture
        ├── ~/state, ~/event
        ├── ~/stop, ~/set_volume
        └── ~/play_clip action
```

`AudioIoNode` 采用组合而不是继承；库使用者可以这样嵌入：

```cpp
auto node = std::make_shared<rclcpp::Node>("system");
auto audio = std::make_shared<audio_io::AudioIoNode>(node);
audio->start();
```

## 构建

### colcon

```bash
cd microduck_rev
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select audio_io_interface audio_io
```

`audio_io` 依赖 `common_cpp`、`audio_io_interface`、ALSA 和 `libmpg123`，需要先构建/安装它们。

### 普通 CMake / CLion

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
cmake -S src/audio_io -B build-audio_io \
  -DCMAKE_CXX_COMPILER=g++-13 \
  -DCMAKE_PREFIX_PATH="$PWD/install/audio_io_interface;$PWD/install/common_cpp;/opt/ros/humble"
cmake --build build-audio_io -j
```

顶层工作区 CMake 已把 `audio_io` 加入源码聚合构建；在 CLion 中打开工作区根目录即可。

## 运行

```bash
ros2 launch audio_io audio_io.launch.py
```

无音频硬件时用空设备验证节点与接口：

```bash
ros2 run audio_io audio_io_node --ros-args \
  -p backend:=null \
  -p capture.enabled:=false
```

节点默认全名是 `/audio/io`。

### 快速示例

播放一个 WAV/MP3 文件：

```bash
ros2 topic pub --once /audio/io/play audio_io_interface/msg/AudioClip \
  "{uri: 'file:///tmp/robot_start_3s.mp3', gain: 0.8, priority: 3, policy: 0, group: 'voice'}"
```

查看状态与事件：

```bash
ros2 topic echo /audio/io/state
ros2 topic echo /audio/io/event
```

采集麦克风：

```bash
ros2 topic hz /audio/io/capture
ros2 topic echo /audio/io/capture --once
```

## 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `~/play` | `audio_io_interface/msg/AudioClip` | 播放文件片段 |
| `~/stream` | `audio_io_interface/msg/AudioStream` | 播放实时 PCM 流 |
| `~/capture` | `audio_io_interface/msg/AudioStream` | 麦克风采集 PCM，`stream_id=mic` |
| `~/state` | `audio_io_interface/msg/AudioState` | 播放/采集状态，`transient_local` |
| `~/event` | `audio_io_interface/msg/AudioEvent` | 开始/结束/抢占/欠载/设备事件 |

## 服务与动作

| 接口 | 类型 | 说明 |
| --- | --- | --- |
| `~/stop` | `audio_io_interface/srv/StopAudio` | 按组或全部停止 |
| `~/set_volume` | `audio_io_interface/srv/SetVolume` | 主音量与静音 |
| `~/play_clip` | `audio_io_interface/action/PlayClip` | 播放并反馈进度/完成 |

## 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `backend` | `alsa` | `alsa` 或 `null` |
| `playback.device` | `plughw:aic3104` | ALSA 播放设备 |
| `playback.rate` | `48000` | 内部采样率 |
| `playback.channels` | `1` | 播放声道数 |
| `playback.period_ms` | `10` | ALSA period |
| `playback.buffer_ms` | `40` | ALSA buffer |
| `capture.enabled` | `true` | 是否启用麦克风 |
| `capture.device` | 空 | 空则复用 playback.device |
| `capture.rate` | `48000` | 采集采样率 |
| `capture.channels` | `1` | 采集声道数 |
| `capture.period_ms` | `10` | 采集 period |
| `capture.buffer_ms` | `40` | 采集 buffer |
| `stream.timeout_ms` | `200` | 实时流无数据后的超时 |
| `volume` | `1.0` | 主音量 |
| `mute` | `false` | 静音 |
| `frame_id` | `audio` | 消息 frame_id |

## ALSA 说明

Linux 音频栈的最底层是 ALSA：

- `plughw:` 设备会自动做格式/采样率/声道转换，适合机器人 codec；
- `period_ms` 是 ALSA 一次唤醒处理的块大小；越大越省 CPU、延迟越高；
- `buffer_ms` 是环形缓冲总长度；太小容易 underrun（`EPIPE`）；
- 播放和采集是同一个 PCM device 的两个方向，但同一时刻只应由一个进程持有；
- 如果 PulseAudio/PipeWire 占用了设备，直接打开 ALSA 会得到 `EBUSY`。

## 支持范围

- WAV：PCM 8/16/24/32-bit、float 32/64-bit、WAVE_FORMAT_EXTENSIBLE；
- MP3：基于 `libmpg123`，输出原生采样率/声道数后统一下混到 48 kHz 单声道；
- 实时流：`s16le` / `f32le`，单/多声道，内部下混并重采样到 48 kHz 单声道；
- FLAC/OGG 暂未内置解码器；需要时可在 `AudioDecoder` 中增加解码后端。

## 相关包

- `audio_io_interface`：`AudioClip`、`AudioStream`、`AudioState`、`AudioEvent`、`StopAudio`、`SetVolume`、`PlayClip` 的消息/服务/action 定义；
- `common_cpp`：日志与通用工具；
- ALSA：`libasound2-dev`；
- MP3：`libmpg123-dev`。

## 背景

本包从 microduck 的声音系统抽象而来，但接口和实现不依赖 microduck 的机器人语义。任何需要“单进程独占声卡 + 统一播放/采集”的 ROS 2 系统都可以直接使用；鸭子叫声、TTS、特雷门、合唱等上层行为应在各自的业务包中实现。
