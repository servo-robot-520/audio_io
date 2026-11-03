# audio_io

## 简介

`audio_io` 是文档 [`docs/microduck-audio-architecture.zh-CN.md`](../../docs/microduck-audio-architecture.zh-CN.md)
中 `/audio/io` 的 ROS 2 实现：

- `audio_io` 静态库：ALSA 播放/采集、WAV 解码、混音、优先级与抢占、实时 PCM 流；
- `audio_io_node` 可执行文件：组合 `audio_io::AudioIoNode`，对外提供话题、服务与 action。

`AudioIoNode` 采用组合而不是继承：构造函数接收外部 `rclcpp::Node::SharedPtr`，其他功能包可以把它嵌入自己的节点并共享执行器。

## 构建

### colcon

```bash
cd microduck_rev
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select audio_io_interface audio_io
```

`audio_io` 依赖 `common_cpp` 和 `audio_io_interface`，需要先构建/安装它们。

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
- 实时流：`s16le` / `f32le`，单/多声道，内部下混并重采样到 48 kHz 单声道；
- MP3/FLAC/OGG 暂未内置解码器；需要时可在 `WavDecoder` 前增加解码后端。
