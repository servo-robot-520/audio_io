#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace audio_io {

    /// 内部统一使用的输出采样率。ALSA 设备、混音器和 48 kHz 实时流都使用该值。
    inline constexpr uint32_t kOutputSampleRate = 48000;

    /// 内部统一使用的声道数；麦克风与扬声器都按单声道处理。
    inline constexpr uint16_t kOutputChannels = 1;

    /// 解码后的 PCM 缓冲。samples 按帧交错排列，取值范围约为 [-1.0, 1.0]。
    struct AudioBuffer {
        uint32_t sample_rate{0};
        uint16_t channels{0};
        std::vector<float> samples;

        [[nodiscard]] std::size_t frames() const noexcept {
            if (channels == 0) {
                return 0;
            }
            return samples.size() / channels;
        }
    };

    /// 将任意采样率/声道数的 PCM 转成 48 kHz 单声道，供混音器统一处理。
    /// 多声道按算术平均下混；采样率不同时使用线性插值重采样。
    [[nodiscard]] std::vector<float> to_mono_48k(const AudioBuffer& buffer);

} // namespace audio_io
