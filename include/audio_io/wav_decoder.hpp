#pragma once

#include <cstdint>
#include <expected>
#include <string>
#include <vector>

#include "audio_io/audio_format.hpp"

namespace audio_io {

    /// 最小 WAV 解码器。
    ///
    /// 只负责把 WAV 的 RIFF 容器拆成 PCM 样本，不依赖 libsndfile/GStreamer。
    /// 当前支持：
    /// - PCM 8/16/24/32-bit；
    /// - IEEE float 32/64-bit；
    /// - WAVE_FORMAT_EXTENSIBLE 中的 PCM/float 子格式；
    /// - 单声道/多声道（多声道在 to_mono_48k 中下混）。
    ///
    /// MP3/FLAC/OGG 等压缩格式不在本解码器范围内；遇到非 WAV 容器会返回错误。
    class WavDecoder {
    public:
        [[nodiscard]] static std::expected<AudioBuffer, std::string> decode_file(
            const std::string& path);

        [[nodiscard]] static std::expected<AudioBuffer, std::string> decode_bytes(
            const std::vector<uint8_t>& bytes);
    };

} // namespace audio_io
