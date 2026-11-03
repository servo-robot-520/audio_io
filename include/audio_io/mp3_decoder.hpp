#pragma once

#include <expected>
#include <string>

#include "audio_io/audio_format.hpp"

namespace audio_io {

    /// MP3 解码器，基于 libmpg123。
    ///
    /// 输出保持 MP3 文件的原生采样率/声道数，由 to_mono_48k 统一下混和重采样。
    class Mp3Decoder {
    public:
        [[nodiscard]] static std::expected<AudioBuffer, std::string> decode_file(
            const std::string& path);
    };

} // namespace audio_io
