#pragma once

#include <expected>
#include <string>

#include "audio_io/audio_format.hpp"

namespace audio_io {

    /// 按文件类型选择 WAV/MP3 解码器。
    ///
    /// 先看扩展名；扩展名未知时依次尝试 WAV 和 MP3，便于 URI 没有后缀的场景。
    class AudioDecoder {
    public:
        [[nodiscard]] static std::expected<AudioBuffer, std::string> decode_file(
            const std::string& path);
    };

} // namespace audio_io
