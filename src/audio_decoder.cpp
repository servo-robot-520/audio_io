#include "audio_io/audio_decoder.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>

#include "audio_io/mp3_decoder.hpp"
#include "audio_io/wav_decoder.hpp"

namespace audio_io {
    namespace {

        [[nodiscard]] std::string lower_extension(const std::string& path) {
            std::string extension = std::filesystem::path(path).extension().string();
            std::transform(extension.begin(), extension.end(), extension.begin(),
                           [](const unsigned char ch) {
                               return static_cast<char>(std::tolower(ch));
                           });
            return extension;
        }

    } // namespace

    std::expected<AudioBuffer, std::string> AudioDecoder::decode_file(
        const std::string& path) {
        const std::string extension = lower_extension(path);
        if (extension == ".wav" || extension == ".wave") {
            return WavDecoder::decode_file(path);
        }
        if (extension == ".mp3") {
            return Mp3Decoder::decode_file(path);
        }

        // 没有可识别后缀：先按 WAV 尝试，再按 MP3 尝试。
        auto wav = WavDecoder::decode_file(path);
        if (wav) {
            return wav;
        }
        return Mp3Decoder::decode_file(path);
    }

} // namespace audio_io
