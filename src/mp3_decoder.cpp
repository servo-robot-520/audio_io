#include "audio_io/mp3_decoder.hpp"

#include <mpg123.h>

#include <algorithm>
#include <memory>
#include <mutex>
#include <vector>

namespace audio_io {
    namespace {

        using Mpg123Handle = std::unique_ptr<mpg123_handle, decltype(&mpg123_delete)>;

        /// libmpg123 的全局初始化只需要执行一次。
        std::once_flag g_init_once;

        [[nodiscard]] std::string mpg123_error(mpg123_handle* handle, const int code) {
            const char* text = handle != nullptr
                ? mpg123_strerror(handle)
                : mpg123_plain_strerror(code);
            return text != nullptr ? std::string{text} : std::string{"unknown mpg123 error"};
        }

        /// 把 MP3 解码结果配置为 16-bit signed 输出。
        /// 保持原生采样率/声道数，后续由 to_mono_48k 统一处理。
        [[nodiscard]] bool configure_signed16(mpg123_handle* handle, long& rate,
                                              int& channels) {
            int encoding = 0;
            if (mpg123_getformat(handle, &rate, &channels, &encoding) != MPG123_OK) {
                return false;
            }
            if (mpg123_format_none(handle) != MPG123_OK) {
                return false;
            }
            return mpg123_format(handle, rate, channels, MPG123_ENC_SIGNED_16) ==
                MPG123_OK;
        }

    } // namespace

    std::expected<AudioBuffer, std::string> Mp3Decoder::decode_file(
        const std::string& path) {
        std::call_once(g_init_once, [] {
            if (mpg123_init() != MPG123_OK) {
                // 具体错误会在 mpg123_new 失败时返回；这里保持一次性初始化语义。
            }
        });

        int error = MPG123_OK;
        const Mpg123Handle handle(mpg123_new(nullptr, &error), &mpg123_delete);
        if (!handle) {
            return std::unexpected("mpg123_new failed: " + mpg123_error(nullptr, error));
        }

        if (mpg123_open(handle.get(), path.c_str()) != MPG123_OK) {
            return std::unexpected("mpg123_open failed for " + path + ": " +
                mpg123_error(handle.get(), MPG123_ERR));
        }

        long rate = 0;
        int channels = 0;
        if (!configure_signed16(handle.get(), rate, channels)) {
            return std::unexpected("mpg123 cannot configure S16 output for " + path +
                ": " + mpg123_error(handle.get(), MPG123_ERR));
        }
        if (rate <= 0 || channels <= 0) {
            return std::unexpected("mpg123 returned invalid format for " + path);
        }

        AudioBuffer buffer;
        buffer.sample_rate = static_cast<uint32_t>(rate);
        buffer.channels = static_cast<uint16_t>(channels);

        const size_t block = std::max<size_t>(mpg123_outblock(handle.get()), 16384U);
        std::vector<unsigned char> pcm(block);

        while (true) {
            size_t done = 0;
            if (const int result = mpg123_read(handle.get(), pcm.data(), pcm.size(), &done); result == MPG123_OK) {
                // S16_LE：两个字节一个样本，按小端还原为有符号 16-bit。
                for (size_t offset = 0; offset + 1 < done; offset += 2) {
                    const auto raw = static_cast<uint16_t>(pcm[offset]) |
                        static_cast<uint16_t>(
                            static_cast<uint16_t>(pcm[offset + 1]) << 8U);
                    buffer.samples.push_back(
                        static_cast<float>(static_cast<int16_t>(raw)) / 32768.0F);
                }
                if (done == 0) {
                    break;
                }
            }
            else if (result == MPG123_NEW_FORMAT) {
                // 某些 MP3 中途切换采样率/声道数；重新配置并继续。
                if (!configure_signed16(handle.get(), rate, channels)) {
                    return std::unexpected("mpg123 format change failed for " + path);
                }
            }
            else if (result == MPG123_DONE or result == MPG123_NEED_MORE) {
                break;
            }
            else {
                return std::unexpected("mpg123_read failed for " + path + ": " +
                    mpg123_error(handle.get(), result));
            }
        }

        if (buffer.samples.empty()) {
            return std::unexpected("MP3 file decoded to no samples: " + path);
        }
        return buffer;
    }

} // namespace audio_io
