#include "audio_io/wav_decoder.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace audio_io {
    namespace {

        constexpr uint16_t kWaveFormatPcm = 1;
        constexpr uint16_t kWaveFormatFloat = 3;
        constexpr uint16_t kWaveFormatExtensible = 0xFFFE;

        [[nodiscard]] uint16_t read_u16(const uint8_t* data) noexcept {
            return static_cast<uint16_t>(data[0]) |
                   static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
        }

        [[nodiscard]] uint32_t read_u32(const uint8_t* data) noexcept {
            return static_cast<uint32_t>(data[0]) |
                   (static_cast<uint32_t>(data[1]) << 8U) |
                   (static_cast<uint32_t>(data[2]) << 16U) |
                   (static_cast<uint32_t>(data[3]) << 24U);
        }

        [[nodiscard]] uint64_t read_u64(const uint8_t* data) noexcept {
            return static_cast<uint64_t>(read_u32(data)) |
                   (static_cast<uint64_t>(read_u32(data + 4)) << 32U);
        }

        [[nodiscard]] bool chunk_id_is(const uint8_t* data, const char* id) noexcept {
            return std::memcmp(data, id, 4) == 0;
        }

        /// 把 24-bit 有符号 PCM 符号扩展成 32-bit。
        [[nodiscard]] int32_t read_s24(const uint8_t* data) noexcept {
            const uint32_t raw = static_cast<uint32_t>(data[0]) |
                                 (static_cast<uint32_t>(data[1]) << 8U) |
                                 (static_cast<uint32_t>(data[2]) << 16U);
            if ((raw & 0x00800000U) != 0U) {
                return static_cast<int32_t>(raw | 0xFF000000U);
            }
            return static_cast<int32_t>(raw);
        }

        /// 读取一个采样并归一化到 [-1.0, 1.0]。
        [[nodiscard]] float read_sample(const uint8_t* data, const uint16_t bits,
                                        const uint16_t format) {
            if (format == kWaveFormatFloat) {
                if (bits == 32U) {
                    return std::bit_cast<float>(read_u32(data));
                }
                if (bits == 64U) {
                    return static_cast<float>(std::bit_cast<double>(read_u64(data)));
                }
                throw std::runtime_error("unsupported float WAV bit depth");
            }

            switch (bits) {
            case 8:
                // 8-bit WAV 是无符号数，128 为静音中点。
                return (static_cast<int32_t>(data[0]) - 128) / 128.0F;
            case 16:
                return static_cast<float>(static_cast<int16_t>(read_u16(data))) / 32768.0F;
            case 24:
                return static_cast<float>(read_s24(data)) / 8388608.0F;
            case 32:
                return static_cast<float>(static_cast<int32_t>(read_u32(data))) /
                       2147483648.0F;
            default:
                throw std::runtime_error("unsupported PCM WAV bit depth");
            }
        }

    } // namespace

    std::vector<float> to_mono_48k(const AudioBuffer& buffer) {
        if (buffer.channels == 0U || buffer.sample_rate == 0U || buffer.samples.empty()) {
            return {};
        }

        // 下混：每个声道的样本取平均，得到原始采样率的单声道序列。
        const std::size_t frames = buffer.frames();
        std::vector<float> mono(frames, 0.0F);
        for (std::size_t frame = 0; frame < frames; ++frame) {
            float sum = 0.0F;
            for (std::size_t channel = 0; channel < buffer.channels; ++channel) {
                sum += buffer.samples[frame * buffer.channels + channel];
            }
            mono[frame] = sum / static_cast<float>(buffer.channels);
        }

        if (buffer.sample_rate == kOutputSampleRate) {
            return mono;
        }

        // 线性插值重采样。视频/语音片段通常很短，线性插值足够；
        // 若以后需要高质量重采样，可替换为 libsamplerate/speex 实现。
        const double ratio = static_cast<double>(buffer.sample_rate) /
                             static_cast<double>(kOutputSampleRate);
        const auto output_frames = static_cast<std::size_t>(
            std::llround(static_cast<double>(mono.size()) / ratio));
        std::vector<float> resampled(output_frames, 0.0F);
        for (std::size_t i = 0; i < output_frames; ++i) {
            const double source_position = static_cast<double>(i) * ratio;
            const auto index = static_cast<std::size_t>(source_position);
            const auto next = std::min(index + 1, mono.size() - 1);
            const float fraction = static_cast<float>(source_position -
                                                      static_cast<double>(index));
            resampled[i] = mono[index] + (mono[next] - mono[index]) * fraction;
        }
        return resampled;
    }

    std::expected<AudioBuffer, std::string> WavDecoder::decode_file(const std::string& path) {
        std::ifstream input(path, std::ios::binary | std::ios::ate);
        if (!input) {
            return std::unexpected("cannot open WAV file: " + path);
        }

        const std::streamsize size = input.tellg();
        if (size <= 0) {
            return std::unexpected("empty WAV file: " + path);
        }
        input.seekg(0, std::ios::beg);

        std::vector<uint8_t> bytes(static_cast<std::size_t>(size));
        if (!input.read(reinterpret_cast<char*>(bytes.data()), size)) {
            return std::unexpected("cannot read WAV file: " + path);
        }
        return decode_bytes(bytes);
    }

    std::expected<AudioBuffer, std::string> WavDecoder::decode_bytes(
        const std::vector<uint8_t>& bytes) {
        // RIFF/WAVE 结构：
        //   "RIFF" | uint32 chunk_size | "WAVE"
        //   然后是一串 chunk："id" | uint32 size | payload（奇数长度需要 1 字节填充）
        //   fmt  chunk 描述格式，data chunk 才是 PCM 样本。
        if (bytes.size() < 12U || !chunk_id_is(bytes.data(), "RIFF") ||
            !chunk_id_is(bytes.data() + 8U, "WAVE")) {
            return std::unexpected("not a RIFF/WAVE file");
        }

        uint16_t format = 0;
        uint16_t channels = 0;
        uint32_t sample_rate = 0;
        uint16_t bits_per_sample = 0;
        uint16_t block_align = 0;
        bool have_format = false;
        bool have_data = false;
        std::size_t data_offset = 0;
        std::size_t data_size = 0;

        std::size_t offset = 12;
        while (offset + 8U <= bytes.size()) {
            const uint8_t* header = bytes.data() + offset;
            const uint32_t chunk_size = read_u32(header + 4U);
            const std::size_t payload = offset + 8U;
            if (chunk_size > bytes.size() - payload) {
                return std::unexpected("truncated WAV chunk");
            }

            if (chunk_id_is(header, "fmt ")) {
                if (chunk_size < 16U) {
                    return std::unexpected("WAV fmt chunk is too short");
                }
                format = read_u16(bytes.data() + payload);
                channels = read_u16(bytes.data() + payload + 2U);
                sample_rate = read_u32(bytes.data() + payload + 4U);
                block_align = read_u16(bytes.data() + payload + 12U);
                bits_per_sample = read_u16(bytes.data() + payload + 14U);

                // WAVE_FORMAT_EXTENSIBLE 的真实格式在 SubFormat GUID 的前两个字节。
                if (format == kWaveFormatExtensible) {
                    if (chunk_size < 40U) {
                        return std::unexpected("WAVE_FORMAT_EXTENSIBLE fmt chunk is too short");
                    }
                    format = read_u16(bytes.data() + payload + 24U);
                }
                have_format = true;
            } else if (chunk_id_is(header, "data")) {
                data_offset = payload;
                data_size = chunk_size;
                have_data = true;
            }

            offset = payload + chunk_size + (chunk_size & 1U);
        }

        if (!have_format || !have_data) {
            return std::unexpected("WAV file is missing fmt or data chunk");
        }
        if (format != kWaveFormatPcm && format != kWaveFormatFloat) {
            return std::unexpected("unsupported WAV encoding");
        }
        if (channels == 0U || sample_rate == 0U) {
            return std::unexpected("invalid WAV channel count or sample rate");
        }
        if (bits_per_sample == 0U || bits_per_sample % 8U != 0U) {
            return std::unexpected("invalid WAV bit depth");
        }

        const auto bytes_per_sample = static_cast<uint16_t>(bits_per_sample / 8U);
        if (block_align == 0U) {
            block_align = static_cast<uint16_t>(channels * bytes_per_sample);
        }
        if (block_align < static_cast<uint16_t>(channels * bytes_per_sample)) {
            return std::unexpected("invalid WAV block alignment");
        }

        const std::size_t frame_count = data_size / block_align;
        AudioBuffer buffer;
        buffer.sample_rate = sample_rate;
        buffer.channels = 1; // 解码阶段直接下混成单声道，减少后续处理分支。
        buffer.samples.reserve(frame_count);

        try {
            for (std::size_t frame = 0; frame < frame_count; ++frame) {
                float sum = 0.0F;
                const std::size_t frame_offset =
                    data_offset + frame * static_cast<std::size_t>(block_align);
                for (uint16_t channel = 0; channel < channels; ++channel) {
                    const std::size_t sample_offset =
                        frame_offset + static_cast<std::size_t>(channel) * bytes_per_sample;
                    sum += read_sample(bytes.data() + sample_offset, bits_per_sample, format);
                }
                buffer.samples.push_back(sum / static_cast<float>(channels));
            }
        }
        catch (const std::exception& exception) {
            return std::unexpected(std::string{"cannot decode WAV samples: "} +
                                   exception.what());
        }

        return buffer;
    }

} // namespace audio_io
