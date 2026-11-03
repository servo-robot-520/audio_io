#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "audio_io/audio_format.hpp"
#include "audio_io/wav_decoder.hpp"

namespace {

void append_u16(std::vector<uint8_t>& bytes, const uint16_t value) {
    bytes.push_back(static_cast<uint8_t>(value & 0xFFU));
    bytes.push_back(static_cast<uint8_t>((value >> 8U) & 0xFFU));
}

void append_u32(std::vector<uint8_t>& bytes, const uint32_t value) {
    bytes.push_back(static_cast<uint8_t>(value & 0xFFU));
    bytes.push_back(static_cast<uint8_t>((value >> 8U) & 0xFFU));
    bytes.push_back(static_cast<uint8_t>((value >> 16U) & 0xFFU));
    bytes.push_back(static_cast<uint8_t>((value >> 24U) & 0xFFU));
}

/// 生成一个最小的 PCM16 单声道 WAV，用于验证 RIFF chunk 解析与样本归一化。
std::vector<uint8_t> make_pcm16_wav(const uint32_t sample_rate, const uint16_t channels,
                                    const std::vector<int16_t>& samples) {
    const uint32_t data_size = static_cast<uint32_t>(samples.size() * sizeof(int16_t));
    std::vector<uint8_t> bytes;
    bytes.insert(bytes.end(), {'R', 'I', 'F', 'F'});
    append_u32(bytes, 36U + data_size);
    bytes.insert(bytes.end(), {'W', 'A', 'V', 'E'});
    bytes.insert(bytes.end(), {'f', 'm', 't', ' '});
    append_u32(bytes, 16U);
    append_u16(bytes, 1U);
    append_u16(bytes, channels);
    append_u32(bytes, sample_rate);
    append_u32(bytes, sample_rate * channels * 2U);
    append_u16(bytes, static_cast<uint16_t>(channels * 2U));
    append_u16(bytes, 16U);
    bytes.insert(bytes.end(), {'d', 'a', 't', 'a'});
    append_u32(bytes, data_size);
    for (const int16_t sample : samples) {
        append_u16(bytes, static_cast<uint16_t>(sample));
    }
    return bytes;
}

} // namespace

TEST(WavDecoder, DecodesPcm16Mono) {
    std::vector<int16_t> samples;
    for (int i = 0; i < 480; ++i) {
        samples.push_back(static_cast<int16_t>(
            std::lround(16000.0 * std::sin(2.0 * M_PI * 440.0 * i / 48000.0))));
    }

    const auto wav = make_pcm16_wav(48000, 1, samples);
    const auto decoded = audio_io::WavDecoder::decode_bytes(wav);
    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->sample_rate, 48000U);
    EXPECT_EQ(decoded->channels, 1U);
    ASSERT_EQ(decoded->samples.size(), samples.size());
    EXPECT_NEAR(decoded->samples[0], 0.0F, 1e-4F);
}

TEST(WavDecoder, RejectsNonWaveContainer) {
    const std::vector<uint8_t> bytes{'N', 'O', 'T', 'W', 'A', 'V', 'E'};
    const auto decoded = audio_io::WavDecoder::decode_bytes(bytes);
    EXPECT_FALSE(decoded.has_value());
}

TEST(AudioFormat, DownsamplesAndResamplesToMono48k) {
    audio_io::AudioBuffer buffer;
    buffer.sample_rate = 44100;
    buffer.channels = 2;
    buffer.samples.resize(44100 * 2);
    for (std::size_t frame = 0; frame < 44100; ++frame) {
        buffer.samples[frame * 2] = 0.5F;
        buffer.samples[frame * 2 + 1] = -0.5F;
    }

    const auto mono = audio_io::to_mono_48k(buffer);
    // 左右声道相加为 0，下混后应接近静音；长度应接近 1 秒的 48000 帧。
    ASSERT_EQ(mono.size(), 48000U);
    for (const float sample : mono) {
        EXPECT_NEAR(sample, 0.0F, 1e-6F);
    }
}
