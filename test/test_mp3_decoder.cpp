#include <cmath>

#include <gtest/gtest.h>

#include "audio_io/audio_decoder.hpp"
#include "audio_io/mp3_decoder.hpp"

namespace {

TEST(Mp3Decoder, DecodesTrimmedRobotStart) {
    const auto decoded = audio_io::Mp3Decoder::decode_file(TEST_MP3_FILE);
    ASSERT_TRUE(decoded.has_value()) << decoded.error();

    EXPECT_EQ(decoded->sample_rate, 22050U);
    EXPECT_EQ(decoded->channels, 2U);
    ASSERT_FALSE(decoded->samples.empty());

    const double duration = static_cast<double>(decoded->frames()) /
                            static_cast<double>(decoded->sample_rate);
    // ffmpeg -t 3.0 -c copy 会保留最后一个 MP3 帧，因此略大于 3 秒。
    EXPECT_NEAR(duration, 3.05, 0.1);

    double energy = 0.0;
    for (const float sample : decoded->samples) {
        energy += static_cast<double>(sample) * static_cast<double>(sample);
    }
    EXPECT_GT(energy, 0.0) << "decoded MP3 should contain non-silent samples";
}

TEST(AudioDecoder, DispatchesMp3ByExtension) {
    const auto decoded = audio_io::AudioDecoder::decode_file(TEST_MP3_FILE);
    ASSERT_TRUE(decoded.has_value()) << decoded.error();
    EXPECT_EQ(decoded->sample_rate, 22050U);
    EXPECT_EQ(decoded->channels, 2U);
}

} // namespace
