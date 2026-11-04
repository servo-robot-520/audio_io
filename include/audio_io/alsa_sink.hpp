#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace audio_io {

    /// 播放设备抽象。
    ///
    /// 这样 PlaybackEngine 不直接依赖 ALSA：生产环境用 AlsaSink，CI/无硬件时用 NullSink。
    class AudioSink {
    public:
        virtual ~AudioSink() = default;

        /// 打开设备；失败时返回 false，并在 error 中写入 ALSA 错误文本。
        virtual bool open(std::string& error) = 0;

        /// 关闭设备；必须可重复调用。
        virtual void close() = 0;

        [[nodiscard]] virtual bool is_open() const noexcept = 0;

        /// 阻塞写入一段 S16_LE 单声道 PCM。
        /// @param samples 交错排列的 16-bit 样本
        /// @param frames 帧数（单声道时等于样本数）
        /// @param error 失败原因
        virtual bool write(const int16_t* samples, std::size_t frames, std::string& error) = 0;

        [[nodiscard]] virtual std::string name() const = 0;
    };

    /// ALSA 播放设备。
    ///
    /// Linux 音频栈的最底层是 ALSA。`plughw:` 设备会自动做采样率/格式/声道转换，
    /// 所以这里固定请求 S16_LE / 单声道 / 48 kHz，由 ALSA 插件适配具体硬件。
    class AlsaSink final : public AudioSink {
    public:
        AlsaSink(std::string device, uint32_t sample_rate, uint16_t channels,
                 uint32_t period_ms, uint32_t buffer_ms);

        ~AlsaSink() override;

        AlsaSink(const AlsaSink&) = delete;
        AlsaSink& operator=(const AlsaSink&) = delete;

        bool open(std::string& error) override;
        void close() override;
        [[nodiscard]] bool is_open() const noexcept override;
        bool write(const int16_t* samples, std::size_t frames, std::string& error) override;
        [[nodiscard]] std::string name() const override;

    private:
        std::string device_;
        uint32_t sample_rate_;
        /// 设备输出声道数，不是内容立体声；样本按 frames * channels 交错写入。
        uint16_t channels_;
        uint32_t period_ms_;
        uint32_t buffer_ms_;
        void* pcm_{nullptr};
    };

    /// 丢弃所有样本的假设备，用于无音频硬件的开发和测试。
    /// 它按播放时长 sleep，因此播放线程的节奏与真实设备一致。
    class NullSink final : public AudioSink {
    public:
        explicit NullSink(uint32_t sample_rate = 48000);

        bool open(std::string& error) override;
        void close() override;
        [[nodiscard]] bool is_open() const noexcept override;
        bool write(const int16_t* samples, std::size_t frames, std::string& error) override;
        [[nodiscard]] std::string name() const override;

    private:
        uint32_t sample_rate_;
        bool open_{false};
    };

} // namespace audio_io
