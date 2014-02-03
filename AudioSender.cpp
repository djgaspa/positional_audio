#include <iostream>
#include <chrono>
#include <vector>
#include <array>
#include <asio.hpp>
#include <AL/alc.h>
#include <AL/al.h>
#include <opus/opus.h>
#include "AudioSender.hpp"

AudioSender::~AudioSender()
{
    stop();
}

void AudioSender::run(const std::string &address, const short port, const int sampling_frequency, const int frame_duration_ms, const int bitrate)
{
    const auto frame_duration = std::chrono::milliseconds(frame_duration_ms);
    const auto sample_duration = std::chrono::nanoseconds(static_cast<int>(1e9 / sampling_frequency));
    const auto samples_per_frame = frame_duration / sample_duration;
    asio::io_service io_service;
    asio::ip::udp::socket s(io_service, asio::ip::udp::v4());
    s.connect(asio::ip::udp::endpoint(asio::ip::address::from_string(address), port));
    auto* device = ::alcCaptureOpenDevice(nullptr, sampling_frequency, AL_FORMAT_MONO16, 1000 * samples_per_frame);
    if (device == nullptr) {
        std::cerr << "Error opening input device" << std::endl;
        return;
    }
    //std::cout << "Input device: " << ::alcGetString(device, ALC_CAPTURE_DEVICE_SPECIFIER) << std::endl;
    using clock = std::chrono::steady_clock;
    const auto t0 = clock::now();
    std::vector<short> samples(samples_per_frame);
    std::vector<unsigned char> buffer;
    auto* enc = ::opus_encoder_create(sampling_frequency, 1, OPUS_APPLICATION_VOIP, nullptr);
    if (enc == nullptr)
        std::cerr << "Error creating opus encoder" << std::endl;
    ::opus_encoder_ctl(enc, OPUS_SET_BITRATE(bitrate));
    ::opus_encoder_ctl(enc, OPUS_SET_DTX(1));
    ::opus_encoder_ctl(enc, OPUS_SET_MAX_BANDWIDTH(OPUS_BANDWIDTH_WIDEBAND));
    ::opus_encoder_ctl(enc, OPUS_SET_PACKET_LOSS_PERC(10));
    int dtx, vbr, complexity, max_bw, fec;
    ::opus_encoder_ctl(enc, OPUS_GET_DTX(&dtx));
    ::opus_encoder_ctl(enc, OPUS_GET_VBR(&vbr));
    ::opus_encoder_ctl(enc, OPUS_GET_COMPLEXITY(&complexity));
    ::opus_encoder_ctl(enc, OPUS_GET_MAX_BANDWIDTH(&max_bw));
    ::opus_encoder_ctl(enc, OPUS_GET_INBAND_FEC(&fec));
    std::cout << "DTX: " << (dtx == 0 ? "NO" : "YES") << std::endl;
    std::cout << "VBR: " << (vbr == 0 ? "NO" : "YES") << std::endl;
    std::cout << "Complexity: " << complexity << std::endl;
    std::cout << "Max Bandwidth: " << max_bw << std::endl;
    std::cout << "FEC: " << (fec == 0 ? "NO" : "YES") << std::endl;
    int n = 0;
    ::alcCaptureStart(device);
    unsigned timestamp = 0;
    auto deadline = clock::now();
    while (is_running == true) {
        deadline += frame_duration;
        std::this_thread::sleep_until(deadline);
        ::ALCint count = 0;
        ::alcGetIntegerv(device, ALC_CAPTURE_SAMPLES, sizeof(count), &count);
        if (count < samples_per_frame) {
            continue;
        }
        ::alcCaptureSamples(device, samples.data(), samples_per_frame);
        buffer.resize(256);
        const int ret = ::opus_encode(enc, samples.data(), samples_per_frame, buffer.data(), buffer.size());
        if (ret < 0)
            std::cerr << "Error encoding" << std::endl;
        if (ret > 1) {
            n += ret;
            buffer.resize(ret);
            std::array<asio::const_buffer, 2> send_buffer {
                asio::buffer(&timestamp, sizeof(timestamp)),
                        asio::buffer(buffer)
            };
            s.send(send_buffer);
        }
        timestamp += samples_per_frame;
    }
    ::alcCaptureStop(device);
    const auto t1 = clock::now();
    const double t = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000.0;
    std::cout << "Time: " << t << std::endl;
    std::cout << "Bandwidth: " << n * 8 / t << " bps" << std::endl;
    ::opus_encoder_destroy(enc);
    if (::alcCaptureCloseDevice(device) == ALC_FALSE)
        std::cerr << "Error closing the device" << std::endl;
}

void AudioSender::start()
{
    if (is_running == true)
        return;
    is_running = true;
    t = std::thread(&AudioSender::run, this, address, port, sampling_frequency, frame_duration_ms, bitrate);
}

void AudioSender::stop()
{
    if (is_running == false)
        return;
    is_running = false;
    t.join();
}
