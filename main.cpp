#include <iostream>
#include <vector>
#include <queue>
#include <array>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <AL/alc.h>
#include <AL/al.h>
#include <opus/opus.h>
#include <speex/speex_jitter.h>
#include <asio.hpp>

int main()
{
    const int sampling_frequency = 16000;
    const auto frame_duration = std::chrono::milliseconds(20);
    const auto sample_duration = std::chrono::nanoseconds(static_cast<int>(1e9 / sampling_frequency));
    const auto samples_per_frame = frame_duration / sample_duration;
    const int port = 40321;
    std::atomic_bool is_running {true};

    std::thread producer([&] {
        const int bitrate = 16000;
        asio::io_service io_service;
        asio::ip::udp::socket s(io_service, asio::ip::udp::v4());
        s.connect(asio::ip::udp::endpoint(asio::ip::address::from_string("127.0.0.1"), port));
        auto* device = ::alcCaptureOpenDevice(nullptr, sampling_frequency, AL_FORMAT_MONO16, 2 * samples_per_frame);
        if (device == nullptr) {
            std::cerr << "Error opening input device" << std::endl;
            return;
        }
        //std::cout << "Input device: " << ::alcGetString(device, ALC_CAPTURE_DEVICE_SPECIFIER) << std::endl;
        using clock = std::chrono::high_resolution_clock;
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
        while (is_running == true) {
            ::ALCint count = 0;
            ::alcGetIntegerv(device, ALC_CAPTURE_SAMPLES, 1, &count);
            if (count < samples_per_frame) {
                std::this_thread::sleep_for(frame_duration / 4);
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
    });

    std::thread consumer([&] {
        std::mutex m;
        using Lock = std::unique_lock<std::mutex>;
        JitterBuffer* jb = ::jitter_buffer_init(samples_per_frame);
        asio::io_service io_service;
        asio::ip::udp::socket s(io_service, asio::ip::udp::endpoint(asio::ip::udp::v4(), port));
        asio::ip::udp::endpoint endpoint;
        std::vector<unsigned char> packet;
        int timestamp;
        std::function<void()> start_read = [&] {
            packet.resize(256);
            std::array<asio::mutable_buffer, 2> receive_buffer = {
                asio::buffer(&timestamp, sizeof(timestamp)),
                asio::buffer(packet)
            };
            s.async_receive_from(receive_buffer, endpoint, [&] (std::error_code e, std::size_t n) {
                packet.resize(n - 4);
                JitterBufferPacket p;
                p.data = (char*)packet.data();
                p.len = packet.size();
                p.span = samples_per_frame;
                p.timestamp = timestamp;
                Lock l(m);
                ::jitter_buffer_put(jb, &p);
                start_read();
            });
        };
        start_read();
        std::thread receiver([&io_service] {
            io_service.run();
        });

        auto* output_device = ::alcOpenDevice(nullptr);
        if (output_device == nullptr)
            std::cerr << "Error opening output device" << std::endl;
        //std::cout << "Output device: " << ::alcGetString(output_device, ALC_DEVICE_SPECIFIER) << std::endl;
        auto* ctx = ::alcCreateContext(output_device, nullptr);
        if (ctx == nullptr)
            std::cerr << "Error creating audio context " << ::alGetError() << std::endl;
        ::alcMakeContextCurrent(ctx);
        std::array<ALuint, 1> src;
        std::vector<ALuint> buffers(std::chrono::milliseconds(80) / frame_duration);
        ::alGenSources(src.size(), src.data());
        ::alGenBuffers(buffers.size(), buffers.data());
        auto* dec = ::opus_decoder_create(sampling_frequency, 1, nullptr);
        if (dec == nullptr)
            std::cerr << "Error creating opus decoder" << std::endl;
        std::vector<short> decoded_samples(samples_per_frame);
        for (int i = 0; i < buffers.size(); ++i) {
            const auto ret = ::opus_decode(dec, nullptr, 0, decoded_samples.data(), decoded_samples.size(), 0);
            if (ret <= 0) {
                std::cerr << "Error initializing audio buffer: " << ret << " : " << ::opus_strerror(ret) << std::endl;
            }
            ::alBufferData(buffers[i], AL_FORMAT_MONO16, decoded_samples.data(), decoded_samples.size() * sizeof(short), sampling_frequency);
        }
        ::alSourceQueueBuffers(src[0], buffers.size(), buffers.data());
        ::alSourcePlay(src[0]);
        while (is_running) {
            ALint n_buffers_processed = 0;
            ::alGetSourcei(src[0], AL_BUFFERS_PROCESSED, &n_buffers_processed);
            if (n_buffers_processed == 0) {
                std::this_thread::sleep_for(frame_duration / 4);
                continue;
            }
            std::vector<ALuint> processed_buffers(n_buffers_processed);
            ::alSourceUnqueueBuffers(src[0], processed_buffers.size(), processed_buffers.data());
            while (processed_buffers.empty() == false) {
                std::array<unsigned char, 1024> data;
                JitterBufferPacket p;
                p.data = (char*)data.data();
                p.len = data.size();
                Lock l(m);
                const bool is_missing = ::jitter_buffer_get(jb, &p, samples_per_frame, nullptr) != JITTER_BUFFER_OK;
                ::jitter_buffer_tick(jb);
                l.unlock();
                const unsigned char* ptr = is_missing ? nullptr : (unsigned char*)p.data;
                const int size = is_missing ? 0 : p.len;
                decoded_samples.resize(samples_per_frame);
                const int ret = ::opus_decode(dec, ptr, size, decoded_samples.data(), decoded_samples.size(), 0);
                if (ret <= 0) {
                    std::cerr << "Error decoding stream" << std::endl;
                    continue;
                }
                decoded_samples.resize(ret);
                const auto i = processed_buffers.back();
                processed_buffers.pop_back();
                ::alBufferData(i, AL_FORMAT_MONO16, decoded_samples.data(), decoded_samples.size() * sizeof(short), sampling_frequency);
                ::alSourceQueueBuffers(src[0], 1, &i);
                ALint state = 0;
                ::alGetSourcei(src[0], AL_SOURCE_STATE, &state);
                if (state != AL_PLAYING) {
                    std::cout << "Restarting play" << std::endl;
                    ::alSourcePlay(src[0]);
                }
            }
        }
        io_service.stop();
        receiver.join();
        ::opus_decoder_destroy(dec);
        ALint state = AL_PLAYING;
        while (state == AL_PLAYING) {
            std::this_thread::sleep_for(frame_duration);
            ::alGetSourcei(src[0], AL_SOURCE_STATE, &state);
        }

        ::alDeleteBuffers(buffers.size(), buffers.data());
        ::alDeleteSources(src.size(), src.data());
        ::alcMakeContextCurrent(nullptr);
        ::alcDestroyContext(ctx);
        if (::alcCloseDevice(output_device) == ALC_FALSE)
            std::cerr << "Error closing the output device" << std::endl;
        ::jitter_buffer_destroy(jb);
    });

    std::cin.get();
    is_running = false;
    producer.join();
    consumer.join();
}
