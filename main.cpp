#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
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
#include <boost/date_time.hpp>
#include <asio.hpp>

int main()
{
    const int sampling_frequency = 16000;
    const auto frame_duration = std::chrono::milliseconds(20);
    const auto sample_duration = std::chrono::nanoseconds(static_cast<int>(1e9 / sampling_frequency));
    const auto samples_per_frame = frame_duration / sample_duration;
    const int port = 40321;

    std::atomic_bool is_producer_running {true};
    auto producer_function = [&] {
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
        while (is_producer_running == true) {
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
    };
    std::thread producer(producer_function);

    std::atomic_bool is_consumer_running {true};
    std::thread consumer([&] {
        std::mutex m;
        using Lock = std::unique_lock<std::mutex>;
        std::unordered_map<unsigned long long, std::shared_ptr<JitterBuffer>> jitter_buffers(32);

        asio::io_service io_service;

        std::shared_ptr<ALCdevice> output_device(::alcOpenDevice(nullptr), [] (ALCdevice* dev) {
            if (::alcCloseDevice(dev) == ALC_FALSE)
                std::cerr << "Error closing the output device" << std::endl;
        });
        if (output_device == nullptr)
            std::cerr << "Error opening output device" << std::endl;
        //std::cout << "Output device: " << ::alcGetString(output_device, ALC_DEVICE_SPECIFIER) << std::endl;
        std::shared_ptr<ALCcontext> ctx(::alcCreateContext(output_device.get(), nullptr), &::alcDestroyContext);
        if (ctx == nullptr)
            std::cerr << "Error creating audio context " << ::alGetError() << std::endl;
        ::alcMakeContextCurrent(ctx.get());

        struct Peer
        {
            std::array<ALuint, 1> src;
            std::vector<ALuint> buffers;
            std::vector<short> decoded_samples;
            std::shared_ptr<OpusDecoder> dec{::opus_decoder_create(sampling_frequency, 1, nullptr), &::opus_decoder_destroy};
            Peer(const int n_buffers, const int samples_per_frame) : buffers(n_buffers), decoded_samples(samples_per_frame) {
                if (dec == nullptr)
                    std::cerr << "Error creating opus decoder" << std::endl;
                ::alGenSources(src.size(), src.data());
                ::alGenBuffers(buffers.size(), buffers.data());
            }
            ~Peer() {
                ::alDeleteBuffers(buffers.size(), buffers.data());
                ::alDeleteSources(src.size(), src.data());
            }
        };

        std::unordered_map<unsigned long long, std::shared_ptr<Peer>> actors;

        auto init_actor = [&] (const unsigned long long id, std::shared_ptr<Peer> p) {
            for (int i = 0; i < p->buffers.size(); ++i) {
                const auto ret = ::opus_decode(p->dec.get(), nullptr, 0, p->decoded_samples.data(), p->decoded_samples.size(), 0);
                if (ret <= 0)
                    std::cerr << "Error initializing audio buffer: " << ret << " : " << ::opus_strerror(ret) << std::endl;
                ::alBufferData(p->buffers[i], AL_FORMAT_MONO16, p->decoded_samples.data(), p->decoded_samples.size() * sizeof(short), sampling_frequency);
            }
            ::alSourceQueueBuffers(p->src[0], p->buffers.size(), p->buffers.data());
            ::alSourcePlay(p->src[0]);
            actors.emplace(id, p);
        };

        asio::deadline_timer player_timer(io_service);
        asio::io_service::strand player_strand(io_service);
        std::function<void()> start_timer = [&] () {
            const boost::posix_time::microseconds interval(std::chrono::duration_cast<std::chrono::microseconds>(frame_duration / 4).count());
            player_timer.expires_from_now(interval);
            auto player_task = player_strand.wrap([&] (std::error_code e) {
                start_timer();
                for (auto it : actors) {
                    const auto id = it.first;
                    auto actor = it.second;
                    ALint n_buffers_processed = 0;
                    ::alGetSourcei(actor->src[0], AL_BUFFERS_PROCESSED, &n_buffers_processed);
                    if (n_buffers_processed == 0)
                        return;
                    std::vector<ALuint> processed_buffers(n_buffers_processed);
                    ::alSourceUnqueueBuffers(actor->src[0], processed_buffers.size(), processed_buffers.data());
                    while (processed_buffers.empty() == false) {
                        std::array<unsigned char, 1024> data;
                        JitterBufferPacket p;
                        p.data = (char*)data.data();
                        p.len = data.size();
                        Lock l(m);
                        const auto jb_it = jitter_buffers.find(id);
                        if (jb_it == jitter_buffers.end()) {
                            ALint n_buffers_queued = 0;
                            ::alGetSourcei(actor->src[0], AL_BUFFERS_QUEUED, &n_buffers_queued);
                            if (n_buffers_queued == 0)
                                actors.erase(id);
                            return;
                        }
                        auto jb = jb_it->second;
                        const bool is_missing = ::jitter_buffer_get(jb.get(), &p, samples_per_frame, nullptr) != JITTER_BUFFER_OK;
                        ::jitter_buffer_tick(jb.get());
                        l.unlock();
                        const unsigned char* ptr = is_missing ? nullptr : (unsigned char*)p.data;
                        const int size = is_missing ? 0 : p.len;
                        actor->decoded_samples.resize(samples_per_frame);
                        const int ret = ::opus_decode(actor->dec.get(), ptr, size, actor->decoded_samples.data(), actor->decoded_samples.size(), 0);
                        if (ret <= 0) {
                            std::cerr << "Error decoding stream" << std::endl;
                            continue;
                        }
                        actor->decoded_samples.resize(ret);
                        const auto i = processed_buffers.back();
                        processed_buffers.pop_back();
                        ::alBufferData(i, AL_FORMAT_MONO16, actor->decoded_samples.data(), actor->decoded_samples.size() * sizeof(short), sampling_frequency);
                        ::alSourceQueueBuffers(actor->src[0], 1, &i);
                        ALint state = 0;
                        ::alGetSourcei(actor->src[0], AL_SOURCE_STATE, &state);
                        if (state != AL_PLAYING) {
                            std::cout << "Restarting play" << std::endl;
                            ::alSourcePlay(actor->src[0]);
                        }
                    }
                }
            });
            player_timer.async_wait(player_task);
        };

        asio::io_service::strand receiver_strand(io_service);
        std::unordered_map<unsigned long long, std::shared_ptr<asio::deadline_timer>> timers(32);
        auto start_deadline_timer = [&] (const unsigned long long id) {
            auto it = timers.find(id);
            if (it == timers.end())
                it = timers.emplace(id, std::make_shared<asio::deadline_timer>(io_service)).first;
            auto& t = *it->second;
            t.expires_from_now(boost::posix_time::seconds(1));
            t.async_wait(receiver_strand.wrap([id, &jitter_buffers, &timers] (std::error_code e) {
                if (e == asio::error::operation_aborted)
                    return;
                jitter_buffers.erase(id);
                timers.erase(id);
            }));
        };

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
            s.async_receive_from(receive_buffer, endpoint, receiver_strand.wrap([&] (std::error_code e, std::size_t n) {
                const auto address = endpoint.address().to_v4().to_ulong();
                const auto port = endpoint.port();
                const auto id = (static_cast<unsigned long long>(address) << 16) + port;
                packet.resize(n - 4);
                JitterBufferPacket p;
                p.data = (char*)packet.data();
                p.len = packet.size();
                p.span = samples_per_frame;
                p.timestamp = timestamp;
                Lock l(m);
                auto ret = jitter_buffers.emplace(id, nullptr);
                auto& jb = ret.first->second;
                if (ret.second == true) {
                    jb.reset(::jitter_buffer_init(samples_per_frame), &::jitter_buffer_destroy);
                    player_strand.post(std::bind(init_actor, id, std::make_shared<Peer>(std::chrono::milliseconds(80) / frame_duration, samples_per_frame)));
                }
                assert(jb.get() != nullptr);
                ::jitter_buffer_put(jb.get(), &p);
                l.unlock();
                start_read();
                start_deadline_timer(id);
            }));
        };

        start_read();
        start_timer();
        std::array<std::thread, 2> thread_pool;
        for (auto& t : thread_pool)
            t = std::thread([&io_service] {
                io_service.run();
            });
        while (is_consumer_running == true)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        io_service.stop();
        for (auto& t : thread_pool)
            t.join();
        for (auto& it : actors) {
            auto& actor = it.second;
            ALint state = AL_PLAYING;
            while (state == AL_PLAYING) {
                std::this_thread::sleep_for(frame_duration);
                ::alGetSourcei(actor->src[0], AL_SOURCE_STATE, &state);
            }
        };
        actors.clear();

        ::alcMakeContextCurrent(nullptr);
    });

    while (std::cin) {
        std::string command;
        std::getline(std::cin, command);
        if (command == "pause" && is_producer_running == true) {
            is_producer_running = false;
            producer.join();
        }
        else if (command == "rec" && is_producer_running == false) {
            is_producer_running = true;
            producer = std::thread(producer_function);
        }
        else if (command == "") {
            if (is_producer_running) {
                is_producer_running = false;
                producer.join();
            }
            if (is_consumer_running) {
                is_consumer_running = false;
                consumer.join();
            }
            break;
        }
        else
            std::cout << "Unknown command" << std::endl;
    }
}
