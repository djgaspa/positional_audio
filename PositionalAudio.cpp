#include <iostream>
#include <chrono>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <asio.hpp>
#include <asio/steady_timer.hpp>
#include <AL/alc.h>
#include <AL/al.h>
#include <opus/opus.h>
#include <speex/speex_jitter.h>
#include "PositionalAudio.hpp"

void PositionalAudio::consumer(const unsigned short port, const int sampling_frequency, const int frame_duration_ms)
{
    const auto frame_duration = std::chrono::milliseconds(frame_duration_ms);
    const auto sample_duration = std::chrono::nanoseconds(static_cast<int>(1e9 / sampling_frequency));
    const auto samples_per_frame = frame_duration / sample_duration;

    using Lock = std::unique_lock<std::mutex>;
    std::mutex m;
    std::unordered_map<unsigned long long, std::shared_ptr<JitterBuffer>> jitter_buffers(32);

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
        std::vector<ALuint> buffers, available_buffers;
        std::vector<short> decoded_samples;
        std::shared_ptr<OpusDecoder> dec;
        Peer(const int freq, const int n_buffers) :
            buffers(n_buffers), dec{::opus_decoder_create(freq, 1, nullptr), &::opus_decoder_destroy}
        {
            if (dec == nullptr)
                std::cerr << "Error creating opus decoder" << std::endl;
            ::alGenSources(src.size(), src.data());
            ::alGenBuffers(buffers.size(), buffers.data());
            available_buffers = buffers;
        }
        ~Peer()
        {
            ::alDeleteBuffers(buffers.size(), buffers.data());
            ::alDeleteSources(src.size(), src.data());
        }
    };

    std::unordered_map<unsigned long long, std::shared_ptr<Peer>> actors;

    auto init_actor = [&] (const unsigned long long id) {
        auto p = std::make_shared<Peer>(sampling_frequency, std::chrono::milliseconds(100) / frame_duration);
        actors.emplace(id, p);
    };

    using Clock = std::chrono::steady_clock;
    asio::steady_timer player_timer(io_service, Clock::now());
    std::function<void()> start_player = [&] () {
        auto player_handler = [&] (std::error_code e) {
            if (e == asio::error::operation_aborted)
                return;
            start_player();
            for (auto it : actors) {
                const auto id = it.first;
                auto actor = it.second;
                ALint n_buffers_processed = 0;
                ::alGetSourcei(actor->src[0], AL_BUFFERS_PROCESSED, &n_buffers_processed);
                std::vector<ALuint> processed_buffers(n_buffers_processed);
                ::alSourceUnqueueBuffers(actor->src[0], processed_buffers.size(), processed_buffers.data());
                actor->available_buffers.insert(actor->available_buffers.end(), processed_buffers.begin(), processed_buffers.end());
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
                int timestamp;
                const int get_ret = ::jitter_buffer_get(jb.get(), &p, samples_per_frame, &timestamp);
                ::jitter_buffer_tick(jb.get());
                l.unlock();
                switch (get_ret) {
                case JITTER_BUFFER_OK: {
                    actor->decoded_samples.resize(samples_per_frame);
                    const int ret = ::opus_decode(actor->dec.get(), (unsigned char*)p.data, p.len, actor->decoded_samples.data(), actor->decoded_samples.size(), 0);
                    if (ret <= 0) {
                        std::cerr << "Error decoding stream" << std::endl;
                        continue;
                    }
                    actor->decoded_samples.resize(ret);
                    break;
                }
                case JITTER_BUFFER_INSERTION:
                case JITTER_BUFFER_MISSING: {
                    actor->decoded_samples.resize(p.span);
                    const int ret = ::opus_decode(actor->dec.get(), nullptr, p.span, actor->decoded_samples.data(), actor->decoded_samples.size(), 0);
                    actor->decoded_samples.resize(ret);
                    if (ret <= 0) {
                        std::cerr << "Error decoding stream" << std::endl;
                        continue;
                    }
                    break;
                }
                default:
                    std::cerr << "Critical error. Unknown value returned by jitter_buffer_get" << std::endl;
                    continue;
                }
                if (actor->available_buffers.empty() == true) {
                    continue;
                }
                const auto i = actor->available_buffers.back();
                actor->available_buffers.pop_back();
                ::alBufferData(i, AL_FORMAT_MONO16, actor->decoded_samples.data(), actor->decoded_samples.size() * sizeof(short), sampling_frequency);
                ::alSourceQueueBuffers(actor->src[0], 1, &i);
                ALint state = 0;
                ::alGetSourcei(actor->src[0], AL_SOURCE_STATE, &state);
                if (state != AL_PLAYING) {
                    ::alSourcePlay(actor->src[0]);
                }
            }
        };
        player_timer.expires_at(player_timer.expires_at() + frame_duration);
        player_timer.async_wait(player_strand.wrap(player_handler));
    };

    asio::io_service::strand receiver_strand(io_service);
    std::unordered_map<unsigned long long, std::shared_ptr<asio::steady_timer>> timers(32);
    auto start_deadline_timer = [&] (const unsigned long long id) {
        auto it = timers.find(id);
        if (it == timers.end())
            it = timers.emplace(id, std::make_shared<asio::steady_timer>(io_service)).first;
        auto& t = *it->second;
        t.expires_from_now(std::chrono::seconds(1));
        t.async_wait(receiver_strand.wrap([id, &m, &jitter_buffers, &timers] (std::error_code e) {
            if (e == asio::error::operation_aborted)
                return;
            timers.erase(id);
            Lock l(m);
            jitter_buffers.erase(id);
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
        auto receiver_handler = [&] (std::error_code e, std::size_t n) {
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
                player_strand.post(std::bind(init_actor, id));
            }
            assert(jb.get() != nullptr);
            ::jitter_buffer_put(jb.get(), &p);
            l.unlock();
            start_read();
            start_deadline_timer(id);
        };
        s.async_receive_from(receive_buffer, endpoint, receiver_strand.wrap(receiver_handler));
    };

    start_read();
    start_player();
    std::array<std::thread, 2> thread_pool;
    for (auto& t : thread_pool)
        t = std::thread([this] {
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
}

PositionalAudio::PositionalAudio()
{
}

PositionalAudio::~PositionalAudio()
{
    stop();
}

void PositionalAudio::start()
{
    if (is_consumer_running)
        return;
    is_consumer_running = true;
    t = std::thread(&PositionalAudio::consumer, this, this->port, m_sampling_frequency, this->frame_duration_ms);
}

void PositionalAudio::stop()
{
    if (is_consumer_running == false)
        return;
    is_consumer_running = false;
    t.join();
}

void PositionalAudio::setListener(float x, float y, float z, float at_x, float at_y, float at_z, float up_x, float up_y, float up_z, float v)
{
    player_strand.post([=] {
        float orientation[6] {at_x, at_y, at_z, up_x, up_y, up_z};
        ::alListener3f(AL_POSITION, x, y, z);
        ::alListenerfv(AL_ORIENTATION, orientation);
        ::alListenerf(AL_VELOCITY, v);
    });
}
