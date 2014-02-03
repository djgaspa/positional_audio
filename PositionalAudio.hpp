#pragma once
#include <atomic>
#include <thread>
#include <asio/io_service.hpp>
#include <asio/strand.hpp>

class PositionalAudio
{
    std::atomic_bool is_consumer_running {false};
    std::thread t;

    asio::io_service io_service;
    asio::io_service::strand player_strand {io_service};

    unsigned short port = 40321;
    int m_sampling_frequency = 16000;
    int frame_duration_ms = 20;

    void consumer(const unsigned short port, const int sampling_frequency, const int frame_duration_ms);

public:
    PositionalAudio();
    ~PositionalAudio();

    void start();
    void stop();
    void setPort(const unsigned short port) {
        this->port = port;
    }
    void setPlayer(const int sampling_freq, const int frame_size) {
        this->m_sampling_frequency = sampling_freq;
        this->frame_duration_ms = frame_size;
    }

    void setListener(float x, float y, float z, float at_x, float at_y, float at_z, float up_x = 0.0f, float up_y = 1.0f, float up_z = 0.0f, float v = 0.0f);
};
