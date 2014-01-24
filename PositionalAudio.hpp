#pragma once
#include <atomic>
#include <thread>
#define ASIO_STANDALONE
#include <asio/io_service.hpp>
#include <asio/strand.hpp>

class PositionalAudio
{
    std::atomic_bool is_consumer_running {false};
    std::thread t;

    asio::io_service io_service;
    asio::io_service::strand player_strand {io_service};

    void consumer();

public:
    PositionalAudio();
    ~PositionalAudio();

    void start();
    void stop();
    void setListener(float x, float y, float z, float at_x, float at_y, float at_z, float up_x = 0.0f, float up_y = 1.0f, float up_z = 0.0f, float v = 0.0f);
};
