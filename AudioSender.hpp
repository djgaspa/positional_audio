#pragma once
#include <atomic>
#include <thread>

class AudioSender
{
    std::atomic_bool is_running {false};
    std::thread t;

    void run();

public:
    ~AudioSender();
    void start();
    void stop();
};
