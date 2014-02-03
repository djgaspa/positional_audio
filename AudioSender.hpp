#pragma once
#include <atomic>
#include <thread>
#include <string>

class AudioSender
{
    std::atomic_bool is_running {false};
    std::thread t;

    std::string address = "127.0.0.1";
    unsigned short port = 40321;

    int sampling_frequency = 16000;
    int frame_duration_ms = 20;
    int bitrate = 16000;

    void run(const std::string& address, const short port, const int sampling_freq, const int frame_duration_ms, const int bitrate);

public:
    ~AudioSender();
    void start();
    void stop();
    void restart() {
        stop();
        start();
    }
    void setDestination(const std::string& address, const unsigned short port) {
        this->address = address;
        this->port = port;
    }
    void setCapture(const int freq, const int frame_ms) {
        sampling_frequency = freq;
        frame_duration_ms = frame_ms;
    }
    void setBitrate(const int bitrate) {
        this->bitrate = bitrate;
    }
};
