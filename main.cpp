#include <iostream>
#include <string>
#include "AudioSender.hpp"
#include "PositionalAudio.hpp"

int main(int argc, char** argv)
{
    const std::string address = (argc > 1 ? argv[1] : "127.0.0.1");
    const unsigned short port = 40321;
    const int sampling_frequency = 16000;
    const int frame_size = 10;
    AudioSender sender;
    sender.setDestination(address, port);
    sender.setCapture(sampling_frequency, frame_size);
    sender.setBitrate(16000);
    sender.start();
    PositionalAudio receiver;
    receiver.setPort(port);
    receiver.setPlayer(sampling_frequency, frame_size);
    receiver.start();

    while (std::cin) {
        std::string command;
        std::getline(std::cin, command);
        if (command == "pause")
            sender.stop();
        else if (command == "rec")
            sender.start();
        else if (command == "")
            break;
        else
            std::cout << "Unknown command" << std::endl;
    }
}
