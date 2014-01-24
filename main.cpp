#include <iostream>
#include "AudioSender.hpp"
#include "PositionalAudio.hpp"

int main()
{
    AudioSender sender;
    sender.start();
    PositionalAudio receiver;
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
