project(AudioCapture)
cmake_minimum_required(VERSION 2.8.10)
add_definitions(-std=c++11)
find_library(OPENAL_LIBRARY openal)
find_library(OPUS_LIBRARY opus)
find_library(SPEEXDSP_LIBRARY speexdsp)
find_path(ASIO_INCLUDE_DIR asio.hpp)
find_path(OPENAL_INCLUDE_DIR AL/al.h)
include_directories(${OPENAL_INCLUDE_DIR} ${ASIO_INCLUDE_DIR})
add_executable(main main.cpp
PositionalAudio.hpp PositionalAudio.cpp
AudioSender.hpp AudioSender.cpp)
target_link_libraries(main ${OPENAL_LIBRARY} ${OPUS_LIBRARY} ${SPEEXDSP_LIBRARY})
