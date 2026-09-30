#pragma once

#include <atomic>
#include <thread>

#include "CanCommandBuffer.hpp"
#include "CanStateBuffer.hpp"
#include "ICanInterface.hpp"

class CanCommunication
{
public:
    CanCommunication(
        ICanInterface &can,
        CanCommandBuffer &command_buffer,
        CanStateBuffer &state_buffer);

    ~CanCommunication();

    void start();
    void stop();

private:
    void run();
    void transmitState();
    
    ICanInterface &can_;
    CanCommandBuffer &command_buffer_;
    CanStateBuffer &state_buffer_;

    std::jthread thread_;
    std::atomic<bool> running_{false};
};