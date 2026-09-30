#include "robot_controller/communication/CanCommunication.hpp"
#include "robot_controller/communication/JointStateCodec.hpp"
#include "robot_controller/communication/JointCommandCodec.hpp"

#include <chrono>

CanCommunication::CanCommunication(
    ICanInterface &can,
    CanCommandBuffer &command_buffer,
    CanStateBuffer &state_buffer)
    : can_{can},
      command_buffer_{command_buffer},
      state_buffer_{state_buffer}
{
}

CanCommunication::~CanCommunication()
{
    stop();
}

void CanCommunication::start()
{
    if (running_.exchange(true))
        return;

    thread_ = std::jthread{
        [this]
        {
            run();
        }};
}

void CanCommunication::stop()
{
    running_.store(false);

    if (thread_.joinable())
        thread_.join();
}

void CanCommunication::run()
{
    constexpr auto communication_period =
        std::chrono::milliseconds{1};

    while (running_.load())
    {
        CanFrame frame;

        if (can_.receive(frame))
        {
            JointCommand command;

            if (JointCommandCodec::decode(
                    frame,
                    command))
            {
                command_buffer_.publish(command);
            }
        }

        transmitState();

        std::this_thread::sleep_for(
            communication_period);
    }
}

void CanCommunication::transmitState()
{
    JointState state;

    if (!state_buffer_.read(state))
        return;

    const CanFrame frame =
        JointStateCodec::encode(state);

    can_.transmit(frame);
}