#include "SimulatedCan.hpp"

bool SimulatedCan::receive(CanFrame& frame) noexcept
{
    std::lock_guard lock{mutex_};

    if (received_frames_.empty())
        return false;

    frame = received_frames_.front();
    received_frames_.pop_front();

    return true;
}

bool SimulatedCan::transmit(
    const CanFrame& frame) noexcept
{
    std::lock_guard lock{mutex_};

    // Loop transmitted frames back into reception.
    received_frames_.push_back(frame);

    return true;
}