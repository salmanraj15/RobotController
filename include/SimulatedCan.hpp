#pragma once

#include <deque>
#include <mutex>

#include "ICanInterface.hpp"

class SimulatedCan final : public ICanInterface
{
public:
    bool receive(CanFrame& frame) noexcept override;
    bool transmit(const CanFrame& frame) noexcept override;

private:
    std::deque<CanFrame> received_frames_;
    mutable std::mutex mutex_;
};