#pragma once

#include <atomic>
#include <cstdint>

#include "robot_controller/core/JointTypes.hpp"

class CanStateBuffer
{
public:
    void publish(const JointState& state) noexcept;

    bool read(JointState& state) const noexcept;

private:
    std::atomic<std::uint64_t> sequence_{0};

    std::atomic<double> position_degrees_{0.0};
    std::atomic<double> velocity_degrees_per_second_{0.0};
};