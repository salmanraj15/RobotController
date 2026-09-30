#pragma once

#include <atomic>
#include <cstdint>

#include "JointTypes.hpp"

class CanCommandBuffer
{
public:
    void publish(const JointCommand& command) noexcept;

    bool read(JointCommand& command) const noexcept;

private:
    std::atomic<std::uint64_t> sequence_{0};

    std::atomic<double> target_position_degrees_{0.0};
};