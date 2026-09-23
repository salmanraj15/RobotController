#pragma once

#include <array>
#include <atomic>
#include <cstdint>

#include "JointTypes.hpp"

class SnapshotBuffer
{
public:
    void publish(const RobotState &state) noexcept;

    RobotState read() const noexcept;

private:
    static constexpr std::size_t joint_count = 6;

    std::atomic<std::uint64_t> sequence_{0};

    std::array<std::atomic<double>, joint_count> positions_{};
    std::array<std::atomic<double>, joint_count> velocities_{};
};