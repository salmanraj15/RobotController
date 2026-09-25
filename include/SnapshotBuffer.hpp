#pragma once

#include <array>
#include <atomic>
#include <cstdint>

#include "JointTypes.hpp"

class SnapshotBuffer
{
public:
    void publish(const RobotState &state) noexcept;

    bool read(RobotState &state) const noexcept;

private:
    static constexpr int max_read_attempts_ = 3;

    std::atomic<std::uint64_t> sequence_{0};

    std::array<std::atomic<double>, joint_count> positions_{};
    std::array<std::atomic<double>, joint_count> velocities_{};
};