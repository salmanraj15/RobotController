#pragma once

#include <array>
#include <cstddef>

#include "Angle.hpp"
#include "AngularVelocity.hpp"

inline constexpr std::size_t joint_count = 6;

struct JointIndex
{
    std::size_t value{};

    constexpr explicit JointIndex(std::size_t index)
        : value{index}
    {
    }
};

// Describes what the joint is currently doing.
struct JointState
{
    Angle position{0.0};
    AngularVelocity velocity{0.0};
};

// Describes what we want the joint to do.
struct JointCommand
{
    // A zero-degree target is a safe initial command.
    Angle target_position{0.0};
};
// Holds a snapshot of all six joint states.
using RobotState = std::array<JointState, joint_count>;