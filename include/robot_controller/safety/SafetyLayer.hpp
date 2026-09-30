#pragma once

#include "robot_controller/core/Joint.hpp"
#include "robot_controller/core/AngularAcceleration.hpp"

class SafetyLayer
{
public:
    // Checks whether an acceleration command is within the joint's limits.
    bool validate(
        const Joint& joint,
        AngularAcceleration requested) const noexcept;
};