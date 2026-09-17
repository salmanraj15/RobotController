#pragma once

#include "Joint.hpp"
#include "AngularAcceleration.hpp"

class SafetyLayer
{
public:
    // Checks whether an acceleration command is within the joint's limits.
    bool validate(
        const Joint& joint,
        AngularAcceleration requested) const;
};