#pragma once

#include "Joint.hpp"
#include "AngularAcceleration.hpp"

class SafetyLayer
{
public:
    bool validate(
        const Joint& joint,
        AngularAcceleration requested) const;
};