#pragma once

#include "Angle.hpp"
#include "AngularVelocity.hpp"

// Describes what the joint is currently doing.
struct JointState
{
    Angle position;
    AngularVelocity velocity;
};

// Describes what we want the joint to do.
struct JointCommand
{
    Angle target_position;
};