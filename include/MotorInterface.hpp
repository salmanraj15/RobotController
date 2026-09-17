#pragma once

#include "AngularAcceleration.hpp"

// Defines how a motor receives an acceleration command.
class MotorInterface
{
public:
    virtual ~MotorInterface() = default;

    // Sends an acceleration command to the motor.
    virtual void setAcceleration(AngularAcceleration acceleration) = 0;
};