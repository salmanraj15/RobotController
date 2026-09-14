#pragma once

#include "Angle.hpp"
#include "AngularAcceleration.hpp"
#include "AngularVelocity.hpp"

class PDController
{
public:
    PDController(
        double position_gain,
        double velocity_gain,
        AngularAcceleration max_acceleration);

    AngularAcceleration calculate(
        Angle target,
        Angle actual,
        AngularVelocity velocity) const;

private:
    double position_gain_;
    double velocity_gain_;
    AngularAcceleration max_acceleration_;
};
