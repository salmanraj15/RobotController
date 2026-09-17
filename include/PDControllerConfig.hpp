#pragma once

#include "AngularAcceleration.hpp"

// Groups the parameters used to configure a PD controller.
struct PDControllerConfig
{
    // Controls how strongly the controller responds to position error.
    double position_gain;

    // Controls how strongly the controller responds to joint velocity.
    double velocity_gain;

    // Limits the acceleration command produced by the controller.
    AngularAcceleration max_acceleration;
};