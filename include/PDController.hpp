#pragma once

#include "Angle.hpp"
#include "AngularAcceleration.hpp"
#include "AngularVelocity.hpp"
#include "Joint.hpp"

class PDController
{
public:
    // Creates a controller with position and velocity gains
    // and a maximum acceleration command.
    PDController(
        double position_gain,
        double velocity_gain,
        AngularAcceleration max_acceleration);

    // Calculates an acceleration command from the desired
    // joint command and the joint's current state.
    AngularAcceleration calculate(
        const JointCommand& command,
        const JointState& actual_state) const;

private:
    // Controls how strongly the controller responds to position error.
    double position_gain_;

    // Controls how strongly the controller responds to joint velocity.
    double velocity_gain_;

    // Limits the acceleration command produced by the controller.
    AngularAcceleration max_acceleration_;
};