#pragma once

#include "AngularAcceleration.hpp"
#include "JointTypes.hpp"
#include "PDControllerConfig.hpp"

class PDController
{
public:
    // Creates a controller from its configuration.
    PDController(const PDControllerConfig& config);

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