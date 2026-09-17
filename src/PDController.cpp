#include "PDController.hpp"

PDController::PDController(
    double position_gain,
    double velocity_gain,
    AngularAcceleration max_acceleration)
    : position_gain_{position_gain},
      velocity_gain_{velocity_gain},
      max_acceleration_{max_acceleration}
{
}

AngularAcceleration PDController::calculate(
    const JointCommand& command,
    const JointState& actual_state) const
{
    // Calculate how far the joint is from its target position.
    const double position_error =
        command.target_position.degrees() - actual_state.position.degrees();

    // Use position error to drive the joint toward its target
    // and velocity feedback to reduce overshoot.
    const double acceleration =
        position_gain_ * position_error -
        velocity_gain_ * actual_state.velocity.degreesPerSecond();

    const double max_acceleration =
        max_acceleration_.degreesPerSecondSquared();

    // Keep the controller's acceleration command within its configured limit.
    double limited_acceleration = acceleration;

    if (limited_acceleration > max_acceleration)
    {
        limited_acceleration = max_acceleration;
    }

    if (limited_acceleration < -max_acceleration)
    {
        limited_acceleration = -max_acceleration;
    }

    return AngularAcceleration{limited_acceleration};
}