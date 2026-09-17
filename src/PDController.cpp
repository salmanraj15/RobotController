#include "PDController.hpp"

PDController::PDController(const PDControllerConfig& config)
    : position_gain_{config.position_gain},
      velocity_gain_{config.velocity_gain},
      max_acceleration_{config.max_acceleration}
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