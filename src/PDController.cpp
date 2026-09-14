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
    Angle target,
    Angle actual,
    AngularVelocity velocity) const
{
    const double position_error =
        target.degrees() - actual.degrees();

    const double acceleration =
        position_gain_ * position_error - velocity_gain_ * velocity.degreesPerSecond();

    const double max_acceleration =
        max_acceleration_.degreesPerSecondSquared();

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
