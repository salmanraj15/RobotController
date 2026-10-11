#include "robot_controller/safety/SafetyLayer.hpp"

#include <cmath>

bool SafetyLayer::validate(
    const Joint& joint,
    AngularAcceleration requested) const noexcept
{
    // Read the requested acceleration and the joint's allowed limit.
    const double value =
        requested.degreesPerSecondSquared();

    const double limit =
        joint.maxAcceleration().degreesPerSecondSquared();

    // Reject non-finite values explicitly; they are not valid commands.
    if (!std::isfinite(value) || !std::isfinite(limit))
    {
        return false;
    }

    // Accept only finite accelerations inside the inclusive limits.
    return value <= limit && value >= -limit;
}