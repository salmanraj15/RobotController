#include "SafetyLayer.hpp"

bool SafetyLayer::validate(
    const Joint& joint,
    AngularAcceleration requested) const noexcept
{
    // Get the maximum acceleration allowed by the joint.
    const double limit =
        joint.maxAcceleration().degreesPerSecondSquared();

    // Get the acceleration requested by the controller.
    const double value =
        requested.degreesPerSecondSquared();

    // Accept the request only if it stays within the joint's limits.
    return value <= limit && value >= -limit;
}