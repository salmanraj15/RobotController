#include "SafetyLayer.hpp"

bool SafetyLayer::validate(const Joint& joint, AngularAcceleration requested) const
{
    const double limit = joint.maxAcceleration().degreesPerSecondSquared();

    const double value = requested.degreesPerSecondSquared();

    return value <= limit && value >= -limit;
}