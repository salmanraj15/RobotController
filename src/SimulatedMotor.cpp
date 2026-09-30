#include "SimulatedMotor.hpp"

void SimulatedMotor::setAcceleration(AngularAcceleration acceleration) noexcept
{
    acceleration_ = acceleration;
}

AngularAcceleration SimulatedMotor::acceleration() const noexcept
{
    return acceleration_;
}
