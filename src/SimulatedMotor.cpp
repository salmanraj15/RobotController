#include "SimulatedMotor.hpp"

void SimulatedMotor::setAcceleration(AngularAcceleration acceleration)
{
    acceleration_ = acceleration;
}

AngularAcceleration SimulatedMotor::acceleration() const
{
    return acceleration_;
}