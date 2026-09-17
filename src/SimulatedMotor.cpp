#include "SimulatedMotor.hpp"

void SimulatedMotor::setAcceleration(AngularAcceleration acceleration)
{
    acceleration_ = acceleration;
}

AngularAcceleration SimulatedMotor::acceleration() const
{
    return acceleration_;
}

void SimulatedMotor::update(const Duration& dt)
{
    // The simulated motor currently only holds the command.
    // Physics will be moved here in a later step.
}