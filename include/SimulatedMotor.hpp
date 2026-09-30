#pragma once

#include "MotorInterface.hpp"

// A simple motor implementation used by the simulator.
class SimulatedMotor : public MotorInterface
{
public:
    // Stores the latest acceleration command sent to the motor.
    void setAcceleration(
        AngularAcceleration acceleration) noexcept override;

    // Returns the latest acceleration command.
    AngularAcceleration acceleration() const noexcept override;

private:
    AngularAcceleration acceleration_{AngularAcceleration{0.0}};
};
