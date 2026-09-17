#pragma once

#include "MotorInterface.hpp"
#include "Duration.hpp"

// A simple motor implementation used by the simulator.
class SimulatedMotor : public MotorInterface
{
public:
    // Stores the latest acceleration command sent to the motor.
    void setAcceleration(AngularAcceleration acceleration) override;

    // Returns the latest acceleration command.
    AngularAcceleration acceleration() const override;

    // Advances the motor simulation by the given timestep.
    void update(const Duration &dt);

private:
    AngularAcceleration acceleration_{AngularAcceleration{0.0}};
};