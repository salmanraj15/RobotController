#pragma once

#include <span>

#include "Duration.hpp"
#include "Joint.hpp"

// Advances the physical simulation of the robot.
class RobotSimulator
{
public:
    // Creates a simulator operating on the robot's joints.
    explicit RobotSimulator(std::span<Joint> joints);

    // Advances the simulation by the given timestep.
    void update(const Duration& dt);

private:
    // A view of the joints being simulated.
    std::span<Joint> joints_;
};