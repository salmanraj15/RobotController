#pragma once

#include <array>

#include "Duration.hpp"
#include "Joint.hpp"

// Advances the physical simulation of the robot.
class RobotSimulator
{
public:
    // Creates a simulator operating on the robot's joints.
    explicit RobotSimulator(std::array<Joint, 6>& joints);

    // Advances the simulation by the given timestep.
    void update(const Duration& dt);

private:
    // The joints whose physical state is being simulated.
    std::array<Joint, 6>& joints_;
};