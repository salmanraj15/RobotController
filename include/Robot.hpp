#pragma once

#include <array>
#include <cstddef>

#include "Joint.hpp"
#include "PDController.hpp"
#include "SafetyLayer.hpp"

class Robot
{
public:
    // Prints the current state of all joints.
    void printState() const;

    // Initializes a joint at the given position.
    bool initializeJointPosition(std::size_t index, Angle position);

    // Sets the target position for a joint.
    bool setJointTargetPosition(std::size_t index, Angle target);

    // Applies an acceleration command directly to a joint.
    bool setJointAcceleration(
        std::size_t index,
        AngularAcceleration acceleration);

    // Advances the simulation and updates all joints.
    void update(Duration dt);

private:
    // The six joints that make up the robot.
    std::array<Joint, 6>
        joints_{
            Joint{Angle{-180.0}, Angle{180.0}, AngularVelocity{60.0}, AngularAcceleration{30.0}},
            Joint{Angle{-90.0}, Angle{90.0}, AngularVelocity{60.0}, AngularAcceleration{30.0}},
            Joint{Angle{-180.0}, Angle{180.0}, AngularVelocity{60.0}, AngularAcceleration{30.0}},
            Joint{Angle{-180.0}, Angle{180.0}, AngularVelocity{60.0}, AngularAcceleration{30.0}},
            Joint{Angle{-90.0}, Angle{90.0}, AngularVelocity{60.0}, AngularAcceleration{30.0}},
            Joint{Angle{-180.0}, Angle{180.0}, AngularVelocity{60.0}, AngularAcceleration{30.0}}};

    // Calculates acceleration commands for the joints.
    PDController controller_{
        1.0,
        0.5,
        AngularAcceleration{30.0}};

    // Checks controller commands against the joint's safety limits.
    SafetyLayer safety_layer_;
};