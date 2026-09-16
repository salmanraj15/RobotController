#pragma once

#include <array>
#include <cstddef>

#include "Joint.hpp"
#include "PDController.hpp"
#include "SafetyLayer.hpp"

class Robot
{
public:
    void printState() const;

    bool initializeJointPosition(std::size_t index, Angle position);
    bool setJointTargetPosition(std::size_t index, Angle target);
    bool setJointAcceleration(
        std::size_t index,
        AngularAcceleration applied_acceleration_);

    void update(Duration dt);

private:
    std::array<Joint, 6>
        joints_{
            Joint{Angle{-180.0}, Angle{180.0}, AngularVelocity{60.0}, AngularAcceleration{30.0}},
            Joint{Angle{-90.0}, Angle{90.0}, AngularVelocity{60.0}, AngularAcceleration{30.0}},
            Joint{Angle{-180.0}, Angle{180.0}, AngularVelocity{60.0}, AngularAcceleration{30.0}},
            Joint{Angle{-180.0}, Angle{180.0}, AngularVelocity{60.0}, AngularAcceleration{30.0}},
            Joint{Angle{-90.0}, Angle{90.0}, AngularVelocity{60.0}, AngularAcceleration{30.0}},
            Joint{Angle{-180.0}, Angle{180.0}, AngularVelocity{60.0}, AngularAcceleration{30.0}}};

    PDController controller_{
        1.0,
        0.5,
        AngularAcceleration{30.0}};
    SafetyLayer safety_layer_;
};
