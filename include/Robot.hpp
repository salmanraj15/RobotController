#pragma once

#include <array>
#include <cstddef>

#include "Joint.hpp"

class Robot
{
public:
    void printState() const;

    bool setJointPosition(std::size_t index, Angle position);
    bool setJointAcceleration(std::size_t index, AngularAcceleration acceleration);
    void update(Duration dt);

private:
    std::array<Joint, 6> joints_{
        Joint{Angle{-180.0}, Angle{180.0}, AngularAcceleration{30.0}},
        Joint{Angle{-90.0},  Angle{90.0},  AngularAcceleration{30.0}},
        Joint{Angle{-180.0}, Angle{180.0}, AngularAcceleration{30.0}},
        Joint{Angle{-180.0}, Angle{180.0}, AngularAcceleration{30.0}},
        Joint{Angle{-90.0},  Angle{90.0},  AngularAcceleration{30.0}},
        Joint{Angle{-180.0}, Angle{180.0}, AngularAcceleration{30.0}}
    };
};