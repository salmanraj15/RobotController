#pragma once

#include "Angle.hpp"
#include "Duration.hpp"
#include "AngularVelocity.hpp"
#include "AngularAcceleration.hpp"

class Joint
{
public:
    Joint(Angle min_position, Angle max_position, AngularAcceleration max_acceleration);

    bool setPosition(Angle position);
    bool setAcceleration(AngularAcceleration acceleration);
    void update (Duration dt);

    Angle position() const;
    AngularVelocity velocity() const;
    double torque() const;

private:
    Angle position_;
    AngularVelocity velocity_;
    AngularAcceleration acceleration_;
    double torque_;

    Angle min_position_;
    Angle max_position_;

    AngularAcceleration max_acceleration_;
};