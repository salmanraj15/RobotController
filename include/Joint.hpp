#pragma once

#include "Angle.hpp"
#include "Duration.hpp"
#include "AngularVelocity.hpp"
#include "AngularAcceleration.hpp"

class Joint
{
public:
    Joint(Angle min_position, Angle max_position, AngularVelocity max_velocity, AngularAcceleration max_acceleration);
    bool initializePosition(Angle position);
    void setTargetPosition(Angle target);
    bool setAcceleration(AngularAcceleration acceleration);
    void update (const Duration& dt);
    

    Angle position() const;
    Angle targetPosition() const;
    AngularVelocity velocity() const;
    AngularAcceleration maxAcceleration() const;
    double torque() const;

private:
    Angle position_;
    Angle target_position_;

    AngularVelocity velocity_;
    AngularAcceleration applied_acceleration_;

    double torque_;

    Angle min_position_;
    Angle max_position_;

    AngularVelocity max_velocity_;
    AngularAcceleration max_acceleration_;
};