#pragma once

#include "Angle.hpp"
#include "Duration.hpp"
#include "AngularVelocity.hpp"
#include "AngularAcceleration.hpp"
#include "JointTypes.hpp"
#include "MotorInterface.hpp"

class Joint
{
public:
    Joint(
        Angle min_position,
        Angle max_position,
        AngularVelocity max_velocity,
        AngularAcceleration max_acceleration,
        MotorInterface &motor);

    bool initializePosition(Angle position);
    void setTargetPosition(Angle target);
    bool setAcceleration(AngularAcceleration acceleration);

    // Returns the joint's current position.
    Angle position() const;

    // Returns the position the controller is trying to reach.
    Angle targetPosition() const;

    // Returns the joint's current velocity.
    AngularVelocity velocity() const;

    // Returns the current physical state of the joint.
    JointState state() const;

    // Returns the current command for the joint.
    JointCommand command() const;

    // Returns the maximum acceleration allowed for this joint.
    AngularAcceleration maxAcceleration() const;

    AngularVelocity maxVelocity() const;
    Angle minPosition() const;
    Angle maxPosition() const;

    // Returns the acceleration currently commanded to the motor.
    AngularAcceleration motorAcceleration() const;

    // Updates the joint's simulated physical state.
    void simulate(AngularVelocity velocity, Angle position);

    double torque() const;

private:
    // The joint's current simulated position.
    Angle position_;

    // The command describing where the joint should move.
    JointCommand command_;

    // The joint's current simulated velocity.
    AngularVelocity velocity_;

    double torque_;

    Angle min_position_;
    Angle max_position_;

    AngularVelocity max_velocity_;
    AngularAcceleration max_acceleration_;

    // The motor interface used to send commands to the actuator.
    MotorInterface &motor_;
    ;
};