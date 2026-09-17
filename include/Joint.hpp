#pragma once

#include "Angle.hpp"
#include "Duration.hpp"
#include "AngularVelocity.hpp"
#include "AngularAcceleration.hpp"

// Describes what the joint is currently doing.
struct JointState
{
    Angle position;
    AngularVelocity velocity;
};

// Describes what we want the joint to do.
struct JointCommand
{
    Angle target_position;
};

class Joint
{
public:
    Joint(
        Angle min_position,
        Angle max_position,
        AngularVelocity max_velocity,
        AngularAcceleration max_acceleration);

    bool initializePosition(Angle position);
    void setTargetPosition(Angle target);
    bool setAcceleration(AngularAcceleration acceleration);
    void update(const Duration& dt);

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

    double torque() const;

private:
    // The joint's current simulated position.
    Angle position_;

    // The command describing where the joint should move.
    JointCommand command_;

    // The joint's current simulated velocity.
    AngularVelocity velocity_;

    // The acceleration currently being applied to the simulation.
    AngularAcceleration applied_acceleration_;

    double torque_;

    Angle min_position_;
    Angle max_position_;

    AngularVelocity max_velocity_;
    AngularAcceleration max_acceleration_;
};