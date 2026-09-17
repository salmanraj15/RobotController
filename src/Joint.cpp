#include "Joint.hpp"

Joint::Joint(
    Angle min_position,
    Angle max_position,
    AngularVelocity max_velocity,
    AngularAcceleration max_acceleration)
    : position_{Angle{0.0}},
      command_{JointCommand{Angle{0.0}}},
      velocity_{AngularVelocity{0.0}},
      applied_acceleration_{AngularAcceleration{0.0}},
      torque_{0.0},
      min_position_{min_position},
      max_position_{max_position},
      max_velocity_{max_velocity},
      max_acceleration_{max_acceleration}
{
}

bool Joint::initializePosition(Angle position)
{
    if (position.degrees() < min_position_.degrees() ||
        position.degrees() > max_position_.degrees())
    {
        return false;
    }

    position_ = position;
    return true;
}

void Joint::setTargetPosition(Angle target)
{
    command_.target_position = target;
}

Angle Joint::targetPosition() const
{
    return command_.target_position;
}

bool Joint::setAcceleration(AngularAcceleration acceleration)
{
    // Reject acceleration commands that exceed the joint's configured limit.
    if (acceleration.degreesPerSecondSquared() > max_acceleration_.degreesPerSecondSquared() ||
        acceleration.degreesPerSecondSquared() < -max_acceleration_.degreesPerSecondSquared())
    {
        return false;
    }

    applied_acceleration_ = acceleration;

    return true;
}

void Joint::update(const Duration& dt)
{
    // Apply acceleration first, then use the new velocity to update position.
    const double new_velocity =
        velocity_.degreesPerSecond() +
        applied_acceleration_.degreesPerSecondSquared() * dt.seconds();

    const double max_velocity = max_velocity_.degreesPerSecond();

    double limited_velocity = new_velocity;

    // Keep the simulated velocity within the joint's configured limit.
    if (limited_velocity > max_velocity)
    {
        limited_velocity = max_velocity;
    }

    if (limited_velocity < -max_velocity)
    {
        limited_velocity = -max_velocity;
    }

    velocity_ = AngularVelocity{limited_velocity};

    const double new_position =
        position_.degrees() +
        velocity_.degreesPerSecond() * dt.seconds();

    // Stop the joint if the simulated position reaches a physical limit.
    if (new_position < min_position_.degrees())
    {
        position_ = min_position_;
        velocity_ = AngularVelocity{0.0};
        applied_acceleration_ = AngularAcceleration{0.0};
        return;
    }

    if (new_position > max_position_.degrees())
    {
        position_ = max_position_;
        velocity_ = AngularVelocity{0.0};
        applied_acceleration_ = AngularAcceleration{0.0};
        return;
    }

    position_ = Angle{new_position};
}

Angle Joint::position() const
{
    return position_;
}

AngularVelocity Joint::velocity() const
{
    return velocity_;
}

AngularAcceleration Joint::maxAcceleration() const
{
    return max_acceleration_;
}

double Joint::torque() const
{
    return torque_;
}

JointState Joint::state() const
{
    return JointState{
        position_,
        velocity_};
}

JointCommand Joint::command() const
{
    return command_;
}