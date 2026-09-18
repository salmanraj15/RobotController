#include "Joint.hpp"

Joint::Joint(
    Angle min_position,
    Angle max_position,
    AngularVelocity max_velocity,
    AngularAcceleration max_acceleration,
    MotorInterface &motor)
    : position_{Angle{0.0}},
      command_{},
      velocity_{AngularVelocity{0.0}},
      torque_{0.0},
      min_position_{min_position},
      max_position_{max_position},
      max_velocity_{max_velocity},
      max_acceleration_{max_acceleration},
      motor_{motor}
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

bool Joint::setAcceleration(
    AngularAcceleration acceleration) noexcept
{
    // Reject commands that exceed the joint's configured limit.
    if (acceleration.degreesPerSecondSquared() >
            max_acceleration_.degreesPerSecondSquared() ||
        acceleration.degreesPerSecondSquared() <
            -max_acceleration_.degreesPerSecondSquared())
    {
        return false;
    }

    motor_.setAcceleration(acceleration);

    return true;
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

AngularVelocity Joint::maxVelocity() const
{
    return max_velocity_;
}

Angle Joint::minPosition() const
{
    return min_position_;
}

Angle Joint::maxPosition() const
{
    return max_position_;
}

AngularAcceleration Joint::motorAcceleration() const
{
    return motor_.acceleration();
}

void Joint::simulate(AngularVelocity velocity, Angle position)
{
    velocity_ = velocity;
    position_ = position;
}

double Joint::torque() const
{
    return torque_;
}

JointState Joint::state() const noexcept
{
    return JointState{
        position_,
        velocity_};
}

JointCommand Joint::command() const noexcept
{
    return command_;
}
