#include "Joint.hpp"

Joint::Joint(Angle min_position, Angle max_position, AngularVelocity max_velocity, AngularAcceleration max_acceleration)
    : position_{Angle{0.0}},
      target_position_{Angle{0.0}},
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
    target_position_ = target;
}

Angle Joint::targetPosition() const
{
    return target_position_;
}

bool Joint::setAcceleration(AngularAcceleration acceleration)
{
    if (applied_acceleration_.degreesPerSecondSquared() > max_acceleration_.degreesPerSecondSquared() ||
        applied_acceleration_.degreesPerSecondSquared() < -max_acceleration_.degreesPerSecondSquared())
    {
        return false;
    }

    applied_acceleration_ = acceleration;

    return true;
}

void Joint::update(const Duration &dt)
{
    const double new_velocity =
        velocity_.degreesPerSecond() + applied_acceleration_.degreesPerSecondSquared() * dt.seconds();

    const double max_velocity = max_velocity_.degreesPerSecond();

    double limited_velocity = new_velocity;

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