#include "RobotSimulator.hpp"

RobotSimulator::RobotSimulator(std::span<Joint> joints)
    : joints_{joints}
{
}

void RobotSimulator::update(const Duration& dt)
{
    for (auto& joint : joints_)
    {
        const double new_velocity =
            joint.velocity().degreesPerSecond() +
            joint.motorAcceleration().degreesPerSecondSquared() *
                dt.seconds();

        const double max_velocity =
            joint.maxVelocity().degreesPerSecond();

        double limited_velocity = new_velocity;

        if (limited_velocity > max_velocity)
        {
            limited_velocity = max_velocity;
        }

        if (limited_velocity < -max_velocity)
        {
            limited_velocity = -max_velocity;
        }

        const double new_position =
            joint.position().degrees() +
            limited_velocity * dt.seconds();

        if (new_position < joint.minPosition().degrees())
        {
            joint.simulate(
                AngularVelocity{0.0},
                joint.minPosition());

            joint.setAcceleration(AngularAcceleration{0.0});
            continue;
        }

        if (new_position > joint.maxPosition().degrees())
        {
            joint.simulate(
                AngularVelocity{0.0},
                joint.maxPosition());

            joint.setAcceleration(AngularAcceleration{0.0});
            continue;
        }

        joint.simulate(
            AngularVelocity{limited_velocity},
            Angle{new_position});
    }
}