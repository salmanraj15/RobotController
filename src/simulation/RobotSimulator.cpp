#include "robot_controller/simulation/RobotSimulator.hpp"

#include <algorithm>
#include <cmath>

RobotSimulator::RobotSimulator(std::span<Joint> joints)
    : joints_{joints}
{
}

void RobotSimulator::update(Duration dt)
{
    // Ignore negative time steps so the simulation never runs backward.
    if (dt.seconds() < 0.0)
    {
        return;
    }

    for (auto &joint : joints_)
    {
        const double initial_position = joint.position().degrees();

        // Read the configured speed limit before integrating the motion.
        const double max_velocity =
            joint.maxVelocity().degreesPerSecond();

        // Recover safely if the starting state is already outside the speed limits.
        // Normal control should prevent this, but the simulator should remain robust.
        const double initial_velocity = std::clamp(
            joint.velocity().degreesPerSecond(),
            -max_velocity,
            max_velocity);

        const double acceleration =
            joint.motorAcceleration().degreesPerSecondSquared();

        const double elapsed_seconds = dt.seconds();

        // Calculate the velocity we'd reach if acceleration acted
        // for the entire time step.
        const double unconstrained_velocity =
            initial_velocity + acceleration * elapsed_seconds;

        // Keep the final velocity within the joint's configured limits.
        const double final_velocity = std::clamp(
            unconstrained_velocity,
            -max_velocity,
            max_velocity);

        // Calculate position while accounting for the velocity limit.
        double displacement = 0.0;

        if (elapsed_seconds > 0.0 && acceleration != 0.0)
        {
            // Determine the velocity limit in the direction of acceleration.
            const double target_velocity =
                acceleration > 0.0 ? max_velocity : -max_velocity;

            // If we're already at the limit and accelerating farther into it,
            // the joint travels at the capped speed for the entire time step.
            const bool already_at_velocity_limit =
                (acceleration > 0.0 &&
                 initial_velocity >= max_velocity) ||
                (acceleration < 0.0 &&
                 initial_velocity <= -max_velocity);

            if (already_at_velocity_limit)
            {
                displacement = target_velocity * elapsed_seconds;
            }
            else
            {
                // Calculate when the joint reaches the velocity limit.
                const double time_to_limit =
                    (target_velocity - initial_velocity) / acceleration;

                const bool reaches_limit_during_step =
                    time_to_limit > 0.0 &&
                    time_to_limit < elapsed_seconds &&
                    ((acceleration > 0.0 &&
                      unconstrained_velocity > max_velocity) ||
                     (acceleration < 0.0 &&
                      unconstrained_velocity < -max_velocity));

                if (reaches_limit_during_step)
                {
                    // Calculate the distance covered while accelerating.
                    const double accelerating_distance =
                        initial_velocity * time_to_limit +
                        0.5 * acceleration *
                            time_to_limit * time_to_limit;

                    // Calculate the remaining distance at constant speed.
                    const double remaining_time =
                        elapsed_seconds - time_to_limit;

                    const double constant_speed_distance =
                        target_velocity * remaining_time;

                    displacement =
                        accelerating_distance + constant_speed_distance;
                }
                else
                {
                    // The velocity limit wasn't reached, so acceleration
                    // acts throughout the time step.
                    displacement =
                        initial_velocity * elapsed_seconds +
                        0.5 * acceleration *
                            elapsed_seconds * elapsed_seconds;
                }
            }
        }
        else
        {
            // With no acceleration, use the velocity after applying the speed limit.
            // This keeps displacement consistent with the velocity saved in the state.
            displacement = final_velocity * elapsed_seconds;
        }

        double new_position = initial_position + displacement;

        // Stop at the lower position limit and clear the motor command.
        if (new_position <= joint.minPosition().degrees())
        {
            joint.simulate(
                AngularVelocity{0.0},
                joint.minPosition());

            joint.setAcceleration(AngularAcceleration{0.0});
            continue;
        }

        // Stop at the upper position limit and clear the motor command.
        if (new_position >= joint.maxPosition().degrees())
        {
            joint.simulate(
                AngularVelocity{0.0},
                joint.maxPosition());

            joint.setAcceleration(AngularAcceleration{0.0});
            continue;
        }

        // Save the updated physical state for this joint.
        joint.simulate(
            AngularVelocity{final_velocity},
            Angle{new_position});
    }
}
