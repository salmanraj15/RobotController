#include "Robot.hpp"

#include <iostream>
#include <span>

RobotState Robot::state() const noexcept
{
    RobotState state{};

    for (std::size_t i = 0; i < joints_.size(); ++i)
    {
        state[i] = joints_[i].state();
    }

    return state;
}

void Robot::printState() const
{
    const auto joints = std::span<const Joint>{joints_};

    std::cout << "Robot has " << joints.size() << " joints";

    for (std::size_t i = 0; i < joints.size(); ++i)
    {
        // Print the current state without modifying the joints.
        std::cout << "\n\nJoint " << i + 1 << ":\n";

        std::cout << "Position: "
                  << joints[i].position().degrees()
                  << " degrees";

        std::cout << "\nTarget: "
                  << joints[i].targetPosition().degrees()
                  << " degrees";

        std::cout << "\nVelocity: "
                  << joints[i].velocity().degreesPerSecond();

        std::cout << "\nTorque: "
                  << joints[i].torque();
    }
}

bool Robot::initializeJointPosition(JointIndex index, Angle position)
{
    if (index.value >= joints_.size())
        return false;

    return joints_[index.value].initializePosition(position);
}

bool Robot::setJointTargetPosition(JointIndex index, Angle target)
{
    if (index.value >= joints_.size())
        return false;

    joints_[index.value].setTargetPosition(target);
    return true;
}

bool Robot::setJointAcceleration(JointIndex index, AngularAcceleration acceleration)
{
    if (index.value >= joints_.size())
        return false;

    return joints_[index.value].setAcceleration(acceleration);
}

void Robot::update(Duration dt)
{
    for (auto &joint : joints_)
    {
        // Read the command and actual state before calculating the next control output.
        const JointState actual_state = joint.state();
        const JointCommand command = joint.command();

        // The controller converts the desired and actual state into an acceleration request.
        const AngularAcceleration requested_acceleration =
            controller_.calculate(
                command,
                actual_state);

        // Validate the controller's request before applying it to the joint.
        if (safety_layer_.validate(joint, requested_acceleration))
        {
            joint.setAcceleration(requested_acceleration);
        }
        else
        {
            // Reject an unsafe request by applying zero acceleration instead.
            joint.setAcceleration(AngularAcceleration{0.0});
        }
    }

    // Advance the physical simulation after all motor commands are prepared.
    simulator_.update(dt);
}
