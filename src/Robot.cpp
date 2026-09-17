#include "Robot.hpp"
#include "SafetyLayer.hpp"

#include <iostream>

void Robot::printState() const
{
    std::cout << "Robot has " << joints_.size() << " joints";

    for (std::size_t i = 0; i < joints_.size(); i++)
    {
        std::cout << "\n\nJoint " << i + 1 << ":\n";

        std::cout << "Position: "
                  << joints_[i].position().degrees()
                  << " degrees";

        std::cout << "\nTarget: "
                  << joints_[i].targetPosition().degrees()
                  << " degrees";

        std::cout << "\nVelocity: "
                  << joints_[i].velocity().degreesPerSecond();

        std::cout << "\nTorque: "
                  << joints_[i].torque();
    }
}

bool Robot::initializeJointPosition(std::size_t index, Angle position)
{
    if (index >= joints_.size())
    {
        return false;
    }

    return joints_[index].initializePosition(position);
}

bool Robot::setJointTargetPosition(std::size_t index, Angle target)
{
    if (index >= joints_.size())
    {
        return false;
    }

    joints_[index].setTargetPosition(target);
    return true;
}

bool Robot::setJointAcceleration(
    std::size_t index,
    AngularAcceleration acceleration)
{
    if (index >= joints_.size())
    {
        return false;
    }

    return joints_[index].setAcceleration(acceleration);
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

        // Advance the physical simulation after all motor commands are prepared.
        simulator_.update(dt);
    }
}