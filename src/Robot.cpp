#include "Robot.hpp"

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

bool Robot::setJointPosition(std::size_t index, Angle position)
{
    if (index >= joints_.size())
    {
        return false;
    }

    return joints_[index].setPosition(position);
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
        const AngularAcceleration acceleration =
            controller_.calculate(
                joint.targetPosition(),
                joint.position(),
                joint.velocity());

        joint.setAcceleration(acceleration);
        joint.update(dt);
    }
}
