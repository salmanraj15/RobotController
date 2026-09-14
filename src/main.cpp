#include <iostream>

#include "Robot.hpp"

int main()
{
    std::cout << "Robot Controller starting...\n\n";

   Robot robot;

    robot.setJointPosition(0, Angle{0.0});

    if (robot.setJointAcceleration(0, AngularAcceleration{30.0}))
    {
        std::cout << "Acceleration command accepted.\n";
    }
    else
    {
        std::cout << "Acceleration command rejected.\n";
    }
    if (robot.setJointAcceleration(0, AngularAcceleration{100.0}))
    {
        std::cout << "100 deg/s^2 command accepted.\n";
    }
    else
    {
        std::cout << "100 deg/s^2 command rejected.\n";
    }


    
    for (int i = 0; i < 10; ++i)
    {
        robot.update(Duration{0.1});

        std::cout << "\nTime: " << (i + 1) * 0.1 << " s\n";
        robot.printState();
    }
/*
     std::cout << "Moving Joint 1 to 45 degrees...\n";

    if (robot.setJointPosition(0, Angle{45.0}))
    {
        std::cout << "Movement accepted.\n";
    }
    else
    {
        std::cout << "Movement rejected.\n";
    }

    std::cout << "\nMoving Joint 2 to 120 degrees...\n";

    if (robot.setJointPosition(1, Angle{120.0}))
    {
        std::cout << "Movement accepted.\n";
    }
    else
    {
        std::cout << "Movement rejected.\n";
    }

    std::cout << "\nFinal robot state:\n\n";

    robot.printState(); */

    return 0;
}