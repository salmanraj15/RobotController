#include "Robot.hpp"

#include <iostream>

int main()
{
Robot robot;

robot.setJointPosition(0, Angle{0.0});
robot.setJointTargetPosition(0, Angle{90.0});

for (int i = 0; i < 100; ++i)
{
    robot.update(Duration{0.1});

    std::cout << "\nTime: "
              << (i + 1) * 0.1
              << " s\n";

    robot.printState();
}

return 0;
}
