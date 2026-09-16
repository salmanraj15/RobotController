#include "Robot.hpp"

#include <iostream>

int main()
{
    Robot robot;

    const Duration dt{0.001};
    Duration simulation_time{0.0};

    robot.initializeJointPosition(0, Angle{0.0});
    robot.setJointTargetPosition(0, Angle{90.0});

    for (int i = 0; i < 2500; ++i)
    {
        robot.update(dt);

        simulation_time += dt;

        if ((i + 1) % 100 == 0)
        {
            std::cout << "\nTime: "
                      << simulation_time.seconds()
                      << " s\n";

            robot.printState();
        }
    }

    return 0;
}