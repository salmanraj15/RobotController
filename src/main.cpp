#include "ControlLoop.hpp"
#include "Robot.hpp"

#include <chrono>
#include <iostream>

int main()
{
    Robot robot;
    ControlLoop control_loop{robot};

    robot.initializeJointPosition(0, Angle{0.0});
    robot.setJointTargetPosition(0, Angle{90.0});

    constexpr int cycle_count = 2500;

    const auto start = std::chrono::steady_clock::now();

    // Start the control thread.
    control_loop.run(cycle_count);

    const auto end = std::chrono::steady_clock::now();

    const auto elapsed =
        std::chrono::duration<double>(end - start);

    std::cout << "\nReal execution time: "
              << elapsed.count()
              << " s\n";

    std::cout << "Simulated time: "
              << cycle_count * 0.001
              << " s\n";

    return 0;
}