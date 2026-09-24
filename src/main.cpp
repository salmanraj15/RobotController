#include "ControlLoop.hpp"
#include "ControlSchedulerFactory.hpp"
#include "Robot.hpp"

#include <chrono>
#include <iostream>
#include <thread>

int main()
{
    Robot robot;
    auto scheduler =
        createControlScheduler(
            std::chrono::milliseconds{1});

    ControlLoop control_loop{
        robot,
        *scheduler};

    robot.initializeJointPosition(JointIndex{0}, Angle{0.0});
    robot.setJointTargetPosition(JointIndex{0}, Angle{90.0});

    constexpr int cycle_count = 2500;

    const auto start = std::chrono::steady_clock::now();

    // Start the control thread.
    control_loop.run(cycle_count);

    // Wait for the control loop to finish.
    while (control_loop.completedCycles() < cycle_count)
    {
        std::this_thread::sleep_for(
            std::chrono::milliseconds{10});
    }

    const RobotState state = control_loop.state();

    control_loop.printSnapshot(state);
    control_loop.printTimingStatistics();

    std::cout << "Completed cycles: "
              << control_loop.completedCycles()
              << '\n';

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