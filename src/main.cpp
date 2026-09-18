#include "Robot.hpp"

#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    Robot robot;

    // The control loop advances the simulation by 1 ms each cycle.
    constexpr double control_period_seconds = 0.001;
    const Duration dt{control_period_seconds};
    Duration simulation_time{0.0};

    robot.initializeJointPosition(0, Angle{0.0});
    robot.setJointTargetPosition(0, Angle{90.0});

    const auto start = std::chrono::steady_clock::now();
    auto next_cycle = start;

    for (int i = 0; i < 2500; ++i)
    {
        robot.update(dt);

        simulation_time += dt;

        next_cycle += std::chrono::milliseconds{1};

        std::this_thread::sleep_until(next_cycle);
    }

    const auto end = std::chrono::steady_clock::now();

    const auto elapsed =
        std::chrono::duration<double>(end - start);

    std::cout << "\nReal execution time: "
              << elapsed.count()
              << " s\n";

    std::cout << "Simulated time: "
              << simulation_time.seconds()
              << " s\n";

    return 0;
}