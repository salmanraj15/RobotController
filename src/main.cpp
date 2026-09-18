#include "Robot.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <limits>

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
    auto previous_cycle = start;

    double min_period = std::numeric_limits<double>::max();
    double max_period = 0.0;
    double total_period = 0.0;
    int missed_deadlines = 0;
    double max_execution_time = 0.0;

    for (int i = 0; i < 2500; ++i)
    {
        std::this_thread::sleep_until(next_cycle);

        const auto cycle_start =
            std::chrono::steady_clock::now();

        if (i > 0)
        {
            const auto cycle_period =
                std::chrono::duration<double, std::milli>(
                    cycle_start - previous_cycle);

            const double period = cycle_period.count();

            if (period < min_period)
            {
                min_period = period;
            }

            if (period > max_period)
            {
                max_period = period;
            }

            total_period += period;
            if (period > 1.0)
            {
                ++missed_deadlines;
            }
        }

        previous_cycle = cycle_start;

        robot.update(dt);
        const auto cycle_end =
            std::chrono::steady_clock::now(); /*  */

        const auto execution_time =
            std::chrono::duration<double, std::milli>(
                cycle_end - cycle_start);

        const double execution_ms = execution_time.count();
        if (execution_ms > max_execution_time)
        {
            max_execution_time = execution_ms;
        }
        simulation_time += dt;

        next_cycle += std::chrono::milliseconds{1};
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

    const double average_period =
        total_period / 2499.0;

    std::cout << "Minimum cycle period: "
              << min_period
              << " ms\n";

    std::cout << "Maximum cycle period: "
              << max_period
              << " ms\n";

    std::cout << "Average cycle period: "
              << average_period
              << " ms\n";

    std::cout << "Missed deadlines: "
              << missed_deadlines
              << '\n';

    std::cout << "Maximum execution time: "
              << max_execution_time
              << " ms\n";
    return 0;
}