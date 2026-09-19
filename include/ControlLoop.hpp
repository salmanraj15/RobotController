#pragma once

#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>

#include "Duration.hpp"
#include "Robot.hpp"

// Runs the robot control loop at a fixed period.
class ControlLoop
{
public:
    // Creates a control loop for the given robot.
    explicit ControlLoop(Robot &robot);

    // Runs the requested number of control cycles.
    void run(int cycle_count);

    // Returns the measured timing statistics.
    void printTimingStatistics() const;

    // Returns the latest state snapshot.
    RobotState state() const;

    // Prints a supplied state snapshot.
    void printSnapshot(const RobotState &state) const;

    // Returns how many cycles have completed.
    int completedCycles() const noexcept;

private:
    // Runs one scheduled control cycle.
    std::chrono::duration<double, std::milli> update();

    Robot &robot_;

    mutable std::mutex state_mutex_;
    RobotState state_{};

    // The control loop runs every 1 ms.
    static constexpr auto control_period_ =
        std::chrono::milliseconds{1};

    Duration dt_{0.001};

    std::chrono::steady_clock::time_point next_cycle_;
    std::chrono::steady_clock::time_point previous_cycle_;

    std::jthread thread_;

    double min_period_{0.0};
    double max_period_{0.0};
    double total_period_{0.0};
    double max_execution_time_{0.0};

    std::atomic<int> completed_cycles_{0};
    int measured_cycles_{0};
    int delayed_cycles_{0};

    double max_jitter_{0.0};
    double max_backlog_{0.0};
    int backlog_cycles_{0};
};