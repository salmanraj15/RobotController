#pragma once

#include <chrono>
#include <thread>
#include <atomic>
#include <array>

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
    RobotState state() const noexcept;

    // Prints a supplied state snapshot.
    void printSnapshot(const RobotState &state) const;

    // Returns how many cycles have completed.
    int completedCycles() const noexcept;

private:
    // Runs one scheduled control cycle.
    std::chrono::duration<double, std::milli> update();

    Robot &robot_;

    // Holds the two snapshot buffers.
    std::array<RobotState, 2> state_buffers_{};

    // Tracks which snapshot readers should use.
    std::atomic<int> published_state_{0};

    // Tracks whether a reader is using each snapshot buffer.
    mutable std::array<std::atomic<int>, 2> state_readers_{};

    // The control thread writes to this buffer next.
    int write_state_{1};

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