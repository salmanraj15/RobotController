#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include "Duration.hpp"
#include "Robot.hpp"

// Runs the robot control loop at a fixed period.
class ControlLoop
{
public:
    explicit ControlLoop(Robot &robot);

    void run(int cycle_count);

    void printTimingStatistics() const;

    RobotState state() const;

    void printSnapshot(const RobotState &state) const;

    int completedCycles() const noexcept;

private:
    struct CycleTiming
    {
        std::chrono::steady_clock::time_point scheduled_start;
        std::chrono::steady_clock::time_point actual_start;
        std::chrono::steady_clock::time_point deadline;
        std::chrono::steady_clock::time_point actual_end;
    };

    std::chrono::duration<double, std::milli> update();

    std::chrono::steady_clock::time_point
    current_deadline() const noexcept;

    bool isBehindSchedule(
        std::chrono::steady_clock::time_point now) const noexcept;

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

    double max_scheduling_delay_{0.0};
    double min_deadline_margin_{0.0};

    std::atomic<int> completed_cycles_{0};

    int measured_cycles_{0};
    int delayed_cycles_{0};

    double max_backlog_{0.0};
    int backlog_cycles_{0};
    int deadline_misses_{0};
};