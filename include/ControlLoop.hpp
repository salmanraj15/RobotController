#pragma once

#include <atomic>
#include <chrono>
#include <thread>

#include "Duration.hpp"
#include "Robot.hpp"
#include "SnapshotBuffer.hpp"
#include "IControlScheduler.hpp"

// Runs the robot control loop at a fixed period.
class ControlLoop
{
public:
    ControlLoop(
        Robot &robot,
        IControlScheduler &scheduler);

    void run(int cycle_count);

    void printTimingStatistics() const;

    RobotState state() const noexcept;

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

    Robot &robot_;

    SnapshotBuffer state_snapshot_;

    // The control loop runs every 1 ms.
    static constexpr auto control_period_ =
        std::chrono::milliseconds{1};

    Duration dt_{0.001};

    IControlScheduler &scheduler_;
    std::chrono::steady_clock::time_point previous_cycle_;

    std::jthread thread_;

    double min_period_{0.0};
    double max_period_{0.0};
    double total_period_{0.0};
    double max_execution_time_{0.0};
    double max_control_time_{0.0};
    double max_snapshot_time_{0.0};
    double max_inter_cycle_gap_{0.0};

    std::chrono::steady_clock::time_point previous_cycle_end_;

    double max_scheduling_delay_{0.0};
    double min_deadline_margin_{0.0};

    std::atomic<int> completed_cycles_{0};

    int measured_cycles_{0};
    int delayed_cycles_{0};

    double max_backlog_{0.0};
    int backlog_cycles_{0};

    int consecutive_delayed_cycles_{0};
    int max_consecutive_delayed_cycles_{0};

    int deadline_misses_{0};

    int scheduling_misses_{0};
    int execution_misses_{0};
    int combined_misses_{0};

    int delay_under_100us_{0};
    int delay_100us_to_1ms_{0};
    int delay_1ms_to_5ms_{0};
    int delay_5ms_to_10ms_{0};
    int delay_over_10ms_{0};

    int processor_changes_{0};
    int previous_processor_{-1};
};