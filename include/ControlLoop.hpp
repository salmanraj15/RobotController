#pragma once

#include <chrono>
#include <thread>

#include "Duration.hpp"
#include "Robot.hpp"

// Runs the robot control loop at a fixed period.
class ControlLoop
{
public:
    // Creates a control loop for the given robot.
    explicit ControlLoop(Robot& robot);

    // Runs the requested number of control cycles.
    void run(int cycle_count);

    // Returns the measured timing statistics.
    void printTimingStatistics() const;

private:
    // Runs one scheduled control cycle.
    std::chrono::duration<double, std::milli> update();

    Robot& robot_;
    Duration dt_{0.001};

    std::chrono::steady_clock::time_point next_cycle_;
    std::jthread thread_;

    double min_period_{0.0};
    double max_period_{0.0};
    double total_period_{0.0};
    int missed_deadlines_{0};
    double max_execution_time_{0.0};
};