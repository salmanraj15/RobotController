#pragma once

#include <chrono>

class ControlScheduler
{
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    explicit ControlScheduler(
        std::chrono::milliseconds period);

    void reset();

    TimePoint waitForNextCycle();

    bool hasBacklog(
        TimePoint actual_start,
        TimePoint scheduled_start) const noexcept;

    std::chrono::duration<double, std::milli>
    schedulingDelay(
        TimePoint actual_start,
        TimePoint scheduled_start) const noexcept;

    std::chrono::duration<double, std::milli>
    backlog(
        TimePoint actual_start,
        TimePoint scheduled_start) const noexcept;

private:
    std::chrono::milliseconds period_;
    TimePoint next_cycle_;
};