#pragma once

#include <chrono>

class IControlScheduler
{
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    virtual ~IControlScheduler() = default;

    virtual void reset() = 0;

    virtual TimePoint waitForNextCycle() = 0;

    virtual bool hasBacklog(
        TimePoint actual_start,
        TimePoint scheduled_start) const noexcept = 0;

    virtual std::chrono::duration<double, std::milli>
    schedulingDelay(
        TimePoint actual_start,
        TimePoint scheduled_start) const noexcept = 0;

    virtual std::chrono::duration<double, std::milli>
    backlog(
        TimePoint actual_start,
        TimePoint scheduled_start) const noexcept = 0;
};