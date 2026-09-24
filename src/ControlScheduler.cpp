#include "ControlScheduler.hpp"

#include <thread>

ControlScheduler::ControlScheduler(
    std::chrono::milliseconds period)
    : period_{period},
      next_cycle_{Clock::now()}
{
}

void ControlScheduler::reset()
{
    next_cycle_ = Clock::now();
}

ControlScheduler::TimePoint
ControlScheduler::waitForNextCycle()
{
    // Wait until the next scheduled cycle.
    std::this_thread::sleep_until(next_cycle_);

    const auto scheduled_start = next_cycle_;

    // Keep the schedule based on the original timeline.
    next_cycle_ += period_;

    return scheduled_start;
}

bool ControlScheduler::hasBacklog(
    TimePoint actual_start,
    TimePoint scheduled_start) const noexcept
{
    return actual_start >= scheduled_start + period_;
}

std::chrono::duration<double, std::milli>
ControlScheduler::schedulingDelay(
    TimePoint actual_start,
    TimePoint scheduled_start) const noexcept
{
    const auto delay =
        actual_start - scheduled_start;

    return std::chrono::duration<double, std::milli>(delay);
}

std::chrono::duration<double, std::milli>
ControlScheduler::backlog(
    TimePoint actual_start,
    TimePoint scheduled_start) const noexcept
{
    if (!hasBacklog(actual_start, scheduled_start))
        return std::chrono::duration<double, std::milli>{0.0};

    const auto delay =
        actual_start - scheduled_start - period_;

    return std::chrono::duration<double, std::milli>(delay);
}
