#include "PortableControlScheduler.hpp"

#include <thread>

PortableControlScheduler::PortableControlScheduler(
    std::chrono::milliseconds period)
    : period_{period},
      next_cycle_{Clock::now()}
{
}

void PortableControlScheduler::reset()
{
    next_cycle_ = Clock::now();
}

void PortableControlScheduler::onControlThreadStart() noexcept
{
    // Nothing special on the portable scheduler.
}

int PortableControlScheduler::currentProcessor() const noexcept
{
    return -1;
}

double PortableControlScheduler::maxWakeLateness() const noexcept
{
    return 0.0;
}

PortableControlScheduler::TimePoint
PortableControlScheduler::waitForNextCycle()
{
    // Wait until the next scheduled cycle.
    std::this_thread::sleep_until(next_cycle_);

    const auto scheduled_start = scheduledCycle();

    advanceSchedule();

    return scheduled_start;
}

bool PortableControlScheduler::hasBacklog(
    TimePoint actual_start,
    TimePoint scheduled_start) const noexcept
{
    return actual_start >= scheduled_start + period_;
}

std::chrono::duration<double, std::milli>
PortableControlScheduler::schedulingDelay(
    TimePoint actual_start,
    TimePoint scheduled_start) const noexcept
{
    const auto delay =
        actual_start - scheduled_start;

    return std::chrono::duration<double, std::milli>(delay);
}

std::chrono::duration<double, std::milli>
PortableControlScheduler::backlog(
    TimePoint actual_start,
    TimePoint scheduled_start) const noexcept
{
    if (!hasBacklog(actual_start, scheduled_start))
        return std::chrono::duration<double, std::milli>{0.0};

    const auto delay =
        actual_start - scheduled_start - period_;

    return std::chrono::duration<double, std::milli>(delay);
}

PortableControlScheduler::TimePoint
PortableControlScheduler::scheduledCycle() const noexcept
{
    return next_cycle_;
}

void PortableControlScheduler::advanceSchedule() noexcept
{
    next_cycle_ += period_;
}