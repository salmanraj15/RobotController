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

void ControlScheduler::wait()
{
    // Wait until the next scheduled cycle.
    std::this_thread::sleep_until(next_cycle_);
}

void ControlScheduler::advance()
{
    // Keep the schedule based on the original timeline.
    next_cycle_ += period_;
}

bool ControlScheduler::isBehindSchedule(
    TimePoint now) const noexcept
{
    return now >= next_cycle_ + period_;
}

ControlScheduler::TimePoint
ControlScheduler::scheduledStart() const noexcept
{
    return next_cycle_;
}