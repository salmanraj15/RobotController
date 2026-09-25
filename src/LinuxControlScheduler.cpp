#include "LinuxControlScheduler.hpp"

#ifdef __linux__

#include <cerrno>
#include <stdexcept>

LinuxControlScheduler::LinuxControlScheduler(
    std::chrono::milliseconds period)
    : PortableControlScheduler{period},
      chrono_start_time_{Clock::now()}
{
    // Capture the Linux monotonic clock at the same starting point.
    if (clock_gettime(
            CLOCK_MONOTONIC,
            &linux_start_time_) != 0)
    {
        throw std::runtime_error(
            "Failed to read CLOCK_MONOTONIC");
    }
}

void LinuxControlScheduler::onControlThreadStart() noexcept
{
    // Linux-specific thread setup will be added here.
}

int LinuxControlScheduler::currentProcessor() const noexcept
{
    return -1;
}

double LinuxControlScheduler::maxWakeLateness() const noexcept
{
    return 0.0;
}

LinuxControlScheduler::TimePoint
LinuxControlScheduler::waitForNextCycle()
{
    const auto scheduled_start =
        scheduledCycle();

    const auto elapsed =
        scheduled_start - chrono_start_time_;

    // Convert the scheduled time to Linux's clock domain.
    const auto target =
        addDuration(
            linux_start_time_,
            elapsed);

    while (true)
    {
        const int result =
            clock_nanosleep(
                CLOCK_MONOTONIC,
                TIMER_ABSTIME,
                &target,
                nullptr);

        if (result == 0)
        {
            break;
        }

        if (result != EINTR)
        {
            throw std::runtime_error(
                "clock_nanosleep failed");
        }
    }

    advanceSchedule();

    return scheduled_start;
}

timespec LinuxControlScheduler::addDuration(
    timespec time,
    Clock::duration duration) noexcept
{
    const auto nanoseconds =
        std::chrono::duration_cast<
            std::chrono::nanoseconds>(
                duration);

    const auto total =
        nanoseconds.count();

    time.tv_sec +=
        static_cast<time_t>(
            total / 1'000'000'000);

    time.tv_nsec +=
        static_cast<long>(
            total % 1'000'000'000);

    if (time.tv_nsec >= 1'000'000'000)
    {
        ++time.tv_sec;
        time.tv_nsec -= 1'000'000'000;
    }

    return time;
}

#endif