#include "LinuxControlScheduler.hpp"

#ifdef __linux__

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <pthread.h>
#include <sched.h>
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
    configureRealtimeScheduling();
}

void LinuxControlScheduler::configureRealtimeScheduling() noexcept
{
    cpu_set_t cpu_set;

    CPU_ZERO(&cpu_set);
    CPU_SET(control_cpu_, &cpu_set);

    // Keep the control thread on one CPU.
    if (sched_setaffinity(
            0,
            sizeof(cpu_set),
            &cpu_set) != 0)
    {
        std::fprintf(
            stderr,
            "Linux scheduler: failed to set CPU affinity: %s\n",
            std::strerror(errno));

        return;
    }

    sched_param parameters{};
    parameters.sched_priority = fifo_priority_;

    // Give the control thread real-time FIFO scheduling.
    const int result =
        pthread_setschedparam(
            pthread_self(),
            SCHED_FIFO,
            &parameters);

    if (result != 0)
    {
        std::fprintf(
            stderr,
            "Linux scheduler: failed to set SCHED_FIFO: %s\n",
            std::strerror(result));
    }
}

int LinuxControlScheduler::currentProcessor() const noexcept
{
    return sched_getcpu();
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