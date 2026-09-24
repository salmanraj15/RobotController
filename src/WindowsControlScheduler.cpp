#include "WindowsControlScheduler.hpp"
#include <stdexcept>

WindowsControlScheduler::WindowsControlScheduler(
    std::chrono::milliseconds period)
    : PortableControlScheduler{period},
      timer_{CreateWaitableTimer(
          nullptr,
          FALSE,
          nullptr)}
{
    if (timer_ == nullptr)
    {
        throw std::runtime_error(
            "Failed to create Windows waitable timer");
    }
}

WindowsControlScheduler::~WindowsControlScheduler()
{
    CloseHandle(timer_);
}

WindowsControlScheduler::TimePoint
WindowsControlScheduler::waitForNextCycle()
{
    const auto scheduled_start =
        scheduledCycle();

    const auto now =
        Clock::now();

    if (scheduled_start > now)
    {
        const auto remaining =
            scheduled_start - now;

        LARGE_INTEGER due_time{};
        due_time.QuadPart =
            -toWindows100ns(remaining);

        if (!SetWaitableTimer(
                timer_,
                &due_time,
                0,
                nullptr,
                nullptr,
                FALSE))
        {
            throw std::runtime_error(
                "Failed to set Windows waitable timer");
        }

        const DWORD result =
            WaitForSingleObject(
                timer_,
                INFINITE);

        if (result != WAIT_OBJECT_0)
        {
            throw std::runtime_error(
                "Windows waitable timer failed");
        }
    }

    advanceSchedule();

    return scheduled_start;
}

long long
WindowsControlScheduler::toWindows100ns(
    Clock::duration duration) noexcept
{
    using WindowsDuration =
        std::chrono::duration<
            long long,
            std::ratio<1, 10'000'000>>;

    return std::chrono::duration_cast<
        WindowsDuration>(duration).count();
}
