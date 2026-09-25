#pragma once

#include "PortableControlScheduler.hpp"

#ifdef __linux__
#include <ctime>
#endif

class LinuxControlScheduler final
    : public PortableControlScheduler
{
public:
    explicit LinuxControlScheduler(
        std::chrono::milliseconds period);

    void onControlThreadStart() noexcept override;
    int currentProcessor() const noexcept override;
    double maxWakeLateness() const noexcept override;

    TimePoint waitForNextCycle() override;

private:
#ifdef __linux__
    // Convert our scheduler duration into Linux monotonic time.
    static timespec addDuration(
        timespec time,
        Clock::duration duration) noexcept;

    // Keep both clocks tied to the same starting point.
    timespec linux_start_time_{};
    TimePoint chrono_start_time_{};
#endif
};