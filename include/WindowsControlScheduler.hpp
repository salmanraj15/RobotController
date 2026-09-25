#pragma once

#include "PortableControlScheduler.hpp"

#ifdef _WIN32
#include <windows.h>
#endif

class WindowsControlScheduler final
    : public PortableControlScheduler
{
public:
    explicit WindowsControlScheduler(
        std::chrono::milliseconds period);

    ~WindowsControlScheduler();

    void onControlThreadStart() noexcept override;
    int currentProcessor() const noexcept override;
    double maxWakeLateness() const noexcept override;

    TimePoint waitForNextCycle() override;

private:
#ifdef _WIN32
    static constexpr DWORD_PTR control_affinity_mask_ = 1;

    static constexpr auto spin_window_ =
        std::chrono::microseconds{100};

    static long long toWindows100ns(
        Clock::duration duration) noexcept;

    HANDLE timer_;

    double max_wake_lateness_{0.0};
#endif
};