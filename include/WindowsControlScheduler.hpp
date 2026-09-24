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

    TimePoint waitForNextCycle() override;

private:
#ifdef _WIN32
    static long long toWindows100ns(
        Clock::duration duration) noexcept;

    HANDLE timer_;
#endif
};