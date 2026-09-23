#pragma once

#include <chrono>

class ControlScheduler
{
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    explicit ControlScheduler(
        std::chrono::milliseconds period);

    void reset();

    void wait();

    void advance();

    bool isBehindSchedule(TimePoint now) const noexcept;

    TimePoint scheduledStart() const noexcept;

private:
    std::chrono::milliseconds period_;
    TimePoint next_cycle_;
};