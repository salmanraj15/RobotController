#pragma once

#include "IControlScheduler.hpp"

class ControlScheduler final : public IControlScheduler
{
public:
    explicit ControlScheduler(
        std::chrono::milliseconds period);

    void reset() override;

    TimePoint waitForNextCycle() override;

    bool hasBacklog(
        TimePoint actual_start,
        TimePoint scheduled_start) const noexcept override;

    std::chrono::duration<double, std::milli>
    schedulingDelay(
        TimePoint actual_start,
        TimePoint scheduled_start) const noexcept override;

    std::chrono::duration<double, std::milli>
    backlog(
        TimePoint actual_start,
        TimePoint scheduled_start) const noexcept override;

private:
    std::chrono::milliseconds period_;
    TimePoint next_cycle_;
};