#include "ControlLoop.hpp"

#include <iostream>
#include <thread>

ControlLoop::ControlLoop(Robot &robot)
    : robot_{robot},
      scheduler_{control_period_},
      previous_cycle_{scheduler_.scheduledStart()}
{
}

std::chrono::duration<double, std::milli> ControlLoop::update()
{
    // Wait for the next scheduled cycle.
    scheduler_.wait();

    const auto cycle_start =
        std::chrono::steady_clock::now();

    const auto scheduled_start =
        scheduler_.scheduledStart();

    const auto cycle_deadline =
        scheduled_start + control_period_;

    CycleTiming timing{
        scheduled_start,
        cycle_start,
        cycle_deadline,
        {}};

    const bool behind_schedule =
    scheduler_.isBehindSchedule(cycle_start);

    // Measure the time between cycle starts.
    if (measured_cycles_ > 0)
    {
        const auto period =
            std::chrono::duration<double, std::milli>(
                cycle_start - previous_cycle_);

        const double period_ms = period.count();

        if (measured_cycles_ == 1 || period_ms < min_period_)
        {
            min_period_ = period_ms;
        }

        if (period_ms > max_period_)
        {
            max_period_ = period_ms;
        }

        total_period_ += period_ms;
    }

    previous_cycle_ = cycle_start;

    // Record cycles that started late.
    if (behind_schedule)
    {
        ++delayed_cycles_;

        const auto backlog =
            cycle_start - scheduled_start;

        const auto backlog_ms =
            std::chrono::duration<double, std::milli>(backlog);

        const double backlog_value = backlog_ms.count();

        if (backlog_value > max_backlog_)
        {
            max_backlog_ = backlog_value;
        }

        if (backlog >= control_period_)
        {
            ++backlog_cycles_;
        }
    }

    // Run the actual control work.
    const auto control_start =
        std::chrono::steady_clock::now();

    robot_.update(dt_);

    const auto control_end =
        std::chrono::steady_clock::now();

    const auto control_time =
        std::chrono::duration<double, std::milli>(
            control_end - control_start);

    const double control_ms =
        control_time.count();

    if (control_ms > max_control_time_)
    {
        max_control_time_ = control_ms;
    }

    // Publish the new state after the control update.
    const auto snapshot_start =
        std::chrono::steady_clock::now();

    state_snapshot_.publish(robot_.state());

    const auto snapshot_end =
        std::chrono::steady_clock::now();

    const auto snapshot_time =
        std::chrono::duration<double, std::milli>(
            snapshot_end - snapshot_start);

    const double snapshot_ms =
        snapshot_time.count();

    if (snapshot_ms > max_snapshot_time_)
    {
        max_snapshot_time_ = snapshot_ms;
    }

    timing.actual_end =
        std::chrono::steady_clock::now();

    const auto scheduling_delay =
        std::chrono::duration<double, std::milli>(
            timing.actual_start - timing.scheduled_start);

    const double scheduling_delay_ms =
        scheduling_delay.count();

    if (scheduling_delay_ms > max_scheduling_delay_)
    {
        max_scheduling_delay_ = scheduling_delay_ms;
    }

    const auto deadline_margin =
        std::chrono::duration<double, std::milli>(
            timing.deadline - timing.actual_end);

    const double deadline_margin_ms =
        deadline_margin.count();

    if (measured_cycles_ == 0 ||
        deadline_margin_ms < min_deadline_margin_)
    {
        min_deadline_margin_ = deadline_margin_ms;
    }

    const auto execution_time =
        std::chrono::duration<double, std::milli>(
            timing.actual_end - timing.actual_start);

    const double execution_ms = execution_time.count();

    if (execution_ms > max_execution_time_)
    {
        max_execution_time_ = execution_ms;
    }

    if (timing.actual_end > timing.deadline)
    {
        ++deadline_misses_;
    }

    ++measured_cycles_;
    ++completed_cycles_;

    // Keep the fixed 1 ms schedule.
    scheduler_.advance();

    return execution_time;
}

void ControlLoop::run(int cycle_count)
{
    thread_ = std::jthread(
        [this, cycle_count](std::stop_token stop_token)
        {
            for (int i = 0;
                 i < cycle_count;
                 ++i)
            {
                if (stop_token.stop_requested())
                {
                    break;
                }

                update();
            }
        });
}

void ControlLoop::printTimingStatistics() const
{
    if (measured_cycles_ < 2)
    {
        return;
    }

    const double average_period =
        total_period_ /
        static_cast<double>(measured_cycles_ - 1);

    std::cout << "\nMinimum cycle period: "
              << min_period_
              << " ms\n";

    std::cout << "Maximum cycle period: "
              << max_period_
              << " ms\n";

    std::cout << "Average cycle period: "
              << average_period
              << " ms\n";

    std::cout << "Maximum execution time: "
              << max_execution_time_
              << " ms\n";

    std::cout << "Maximum control time: "
              << max_control_time_
              << " ms\n";

    std::cout << "Maximum snapshot time: "
              << max_snapshot_time_
              << " ms\n";

    std::cout << "Delayed cycles: "
              << delayed_cycles_
              << '\n';

    std::cout << "Deadline misses: "
              << deadline_misses_
              << '\n';

    std::cout << "Maximum scheduling delay: "
              << max_scheduling_delay_
              << " ms\n";

    std::cout << "Maximum schedule backlog: "
              << max_backlog_
              << " ms\n";

    std::cout << "Cycles with >= 1 ms backlog: "
              << backlog_cycles_
              << '\n';

    std::cout << "Minimum deadline margin: "
              << min_deadline_margin_
              << " ms\n";
}

RobotState ControlLoop::state() const noexcept
{
    return state_snapshot_.read();
}

void ControlLoop::printSnapshot(const RobotState &state) const
{
    std::cout << "Snapshot joint 1: "
              << state[0].position.degrees()
              << " degrees\n";
}

int ControlLoop::completedCycles() const noexcept
{
    return completed_cycles_.load();
}

