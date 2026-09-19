#include "ControlLoop.hpp"

#include <iostream>
#include <thread>

ControlLoop::ControlLoop(Robot &robot)
    : robot_{robot},
      next_cycle_{std::chrono::steady_clock::now()},
      previous_cycle_{next_cycle_}
{
}

std::chrono::duration<double, std::milli> ControlLoop::update()
{
    // Wait until the next scheduled cycle.
    std::this_thread::sleep_until(next_cycle_);

    const auto cycle_start = std::chrono::steady_clock::now();

    const auto cycle_deadline = next_cycle_ + control_period_;

    // Check whether this cycle started later than scheduled.
    if (cycle_start > next_cycle_)
    {
        const auto backlog =
            std::chrono::duration<double, std::milli>(
                cycle_start - next_cycle_);

        const double backlog_ms = backlog.count();

        if (backlog_ms > max_backlog_)
        {
            max_backlog_ = backlog_ms;
        }
    }

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

    // Record cycles that started behind schedule.
    if (cycle_start > next_cycle_)
    {
        ++delayed_cycles_;

        const auto backlog = cycle_start - next_cycle_;

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

        if (backlog_value > max_jitter_)
        {
            max_jitter_ = backlog_value;
        }
    }

    // Publish one complete snapshot.
    {
        std::lock_guard lock{state_mutex_};
        state_ = robot_.state();
    }

    // Run the actual control work.
    robot_.update(dt_);

    const auto cycle_end = std::chrono::steady_clock::now();

    const auto execution_time =
        std::chrono::duration<double, std::milli>(
            cycle_end - cycle_start);

    const double execution_ms = execution_time.count();

    if (execution_ms > max_execution_time_)
    {
        max_execution_time_ = execution_ms;
    }

    if (cycle_end > cycle_deadline)
    {
        ++deadline_misses_;
    }

    ++measured_cycles_;
    ++completed_cycles_;

    // Keep the schedule based on the original cycle times.
    next_cycle_ += control_period_;

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

    std::cout << "Delayed cycles: "
              << delayed_cycles_
              << '\n';

    std::cout << "Deadline misses: "
              << deadline_misses_
              << '\n';
              
    std::cout << "Maximum jitter: "
              << max_jitter_
              << " ms\n";

    std::cout << "Maximum schedule backlog: "
              << max_backlog_
              << " ms\n";

    std::cout << "Cycles with >= 1 ms backlog: "
              << backlog_cycles_
              << '\n';
}

RobotState ControlLoop::state() const
{
    std::lock_guard lock{state_mutex_};
    return state_;
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