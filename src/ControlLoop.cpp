#include "ControlLoop.hpp"

#include <iostream>
#include <thread>

ControlLoop::ControlLoop(
    Robot &robot,
    IControlScheduler &scheduler)
    : robot_{robot},
      scheduler_{scheduler},
      previous_cycle_{std::chrono::steady_clock::now()},
      previous_cycle_end_{std::chrono::steady_clock::now()}
{
}

void ControlLoop::updateProcessorStatistics() noexcept
{
    const int processor =
        scheduler_.currentProcessor();

    if (processor >= 0)
    {
        if (previous_processor_ >= 0 &&
            processor != previous_processor_)
        {
            ++processor_changes_;
        }

        previous_processor_ = processor;
    }
}

void ControlLoop::updatePeriodStatistics(
    std::chrono::steady_clock::time_point cycle_start) noexcept
{
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
}

void ControlLoop::updateDelayStatistics(
    bool behind_schedule,
    std::chrono::steady_clock::time_point cycle_start,
    std::chrono::steady_clock::time_point scheduled_start) noexcept
{
    if (behind_schedule)
    {
        ++delayed_cycles_;
        ++consecutive_delayed_cycles_;

        if (consecutive_delayed_cycles_ >
            max_consecutive_delayed_cycles_)
        {
            max_consecutive_delayed_cycles_ =
                consecutive_delayed_cycles_;
        }

        const auto backlog =
            scheduler_.backlog(
                cycle_start,
                scheduled_start);

        const auto backlog_ms =
            std::chrono::duration<double, std::milli>(backlog);

        const double backlog_value =
            backlog_ms.count();

        if (backlog_value > max_backlog_)
        {
            max_backlog_ = backlog_value;
        }

        if (backlog >= control_period_)
        {
            ++backlog_cycles_;
        }
    }
    else
    {
        consecutive_delayed_cycles_ = 0;
    }
}

void ControlLoop::updateControlStatistics(
    std::chrono::steady_clock::time_point control_start,
    std::chrono::steady_clock::time_point control_end) noexcept
{
    const auto control_time =
        std::chrono::duration<double, std::milli>(
            control_end - control_start);

    const double control_ms =
        control_time.count();

    if (control_ms > max_control_time_)
    {
        max_control_time_ = control_ms;
    }
}

void ControlLoop::updateSnapshotStatistics(
    std::chrono::steady_clock::time_point snapshot_start,
    std::chrono::steady_clock::time_point snapshot_end) noexcept
{
    const auto snapshot_time =
        std::chrono::duration<double, std::milli>(
            snapshot_end - snapshot_start);

    const double snapshot_ms =
        snapshot_time.count();

    if (snapshot_ms > max_snapshot_time_)
    {
        max_snapshot_time_ = snapshot_ms;
    }
}

void ControlLoop::updateInterCycleStatistics(
    std::chrono::steady_clock::time_point cycle_start,
    std::chrono::steady_clock::time_point cycle_end) noexcept
{
    if (measured_cycles_ > 0)
    {
        const auto inter_cycle_gap =
            std::chrono::duration<double, std::milli>(
                cycle_start - previous_cycle_end_);

        const double gap_ms =
            inter_cycle_gap.count();

        if (gap_ms > max_inter_cycle_gap_)
        {
            max_inter_cycle_gap_ = gap_ms;
        }
    }

    previous_cycle_end_ = cycle_end;
}

void ControlLoop::updateSchedulingStatistics(
    double scheduling_delay_ms) noexcept
{
    if (scheduling_delay_ms < 0.1)
    {
        ++delay_under_100us_;
    }
    else if (scheduling_delay_ms < 1.0)
    {
        ++delay_100us_to_1ms_;
    }
    else if (scheduling_delay_ms < 5.0)
    {
        ++delay_1ms_to_5ms_;
    }
    else if (scheduling_delay_ms < 10.0)
    {
        ++delay_5ms_to_10ms_;
    }
    else
    {
        ++delay_over_10ms_;
    }

    if (scheduling_delay_ms > max_scheduling_delay_)
    {
        max_scheduling_delay_ = scheduling_delay_ms;
    }
}

void ControlLoop::updateDeadlineStatistics(
    std::chrono::steady_clock::time_point deadline,
    std::chrono::steady_clock::time_point actual_end,
    int measured_cycles) noexcept
{
    const auto deadline_margin =
        std::chrono::duration<double, std::milli>(
            deadline - actual_end);

    const double deadline_margin_ms =
        deadline_margin.count();

    if (measured_cycles == 0 ||
        deadline_margin_ms < min_deadline_margin_)
    {
        min_deadline_margin_ = deadline_margin_ms;
    }
}

std::chrono::duration<double, std::milli>
ControlLoop::updateExecutionStatistics(
    std::chrono::steady_clock::time_point actual_start,
    std::chrono::steady_clock::time_point actual_end) noexcept
{
    const auto execution_time =
        std::chrono::duration<double, std::milli>(
            actual_end - actual_start);

    const double execution_ms =
        execution_time.count();

    if (execution_ms > max_execution_time_)
    {
        max_execution_time_ = execution_ms;
    }

    return execution_time;
}

void ControlLoop::updateMissStatistics(
    std::chrono::steady_clock::time_point actual_end,
    std::chrono::steady_clock::time_point deadline,
    double scheduling_delay_ms,
    double execution_ms) noexcept
{
    if (actual_end > deadline)
    {
        ++deadline_misses_;

        const bool scheduling_overrun =
            scheduling_delay_ms > 0.0;

        const bool execution_overrun =
            execution_ms > control_period_.count();

        if (scheduling_overrun && execution_overrun)
        {
            ++combined_misses_;
        }
        else if (scheduling_overrun)
        {
            ++scheduling_misses_;
        }
        else
        {
            ++execution_misses_;
        }
    }
}

std::chrono::duration<double, std::milli> ControlLoop::update()
{
    // Wait for the next scheduled cycle.
    const auto scheduled_start =
        scheduler_.waitForNextCycle();

    const auto cycle_start =
        std::chrono::steady_clock::now();

    updateProcessorStatistics();

    const auto cycle_deadline =
        scheduled_start + control_period_;

    CycleTiming timing{
        scheduled_start,
        cycle_start,
        cycle_deadline,
        {}};

    const bool behind_schedule =
        scheduler_.hasBacklog(
            cycle_start,
            scheduled_start);

    updatePeriodStatistics(cycle_start);

    updateDelayStatistics(
        behind_schedule,
        cycle_start,
        scheduled_start);

    // Run the actual control work.
    const auto control_start =
        std::chrono::steady_clock::now();

    robot_.update(dt_);

    const auto control_end =
        std::chrono::steady_clock::now();

    updateControlStatistics(
        control_start,
        control_end);

    const auto snapshot_start =
        std::chrono::steady_clock::now();

    state_snapshot_.publish(robot_.state());

    const auto snapshot_end =
        std::chrono::steady_clock::now();

    updateSnapshotStatistics(
        snapshot_start,
        snapshot_end);

    timing.actual_end =
        std::chrono::steady_clock::now();

    updateInterCycleStatistics(
        cycle_start,
        timing.actual_end);

    const auto scheduling_delay =
        scheduler_.schedulingDelay(
            timing.actual_start,
            timing.scheduled_start);

    const double scheduling_delay_ms =
        scheduling_delay.count();

    updateSchedulingStatistics(
        scheduling_delay_ms);

    updateDeadlineStatistics(
        timing.deadline,
        timing.actual_end,
        measured_cycles_);

    const auto execution_time =
        updateExecutionStatistics(
            timing.actual_start,
            timing.actual_end);

    const double execution_ms =
        execution_time.count();

    updateMissStatistics(
        timing.actual_end,
        timing.deadline,
        scheduling_delay_ms,
        execution_ms);

    ++measured_cycles_;
    ++completed_cycles_;

    return execution_time;
}

void ControlLoop::run(int cycle_count)
{
    thread_ = std::jthread(
        [this, cycle_count](std::stop_token stop_token)
        {
            scheduler_.onControlThreadStart();

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

    std::cout << "Maximum inter-cycle gap: "
              << max_inter_cycle_gap_
              << " ms\n";

    std::cout << "Delayed cycles: "
              << delayed_cycles_
              << '\n';

    std::cout << "Deadline misses: "
              << deadline_misses_
              << '\n';

    std::cout << "Scheduling misses: "
              << scheduling_misses_
              << '\n';

    std::cout << "Execution misses: "
              << execution_misses_
              << '\n';

    std::cout << "Combined misses: "
              << combined_misses_
              << '\n';

    std::cout << "Maximum scheduling delay: "
              << max_scheduling_delay_
              << " ms\n";

    std::cout << "Maximum scheduler wake lateness: "
              << scheduler_.maxWakeLateness()
              << " ms\n";

    std::cout << "Scheduling delay < 0.1 ms: "
              << delay_under_100us_
              << '\n';

    std::cout << "Scheduling delay 0.1-1 ms: "
              << delay_100us_to_1ms_
              << '\n';

    std::cout << "Scheduling delay 1-5 ms: "
              << delay_1ms_to_5ms_
              << '\n';

    std::cout << "Scheduling delay 5-10 ms: "
              << delay_5ms_to_10ms_
              << '\n';

    std::cout << "Scheduling delay >= 10 ms: "
              << delay_over_10ms_
              << '\n';

    std::cout << "Maximum consecutive delayed cycles: "
              << max_consecutive_delayed_cycles_
              << '\n';

    std::cout << "Processor changes: "
              << processor_changes_
              << '\n';

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
    RobotState state = last_state_;

    if (state_snapshot_.read(state))
    {
        last_state_ = state;
    }

    return last_state_;
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
