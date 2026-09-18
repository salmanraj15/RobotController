#include "ControlLoop.hpp"

#include <thread>

ControlLoop::ControlLoop(Robot& robot)
    : robot_{robot},
      next_cycle_{std::chrono::steady_clock::now()}
{
}

std::chrono::duration<double, std::milli> ControlLoop::update()
{
    std::this_thread::sleep_until(next_cycle_);

    const auto cycle_start = std::chrono::steady_clock::now();

    // Run the actual control work.
    robot_.update(dt_);

    const auto cycle_end = std::chrono::steady_clock::now();

    next_cycle_ += std::chrono::milliseconds{1};

    return std::chrono::duration<double, std::milli>(
        cycle_end - cycle_start);
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

    // Wait for the control thread to finish.
    thread_.join();
}