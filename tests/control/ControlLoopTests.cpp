#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <gtest/gtest.h>

#include "robot_controller/control/ControlLoop.hpp"
#include "robot_controller/scheduler/IControlScheduler.hpp"

// The real schedulers wait for clock deadlines. For unit tests, we want
// each cycle to run immediately so tests stay quick and predictable.
class FakeControlScheduler final : public IControlScheduler
{
public:
    using Clock = IControlScheduler::Clock;
    using TimePoint = IControlScheduler::TimePoint;

    void reset() override
    {
        // There is no real schedule to reset in this fake.
    }

    void onControlThreadStart() noexcept override
    {
        // No operating-system scheduling setup is needed in a unit test.
    }

    int currentProcessor() const noexcept override
    {
        // The fake scheduler does not pin the test thread to a CPU.
        return -1;
    }

    double maxWakeLateness() const noexcept override
    {
        return 0.0;
    }

    TimePoint waitForNextCycle() override
    {
        std::unique_lock<std::mutex> lock{gate_mutex_};

        ++cycle_calls_;

        if (cycle_calls_ == blocked_cycle_)
        {
            waiting_at_gate_ = true;
            gate_cv_.notify_all();

            gate_cv_.wait(
                lock,
                [this]
                { return release_gate_; });
        }

        return Clock::now();
    }

    // Pause at a selected scheduler call so lifecycle tests can control timing.
    void blockOnCycle(int cycle_number)
    {
        std::lock_guard<std::mutex> lock{gate_mutex_};

        blocked_cycle_ = cycle_number;
        cycle_calls_ = 0;
        waiting_at_gate_ = false;
        release_gate_ = false;
    }

    bool waitUntilBlocked()
    {
        std::unique_lock<std::mutex> lock{gate_mutex_};

        return gate_cv_.wait_for(
            lock,
            std::chrono::seconds{2},
            [this]
            { return waiting_at_gate_; });
    }

    void releaseBlockedCycle()
    {
        {
            std::lock_guard<std::mutex> lock{gate_mutex_};
            release_gate_ = true;
        }

        gate_cv_.notify_all();
    }

    bool hasBacklog(
        TimePoint /* actual_start */,
        TimePoint /* scheduled_start */) const noexcept override
    {
        return false;
    }

    std::chrono::duration<double, std::milli> schedulingDelay(
        TimePoint /* actual_start */,
        TimePoint /* scheduled_start */) const noexcept override
    {
        return std::chrono::duration<double, std::milli>{0.0};
    }

    std::chrono::duration<double, std::milli> backlog(
        TimePoint /* actual_start */,
        TimePoint /* scheduled_start */) const noexcept override
    {
        return std::chrono::duration<double, std::milli>{0.0};
    }

private:
    std::mutex gate_mutex_;
    std::condition_variable gate_cv_;

    int blocked_cycle_{-1};
    int cycle_calls_{0};

    bool waiting_at_gate_{false};
    bool release_gate_{false};
};

class ControlLoopTests : public ::testing::Test
{
protected:
    // Each test gets fresh buffers, a robot, and a scheduler.
    Robot robot;
    FakeControlScheduler scheduler;
    CanCommandBuffer command_buffer;
    CanStateBuffer state_buffer;
};

// The loop should report every cycle it successfully completes.
TEST_F(ControlLoopTests, CompletesRequestedNumberOfCycles)
{
    ControlLoop loop{
        robot,
        scheduler,
        command_buffer,
        state_buffer};

    constexpr int requested_cycles = 5;

    loop.run(requested_cycles);
    loop.wait();

    EXPECT_EQ(loop.completedCycles(), requested_cycles);
}

// With no command published, the robot should remain at its initial state.
TEST_F(ControlLoopTests, RunsWithoutACanCommand)
{
    ControlLoop loop{
        robot,
        scheduler,
        command_buffer,
        state_buffer};

    loop.run(1);
    loop.wait();

    EXPECT_EQ(loop.completedCycles(), 1);

    const RobotState state = loop.state();

    EXPECT_DOUBLE_EQ(state[0].position.degrees(), 0.0);
    EXPECT_DOUBLE_EQ(
        state[0].velocity.degreesPerSecond(),
        0.0);
}

// A CAN command targeting joint index 0 should make that joint start
// moving toward the requested position during the control cycle.
TEST_F(ControlLoopTests, AppliesPublishedCanCommand)
{
    // Build a command that asks joint 0 to move toward 20 degrees.
    JointCommand command{};
    command.target_position = Angle{20.0};
    command_buffer.publish(command);

    ControlLoop loop{
        robot,
        scheduler,
        command_buffer,
        state_buffer};

    loop.run(1);
    loop.wait();

    EXPECT_EQ(loop.completedCycles(), 1);

    const RobotState state = loop.state();

    // The target is positive, so the PD controller should request
    // positive acceleration from the initial zero position and velocity.
    EXPECT_GT(state[0].velocity.degreesPerSecond(), 0.0);
    EXPECT_GT(state[0].position.degrees(), 0.0);
}

// The CAN state buffer should contain the first joint's latest state
// after the control loop has completed a cycle.
TEST_F(ControlLoopTests, PublishesJointStateToCanStateBuffer)
{
    ControlLoop loop{
        robot,
        scheduler,
        command_buffer,
        state_buffer};

    loop.run(1);
    loop.wait();

    JointState published_state;
    ASSERT_TRUE(state_buffer.read(published_state));

    const RobotState robot_state = loop.state();

    EXPECT_DOUBLE_EQ(
        published_state.position.degrees(),
        robot_state[0].position.degrees());

    EXPECT_DOUBLE_EQ(
        published_state.velocity.degreesPerSecond(),
        robot_state[0].velocity.degreesPerSecond());
}

// The public state() method should expose the latest full robot snapshot.
TEST_F(ControlLoopTests, PublishesFullRobotStateSnapshot)
{
    JointCommand command{};
    command.target_position = Angle{20.0};
    command_buffer.publish(command);

    ControlLoop loop{
        robot,
        scheduler,
        command_buffer,
        state_buffer};

    loop.run(2);
    loop.wait();

    EXPECT_EQ(loop.completedCycles(), 2);

    const RobotState published_state = loop.state();
    const RobotState current_state = robot.state();

    for (std::size_t i = 0; i < joint_count; ++i)
    {
        EXPECT_DOUBLE_EQ(
            published_state[i].position.degrees(),
            current_state[i].position.degrees());

        EXPECT_DOUBLE_EQ(
            published_state[i].velocity.degreesPerSecond(),
            current_state[i].velocity.degreesPerSecond());
    }
}

// Zero cycles should finish without performing any updates.
TEST_F(ControlLoopTests, ZeroCyclesCompletesWithoutRunningUpdates)
{
    ControlLoop loop{
        robot,
        scheduler,
        command_buffer,
        state_buffer};

    loop.run(0);
    loop.wait();

    EXPECT_EQ(loop.completedCycles(), 0);
}

// Negative cycle counts should also perform no updates.
TEST_F(ControlLoopTests, NegativeCyclesCompletesWithoutRunningUpdates)
{
    ControlLoop loop{
        robot,
        scheduler,
        command_buffer,
        state_buffer};

    loop.run(-1);
    loop.wait();

    EXPECT_EQ(loop.completedCycles(), 0);
}

// A second run must be rejected while the first worker is still running.
TEST_F(ControlLoopTests, RejectsSecondRunBeforePreviousWorkerIsJoined)
{
    // Pause the first run on its second scheduler call.
    scheduler.blockOnCycle(2);

    ControlLoop loop{
        robot,
        scheduler,
        command_buffer,
        state_buffer};

    loop.run(5);

    // Release the worker before failing the assertion so it cannot remain
    // blocked if the test times out waiting for the gate.
    const bool reached_gate = scheduler.waitUntilBlocked();

    if (!reached_gate)
    {
        scheduler.releaseBlockedCycle();
        loop.wait();
    }

    ASSERT_TRUE(reached_gate)
        << "The worker never reached the scheduler gate.";

    // The first worker is definitely still running here.
    loop.run(2);

    scheduler.releaseBlockedCycle();
    loop.wait();

    // The second run was rejected; the first completed all five cycles.
    EXPECT_EQ(loop.completedCycles(), 5);
}

TEST_F(ControlLoopTests, RejectsRunAfterCompletionUntilWaitJoinsWorker)
{
    ControlLoop loop{
        robot,
        scheduler,
        command_buffer,
        state_buffer};

    loop.run(2);

    // Wait for both cycles to be counted, without calling wait() yet.
    const auto timeout =
        std::chrono::steady_clock::now() + std::chrono::seconds{2};

    while (loop.completedCycles() < 2 &&
           std::chrono::steady_clock::now() < timeout)
    {
        std::this_thread::yield();
    }

    ASSERT_EQ(loop.completedCycles(), 2);

    // The worker may have finished its cycles, but wait() has not joined it.
    loop.run(3);

    // A rejected run must not reset the counter or replace the first worker.
    EXPECT_EQ(loop.completedCycles(), 2);

    loop.wait();
}

TEST_F(ControlLoopTests, CanRunAgainAfterPreviousWorkerIsJoined)
{
    ControlLoop loop{
        robot,
        scheduler,
        command_buffer,
        state_buffer};

    loop.run(2);
    loop.wait();

    ASSERT_EQ(loop.completedCycles(), 2);

    // Once wait() joins the previous worker, another run is allowed.
    loop.run(3);
    loop.wait();

    EXPECT_EQ(loop.completedCycles(), 5);
}

TEST_F(ControlLoopTests, StopRequestEndsLoopBetweenCycles)
{
    // Hold the second scheduler call so the stop request happens at a known point.
    scheduler.blockOnCycle(2);

    ControlLoop loop{
        robot,
        scheduler,
        command_buffer,
        state_buffer};

    loop.run(10);

    // Do not leave the worker blocked if the gate is not reached in time.
    const bool reached_gate = scheduler.waitUntilBlocked();

    if (!reached_gate)
    {
        scheduler.releaseBlockedCycle();
        loop.wait();
    }

    ASSERT_TRUE(reached_gate)
        << "The worker never reached the second scheduler call.";

    loop.requestStop();
    scheduler.releaseBlockedCycle();
    loop.wait();

    // The already-started cycle completes, then the loop observes the stop.
    EXPECT_EQ(loop.completedCycles(), 2);
}

TEST_F(ControlLoopTests, StopRequestBeforeRunDoesNotPersistIntoNextRun)
{
    ControlLoop loop{
        robot,
        scheduler,
        command_buffer,
        state_buffer};

    loop.requestStop();

    // Starting a new valid run resets the previous stop request.
    loop.run(3);
    loop.wait();

    EXPECT_EQ(loop.completedCycles(), 3);
}

TEST_F(ControlLoopTests, WaitCanBeCalledMoreThanOnce)
{
    ControlLoop loop{
        robot,
        scheduler,
        command_buffer,
        state_buffer};

    loop.run(2);

    loop.wait();
    loop.wait();

    EXPECT_EQ(loop.completedCycles(), 2);
}
