
#include <atomic>
#include <thread>

#include <gtest/gtest.h>

#include "robot_controller/synchronization/SnapshotBuffer.hpp"

TEST(SnapshotBufferTests, ReadsPublishedState)
{
    SnapshotBuffer buffer;

    RobotState published{};

    published[0].position = Angle{45.0};
    published[0].velocity = AngularVelocity{12.0};

    buffer.publish(published);

    RobotState result{};

    ASSERT_TRUE(buffer.read(result));

    EXPECT_DOUBLE_EQ(
        result[0].position.degrees(),
        45.0);

    EXPECT_DOUBLE_EQ(
        result[0].velocity.degreesPerSecond(),
        12.0);
}

TEST(SnapshotBufferTests, ReadsInitialStateBeforeFirstPublish)
{
    SnapshotBuffer buffer;
    RobotState result{};

    // A fresh buffer contains a valid, zero-initialized snapshot.
    EXPECT_TRUE(buffer.read(result));

    for (std::size_t i = 0; i < joint_count; ++i)
    {
        EXPECT_DOUBLE_EQ(result[i].position.degrees(), 0.0);
        EXPECT_DOUBLE_EQ(result[i].velocity.degreesPerSecond(), 0.0);
    }
}


TEST(SnapshotBufferTests, ConcurrentReadsRemainConsistent)
{
    SnapshotBuffer buffer;
    std::atomic<bool> start{false};
    std::atomic<bool> failed{false};

    constexpr double initial_value = 1.0;
    constexpr int publication_count = 10000;

    RobotState initial{};

    for (std::size_t i = 0; i < joint_count; ++i)
    {
        initial[i].position = Angle{initial_value};
        initial[i].velocity = AngularVelocity{initial_value};
    }

    buffer.publish(initial);

    std::thread writer([&]
    {
        while (!start.load(std::memory_order_acquire))
        {
            std::this_thread::yield();
        }

        for (int iteration = 0;
             iteration < publication_count;
             ++iteration)
        {
            const double value =
                static_cast<double>(iteration + 2);

            RobotState next{};

            for (std::size_t i = 0; i < joint_count; ++i)
            {
                next[i].position = Angle{value};
                next[i].velocity = AngularVelocity{value};
            }

            buffer.publish(next);
        }
    });

    std::thread reader([&]
    {
        start.store(true, std::memory_order_release);

        for (int iteration = 0;
             iteration < publication_count;
             ++iteration)
        {
            RobotState result{};

            if (!buffer.read(result))
            {
                std::this_thread::yield();
                continue;
            }

            const double expected =
                result[0].position.degrees();

            for (std::size_t i = 0; i < joint_count; ++i)
            {
                if (result[i].position.degrees() != expected ||
                    result[i].velocity.degreesPerSecond() != expected)
                {
                    failed.store(true, std::memory_order_relaxed);
                    return;
                }
            }
        }
    });

    writer.join();
    reader.join();

    EXPECT_FALSE(failed.load(std::memory_order_relaxed));
}
