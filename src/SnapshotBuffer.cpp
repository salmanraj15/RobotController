#include "SnapshotBuffer.hpp"

void SnapshotBuffer::publish(const RobotState &state) noexcept
{
    // Mark the snapshot as being updated.
    sequence_.fetch_add(1, std::memory_order_relaxed);

    for (std::size_t i = 0; i < joint_count; ++i)
    {
        positions_[i].store(
            state[i].position.degrees(),
            std::memory_order_relaxed);

        velocities_[i].store(
            state[i].velocity.degreesPerSecond(),
            std::memory_order_relaxed);
    }

    // Publish the completed snapshot.
    sequence_.fetch_add(1, std::memory_order_release);
}

RobotState SnapshotBuffer::read() const noexcept
{
    RobotState state{};

    while (true)
    {
        const auto before =
            sequence_.load(std::memory_order_acquire);

        if (before & 1U)
        {
            continue;
        }

        for (std::size_t i = 0; i < joint_count; ++i)
        {
            state[i].position =
                Angle{
                    positions_[i].load(
                        std::memory_order_relaxed)};

            state[i].velocity =
                AngularVelocity{
                    velocities_[i].load(
                        std::memory_order_relaxed)};
        }

        const auto after =
            sequence_.load(std::memory_order_acquire);

        if (before == after)
        {
            return state;
        }
    }
}