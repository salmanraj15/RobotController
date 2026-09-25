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

bool SnapshotBuffer::read(RobotState &state) const noexcept
{
    for (int attempt = 0;
         attempt < max_read_attempts_;
         ++attempt)
    {
        const auto before =
            sequence_.load(std::memory_order_acquire);

        if (before & 1U)
        {
            continue;
        }

        RobotState candidate{};

        for (std::size_t i = 0; i < joint_count; ++i)
        {
            candidate[i].position =
                Angle{
                    positions_[i].load(
                        std::memory_order_relaxed)};

            candidate[i].velocity =
                AngularVelocity{
                    velocities_[i].load(
                        std::memory_order_relaxed)};
        }

        const auto after =
            sequence_.load(std::memory_order_acquire);

        if (before == after)
        {
            state = candidate;
            return true;
        }
    }

    return false;
}