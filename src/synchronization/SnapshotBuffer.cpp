#include "robot_controller/synchronization/SnapshotBuffer.hpp"

void SnapshotBuffer::publish(const RobotState &state) noexcept
{
    // This buffer assumes a single writer.
    // An odd sequence indicates that a publication is in progress.
    sequence_.fetch_add(1, std::memory_order_seq_cst);

    for (std::size_t i = 0; i < joint_count; ++i)
    {
        positions_[i].store(
            state[i].position.degrees(),
            std::memory_order_seq_cst);

        velocities_[i].store(
            state[i].velocity.degreesPerSecond(),
            std::memory_order_seq_cst);
    }

    // An even sequence indicates that the snapshot is complete.
    sequence_.fetch_add(1, std::memory_order_seq_cst);
}

bool SnapshotBuffer::read(RobotState &state) const noexcept
{
    for (int attempt = 0;
         attempt < max_read_attempts_;
         ++attempt)
    {
        const auto before =
            sequence_.load(std::memory_order_seq_cst);

        if ((before & 1U) != 0U)
        {
            continue;
        }

        RobotState candidate{};

        for (std::size_t i = 0; i < joint_count; ++i)
        {
            candidate[i].position = Angle{
                positions_[i].load(std::memory_order_seq_cst)};

            candidate[i].velocity = AngularVelocity{
                velocities_[i].load(std::memory_order_seq_cst)};
        }

        const auto after =
            sequence_.load(std::memory_order_seq_cst);

        if (before == after)
        {
            state = candidate;
            return true;
        }
    }

    return false;
}
