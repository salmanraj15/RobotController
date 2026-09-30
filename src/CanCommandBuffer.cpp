#include "CanCommandBuffer.hpp"

void CanCommandBuffer::publish(
    const JointCommand& command) noexcept
{
    // Mark the command as being updated.
    sequence_.fetch_add(1, std::memory_order_relaxed);

    target_position_degrees_.store(
        command.target_position.degrees(),
        std::memory_order_relaxed);

    // Publish the completed command.
    sequence_.fetch_add(1, std::memory_order_release);
}

bool CanCommandBuffer::read(
    JointCommand& command) const noexcept
{
    constexpr int max_attempts = 3;

    for (int attempt = 0;
         attempt < max_attempts;
         ++attempt)
    {
        const auto before =
            sequence_.load(std::memory_order_acquire);

        if (before & 1U)
            continue;

        const double target =
            target_position_degrees_.load(
                std::memory_order_relaxed);

        const auto after =
            sequence_.load(std::memory_order_acquire);

        if (before != after)
            continue;

        command.target_position =
            Angle{target};

        return true;
    }

    return false;
}