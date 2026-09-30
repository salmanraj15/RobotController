#include "robot_controller/communication/CanStateBuffer.hpp"

void CanStateBuffer::publish(
    const JointState& state) noexcept
{
    // Mark the state as being updated.
    sequence_.fetch_add(1, std::memory_order_relaxed);

    position_degrees_.store(
        state.position.degrees(),
        std::memory_order_relaxed);

    velocity_degrees_per_second_.store(
        state.velocity.degreesPerSecond(),
        std::memory_order_relaxed);

    // Publish the completed state.
    sequence_.fetch_add(1, std::memory_order_release);
}

bool CanStateBuffer::read(
    JointState& state) const noexcept
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

        const double position =
            position_degrees_.load(
                std::memory_order_relaxed);

        const double velocity =
            velocity_degrees_per_second_.load(
                std::memory_order_relaxed);

        const auto after =
            sequence_.load(std::memory_order_acquire);

        if (before != after)
            continue;

        state.position = Angle{position};
        state.velocity =
            AngularVelocity{velocity};

        return true;
    }

    return false;
}