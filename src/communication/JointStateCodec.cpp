#include "robot_controller/communication/JointStateCodec.hpp"

#include <cstdint>

CanFrame JointStateCodec::encode(
    const JointState& state) noexcept
{
    CanFrame frame;

    frame.setId(state_id);
    frame.setLength(8);

    const auto position =
        static_cast<std::int32_t>(
            state.position.degrees() * 100.0);

    const auto velocity =
        static_cast<std::int32_t>(
            state.velocity.degreesPerSecond() * 100.0);

    const auto position_value =
        static_cast<std::uint32_t>(position);

    const auto velocity_value =
        static_cast<std::uint32_t>(velocity);

    frame.data()[0] =
        static_cast<std::uint8_t>(position_value & 0xFF);

    frame.data()[1] =
        static_cast<std::uint8_t>(
            (position_value >> 8) & 0xFF);

    frame.data()[2] =
        static_cast<std::uint8_t>(
            (position_value >> 16) & 0xFF);

    frame.data()[3] =
        static_cast<std::uint8_t>(
            (position_value >> 24) & 0xFF);

    frame.data()[4] =
        static_cast<std::uint8_t>(velocity_value & 0xFF);

    frame.data()[5] =
        static_cast<std::uint8_t>(
            (velocity_value >> 8) & 0xFF);

    frame.data()[6] =
        static_cast<std::uint8_t>(
            (velocity_value >> 16) & 0xFF);

    frame.data()[7] =
        static_cast<std::uint8_t>(
            (velocity_value >> 24) & 0xFF);

    return frame;
}

bool JointStateCodec::decode(
    const CanFrame& frame,
    JointState& state) noexcept
{
    if (frame.id() != state_id)
        return false;

    if (frame.length() != 8)
        return false;

    const auto position_value =
        static_cast<std::uint32_t>(frame.data()[0]) |
        (static_cast<std::uint32_t>(frame.data()[1]) << 8) |
        (static_cast<std::uint32_t>(frame.data()[2]) << 16) |
        (static_cast<std::uint32_t>(frame.data()[3]) << 24);

    const auto velocity_value =
        static_cast<std::uint32_t>(frame.data()[4]) |
        (static_cast<std::uint32_t>(frame.data()[5]) << 8) |
        (static_cast<std::uint32_t>(frame.data()[6]) << 16) |
        (static_cast<std::uint32_t>(frame.data()[7]) << 24);

    const auto position =
        static_cast<std::int32_t>(position_value);

    const auto velocity =
        static_cast<std::int32_t>(velocity_value);

    state.position =
        Angle{
            static_cast<double>(position) / 100.0};

    state.velocity =
        AngularVelocity{
            static_cast<double>(velocity) / 100.0};

    return true;
}