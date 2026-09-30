#include "JointCommandCodec.hpp"

#include <cstdint>

CanFrame JointCommandCodec::encode(
    const JointCommand& command) noexcept
{
    CanFrame frame;

    frame.setId(command_id);
    frame.setLength(4);

    const auto centidegrees =
        static_cast<std::int32_t>(
            command.target_position.degrees() * 100.0);

    const auto value =
        static_cast<std::uint32_t>(centidegrees);

    frame.data()[0] =
        static_cast<std::uint8_t>(value & 0xFF);

    frame.data()[1] =
        static_cast<std::uint8_t>((value >> 8) & 0xFF);

    frame.data()[2] =
        static_cast<std::uint8_t>((value >> 16) & 0xFF);

    frame.data()[3] =
        static_cast<std::uint8_t>((value >> 24) & 0xFF);

    return frame;
}

bool JointCommandCodec::decode(
    const CanFrame& frame,
    JointCommand& command) noexcept
{
    if (frame.id() != command_id)
        return false;

    if (frame.length() != 4)
        return false;

    const auto value =
        static_cast<std::uint32_t>(frame.data()[0]) |
        (static_cast<std::uint32_t>(frame.data()[1]) << 8) |
        (static_cast<std::uint32_t>(frame.data()[2]) << 16) |
        (static_cast<std::uint32_t>(frame.data()[3]) << 24);

    const auto centidegrees =
        static_cast<std::int32_t>(value);

    command.target_position =
        Angle{
            static_cast<double>(centidegrees) / 100.0};

    return true;
}