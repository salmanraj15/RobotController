#pragma once

#include "robot_controller/communication/CanFrame.hpp"
#include "robot_controller/core/JointTypes.hpp"

class JointCommandCodec
{
public:
    static CanFrame encode(
        const JointCommand& command) noexcept;

    static bool decode(
        const CanFrame& frame,
        JointCommand& command) noexcept;

private:
    static constexpr std::uint32_t command_id = 0x100;
};