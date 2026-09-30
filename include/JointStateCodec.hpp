#pragma once

#include "CanFrame.hpp"
#include "JointTypes.hpp"

class JointStateCodec
{
public:
    static CanFrame encode(
        const JointState& state) noexcept;

    static bool decode(
        const CanFrame& frame,
        JointState& state) noexcept;

private:
    static constexpr std::uint32_t state_id = 0x200;
};