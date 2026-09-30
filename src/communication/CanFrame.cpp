#include "robot_controller/communication/CanFrame.hpp"

#include <array>
#include <cstdint>

std::uint32_t CanFrame::id() const noexcept
{
    return id_;
}

void CanFrame::setId(std::uint32_t id) noexcept
{
    id_ = id;
}

std::uint8_t CanFrame::length() const noexcept
{
    return length_;
}

void CanFrame::setLength(std::uint8_t length) noexcept
{
    length_ = length;
}

const std::array<std::uint8_t, CanFrame::max_data_length>&
CanFrame::data() const noexcept
{
    return data_;
}

std::array<std::uint8_t, CanFrame::max_data_length>&
CanFrame::data() noexcept
{
    return data_;
}