#pragma once

#include <array>
#include <cstdint>

class CanFrame
{
public:
    static constexpr std::size_t max_data_length = 8;

    std::uint32_t id() const noexcept;
    void setId(std::uint32_t id) noexcept;

    std::uint8_t length() const noexcept;
    void setLength(std::uint8_t length) noexcept;

    const std::array<std::uint8_t, max_data_length>&
    data() const noexcept;

    std::array<std::uint8_t, max_data_length>&
    data() noexcept;

private:
    std::uint32_t id_{};
    std::uint8_t length_{};
    std::array<std::uint8_t, max_data_length> data_{};
};