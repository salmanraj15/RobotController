#pragma once

#include "CanFrame.hpp"

class ICanInterface
{
public:
    virtual ~ICanInterface() = default;

    virtual bool receive(CanFrame& frame) noexcept = 0;
    virtual bool transmit(const CanFrame& frame) noexcept = 0;
};