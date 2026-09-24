#pragma once

#include "PortableControlScheduler.hpp"

class WindowsControlScheduler final
    : public PortableControlScheduler
{
public:
    explicit WindowsControlScheduler(
        std::chrono::milliseconds period);
};