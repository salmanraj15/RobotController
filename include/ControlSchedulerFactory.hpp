#pragma once

#include "IControlScheduler.hpp"

#include <memory>

std::unique_ptr<IControlScheduler>
createControlScheduler(
    std::chrono::milliseconds period);