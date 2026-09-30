#pragma once

#include "robot_controller/scheduler/IControlScheduler.hpp"

#include <memory>

std::unique_ptr<IControlScheduler>
createControlScheduler(
    std::chrono::milliseconds period);