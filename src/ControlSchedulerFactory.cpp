#include "ControlSchedulerFactory.hpp"

#ifdef _WIN32
#include "WindowsControlScheduler.hpp"
#elif defined(__linux__)
#include "LinuxControlScheduler.hpp"
#else
#include "PortableControlScheduler.hpp"
#endif

std::unique_ptr<IControlScheduler>
createControlScheduler(
    std::chrono::milliseconds period)
{
#ifdef _WIN32
    return std::make_unique<WindowsControlScheduler>(period);
#elif defined(__linux__)
    return std::make_unique<LinuxControlScheduler>(period);
#else
    return std::make_unique<PortableControlScheduler>(period);
#endif
}