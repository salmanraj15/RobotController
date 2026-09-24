#include "WindowsControlScheduler.hpp"

WindowsControlScheduler::WindowsControlScheduler(
    std::chrono::milliseconds period)
    : PortableControlScheduler{period}
{
}