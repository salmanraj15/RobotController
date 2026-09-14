#pragma once

class Angle
{
public:
    constexpr explicit Angle(double degrees)
        : degrees_{degrees}
    {
    }

    constexpr double degrees() const
    {
        return degrees_;
    }

private:
    double degrees_;
};