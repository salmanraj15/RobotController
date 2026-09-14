#pragma once

class AngularVelocity
{
public:
    constexpr explicit AngularVelocity(double degrees_per_second)
        : degrees_per_second_{degrees_per_second}
    {
    }

    constexpr double degreesPerSecond() const
    {
        return degrees_per_second_;
    }

private:
    double degrees_per_second_;
};