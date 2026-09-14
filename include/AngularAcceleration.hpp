#pragma once

class AngularAcceleration
{
public:
    constexpr explicit AngularAcceleration(
        double degrees_per_second_squared)
        : degrees_per_second_squared_{degrees_per_second_squared}
    {
    }

    constexpr double degreesPerSecondSquared() const
    {
        return degrees_per_second_squared_;
    }

private:
    double degrees_per_second_squared_;
};