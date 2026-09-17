#pragma once

class AngularAcceleration
{
public:
    // Creates an angular acceleration measured in degrees per second squared.
    constexpr explicit AngularAcceleration(
        double degrees_per_second_squared)
        : degrees_per_second_squared_{degrees_per_second_squared}
    {
    }

    // Returns the angular acceleration in degrees per second squared.
    constexpr double degreesPerSecondSquared() const
    {
        return degrees_per_second_squared_;
    }

private:
    // The stored acceleration value, expressed in degrees per second squared.
    double degrees_per_second_squared_;
};