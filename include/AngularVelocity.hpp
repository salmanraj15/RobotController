#pragma once

class AngularVelocity
{
public:
    // Creates an angular velocity measured in degrees per second.
    constexpr explicit AngularVelocity(double degrees_per_second)
        : degrees_per_second_{degrees_per_second}
    {
    }

    // Returns the angular velocity in degrees per second.
    constexpr double degreesPerSecond() const
    {
        return degrees_per_second_;
    }

private:
    // The stored velocity value, expressed in degrees per second.
    double degrees_per_second_;
};