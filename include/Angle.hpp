#pragma once

class Angle
{
public:
    // Creates an angle measured in degrees.
    constexpr explicit Angle(double degrees)
        : degrees_{degrees}
    {
    }

    // Returns the angle value in degrees.
    constexpr double degrees() const
    {
        return degrees_;
    }

private:
    // The stored angle value, expressed in degrees.
    double degrees_;
};