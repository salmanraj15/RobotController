#pragma once

class Duration
{
public:
    // Creates a duration measured in seconds.
    constexpr explicit Duration(double seconds)
        : seconds_{seconds}
    {
    }

    // Returns the duration in seconds.
    constexpr double seconds() const
    {
        return seconds_;
    }

    // Adds another duration to this duration.
    Duration& operator+=(const Duration& other)
    {
        seconds_ += other.seconds_;
        return *this;
    }

private:
    // The stored duration value, expressed in seconds.
    double seconds_;
};