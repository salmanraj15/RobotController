#pragma once

class Duration
{
public:
    constexpr explicit Duration(double seconds)
        : seconds_{seconds}
    {
    }

    constexpr double seconds() const
    {
        return seconds_;
    }

    Duration& operator+=(const Duration& other)
    {
        seconds_ += other.seconds_;
        return *this;
    }

private:
    double seconds_;
};