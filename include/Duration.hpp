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

private:
    double seconds_;
};