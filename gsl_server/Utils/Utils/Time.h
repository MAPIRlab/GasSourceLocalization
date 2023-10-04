#pragma once
#include <chrono>
#include <cmath>

namespace Utils::Time
{
    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::_V2::system_clock::duration Duration;
    typedef std::chrono::_V2::system_clock::time_point TimePoint;
    inline double toSeconds(Duration duration)
    {
        return duration.count() / std::pow(10, 9);
    }

    struct Stopwatch
    {
        std::chrono::high_resolution_clock clock;
        std::chrono::_V2::system_clock::time_point start;

        Stopwatch() : clock(), start(clock.now())
        {
        }

        double ellapsed()
        {
            return toSeconds(clock.now() - start);
        }
    };
} // namespace Utils::Time