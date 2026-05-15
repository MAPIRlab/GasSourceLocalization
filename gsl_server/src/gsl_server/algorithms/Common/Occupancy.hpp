#pragma once

#include <cstdint>
namespace GSL
{
    // this crazy thing is to allow having an enum class with methods (so we can define the bool conversion operator)
    // taken from here: https://stackoverflow.com/a/23383734/19592578
    struct Occupancy
    {
        // the backing values match the ROS standard
        enum : int8_t
        {
            Unknown = -1,
            Obstacle = 100,
            Free = 0
        } value;

        Occupancy() : value(Unknown) {}
        Occupancy(decltype(value) value) : value(value) {} // allow implicit conversion from the enum to the struct type

        operator bool() const
        {
            return value == Free;
        }

        bool operator==(decltype(value) other) const
        {
            return other == value;
        }

        bool operator==(Occupancy other) const
        {
            return other.value == value;
        }

        explicit operator int8_t() const
        {
            return static_cast<int8_t>(value);
        }
    };
} // namespace GSL