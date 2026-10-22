#pragma once

#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <span>
#include <vector>

namespace dbscan
{
    struct point2
    {
        float x, y;
    };

    struct point3
    {
        float x, y, z;
    };

    auto dbscan(const std::span<const point2>& data, float eps, int min_pts) -> std::vector<std::vector<size_t>>;
    auto dbscan(const std::span<const point3>& data, float eps, int min_pts) -> std::vector<std::vector<size_t>>;

}  // namespace dbscan