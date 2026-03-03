#pragma once
#include <filesystem>
#include <vector>
#include <gmrf_wind_core/gmrf_map.h>

namespace GSL
{
    class Graph
    {
    public:
        static Graph ReadFromDisk(const std::filesystem::path& folder, float cellSize, gmrfw::CGMRF_map::Parameters gmrfParams);

    public:
        std::vector<std::shared_ptr<class Node>> nodes;
    };
} // namespace GSL