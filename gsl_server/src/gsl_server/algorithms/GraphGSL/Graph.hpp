#pragma once
#include "gsl_server/core/ros_typedefs.hpp"
#include <gsl_server/core/Vectors.hpp>
#include <filesystem>
#include <vector>
#include <gmrf_wind_core/gmrf_map.h>

namespace GSL
{
    class Graph
    {
    public:
        static Graph ReadFromDisk(const std::filesystem::path& folder, float cellSize, float nodeSeparationMultiplier, gmrfw::CGMRF_map::Parameters gmrfParams);
        void AddObservation(Vector2 position, Vector2 wind, float gasConcentration);

        MarkerArray VisualizeGraph();

    public:
        std::vector<std::shared_ptr<class Node>> nodes;
    };
} // namespace GSL