#pragma once
#include "gsl_server/core/ros_typedefs.hpp"
#include <filesystem>
#include <gmrf_wind_core/gmrf_map.h>
#include <gsl_server/core/Vectors.hpp>
#include <vector>

namespace GSL
{
    class Graph
    {
    public:
        static Graph ReadFromDisk(const std::filesystem::path& folder, float cellSize, float nodeSeparationMultiplier, gmrfw::CGMRF_map::Parameters gmrfParams);
        void AddObservation(Vector2 position, Vector2 wind, float gasConcentration);
        void UpdateAllWindMaps();

        // visualization
        MarkerArray VisualizeGraph();
        MarkerArray VisualizeOccupancy();
        MarkerArray VisualizeWind();

    public:
        std::vector<std::shared_ptr<class Node>> nodes;
        std::map<std::string, bool> selectedForVisualization;

    private:
        void MergeWindMarkers(MarkerArray& all, const MarkerArray& _new);
    };
} // namespace GSL