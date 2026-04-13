#pragma once
#include "gsl_server/algorithms/Common/Grid2D.hpp"
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
        static Graph ReadFromDisk(const std::filesystem::path& folder, float cellSize, gmrfw::CGMRF_map::Parameters gmrfParams);
        std::shared_ptr<class Node> GetCorrespondingNode(Vector2 position);
        void AddObservation(Vector2 position, Vector2 wind, float gasConcentration);
        void UpdateAllWindMaps();

        static gmrfw::TOccupancyMap ToGMRFOcc(const Grid2D<Occupancy> occupancy);
        std::shared_ptr<gmrfw::CGMRF_map> gmrf;
        gmrfw::CGMRF_map::Parameters gmrf_parameters;
        Map2D completeMap;

        // visualization
        MarkerArray VisualizeGraph();
        MarkerArray VisualizeOccupancy();
        MarkerArray VisualizeWind();

    public:
        std::vector<std::shared_ptr<class Node>> nodes;
        std::map<std::string, bool> selectedForVisualization;
        float nodeSeparationViz = 1; //multiplier for the origin of each node (for visualization only) Makes it easier to see which area corresponds to which node

    private:
        void MergeWindMarkers(MarkerArray& all, const MarkerArray& _new);
    };
} // namespace GSL