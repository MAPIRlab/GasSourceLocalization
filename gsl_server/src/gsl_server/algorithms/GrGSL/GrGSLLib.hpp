#pragma once
#include <gsl_server/algorithms/Common/Grid2D.hpp>
#include <gsl_server/core/ros_typedefs.hpp>
#include <gsl_server/algorithms/Common/Algorithm.hpp>
#include "GrGSL_internal.hpp"

namespace GSL
{
    class GrGSLLib
    {
        using HashSet = std::unordered_set<Vector2Int>;
        GrGSLLib() = delete;

    public:

        static void initMetadata(Grid2DMetadata& metadata, const OccupancyGrid& map, int scale);
        static void initializeMap(Algorithm& algorithm, Grid2D<GrGSL_internal::Cell> grid);
        static void GetSettings(rclcpp::Node::SharedPtr node, GrGSL_internal::Settings& settings, GrGSL_internal::Markers& markers);
        static void Normalize(Grid2D<GrGSL_internal::Cell> grid);

        static void estimateProbabilitiesfromGasAndWind(Grid2D<GrGSL_internal::Cell> grid, const GrGSL_internal::Settings& settings,
                bool hit, bool advection, double windDirection, Vector2 positionOfLastHit, Vector2Int robotPosition);
        static void propagateProbabilities(Grid2D<GrGSL_internal::Cell> grid, HashSet& openPropagationSet, HashSet& closedPropagationSet,
                                           HashSet& activePropagationSet);
        static void calculateWeight(Grid2D<GrGSL_internal::Cell> grid, Vector2Int newCell, Vector2Int activeCell, HashSet& openPropagationSet,
                                    HashSet& closedPropagationSet, HashSet& activePropagationSet);

        static double informationGain(
            const GrGSL_internal::WindVector& windVec,
            Grid2D<GrGSL_internal::Cell> grid,
            const GrGSL_internal::Settings& settings,
            Vector2 positionOfLastHit);

        enum MapFunctionMode {Sequential, Parallel};
        static void mapFunctionToCells(Grid2D<GrGSL_internal::Cell> grid, std::function<void(GrGSL_internal::Cell&, size_t)> function,
                                       MapFunctionMode mode = MapFunctionMode::Sequential);
        static void VisualizeMarkers(Grid2D<GrGSL_internal::Cell> grid, GrGSL_internal::Markers& markers, rclcpp::Node::SharedPtr node);
        static Vector2 expectedValueSource(Grid2D<GrGSL_internal::Cell> grid, double proportionBest);
        static double varianceSourcePosition(Grid2D<GrGSL_internal::Cell> grid);
    };
}