#pragma once
#include <gsl_server/algorithms/Common/Grid.hpp>
#include <gsl_server/algorithms/PMFS/internal/Simulations.hpp>
#include <gsl_server/algorithms/PMFS/internal/Cell.hpp>
#include <gsl_server/algorithms/PMFS/internal/Settings.hpp>
#include <gsl_server/algorithms/PMFS/internal/HitProbKernel.hpp>
#include <gsl_server/algorithms/PMFS/internal/PublishersAndSubscribers.hpp>
#include <gsl_server/core/ros_typedefs.hpp>
#include <gsl_server/algorithms/Algorithm.hpp>

#include <gmrf_wind_mapping/srv/wind_estimation.hpp>

#ifdef USE_GADEN
#include <gaden_player/srv/wind_position.hpp>
#endif

namespace GSL
{

    class PMFSLib
    {
        using WindEstimation = gmrf_wind_mapping::srv::WindEstimation;
        using HashSet = std::unordered_set<Vector2Int>;

        template <typename T> using Grid = Grid<T>;
        using HitProbability = PMFS_internal::HitProbability;
        using HitProbKernel = PMFS_internal::HitProbKernel;

    public:
        static void estimateHitProbabilities(Grid<HitProbability>& hitLocalVariable, const std::unordered_map<Vector2Int, HashSet>& visibilityMap, PMFS_internal::Settings::HitProbabilitySettings& settings,
                                             bool hit, double wind_direction, double wind_speed, Vector2Int robot_pos);

        // returns the sum of all auxWeights, for normalization purposes
        static double propagateProbabilities(Grid<HitProbability>& var, const PMFS_internal::Settings::HitProbabilitySettings& settings,
                                             HashSet& openSet, HashSet& closedSet, HashSet& activeSet, const HitProbKernel& kernel);
        static double applyFalloffLogOdds(Vector2 originalVectorScaled, const HitProbKernel& kernel,
                                          const PMFS_internal::Settings::HitProbabilitySettings& settings);

        static void initMetadata(GridMetadata& metadata, const OccupancyGrid& map, int scale);
        static void initializeMap(Algorithm& algorithm, Grid<HitProbability> grid, const PMFS_internal::Settings& settings,
                                  PMFS_internal::Simulations& simulations, std::unordered_map<Vector2Int, HashSet>& visibilityMap);
        static void initializeWindPredictions(Algorithm& algorithm, Grid<Vector2> grid, std::shared_ptr<WindEstimation::Request>& GMRFRequest
#ifdef USE_GADEN
                                              , std::shared_ptr<gaden_player::srv::WindPosition::Request>& groundTruthWindRequest
#endif
        );

        static void PMFSLib::estimateWind(bool useGroundTruth, Grid<Vector2> estimatedWind, std::shared_ptr<rclcpp::Node> node,
                                          PMFS_internal::PublishersAndSubscribers::GMRFWind& gmrf
#ifdef USE_GADEN
                                          , PMFS_internal::PublishersAndSubscribers::GroundTruthWind& groundTruth
#endif
        );

        template <typename T> static bool pathFree(Grid<T> grid, const Vector2Int& origin, const Vector2Int& end)
        {
            // check there are no obstacles between origin and end
            if (!(grid.freeAt(origin.x, origin.y) && grid.freeAt(end.x, end.y)))
                return false;

            bool pathIsFree = true;
            Vector2 vector = grid.metadata.indexToCoordinates(end.x, end.y) - grid.metadata.indexToCoordinates(origin.x, origin.y);
            Vector2 increment = glm::normalize(vector) * (grid.metadata.cellSize);
            int steps = glm::length(vector) / (grid.metadata.cellSize);
            int index = 0;
            Vector2 current_point = grid.metadata.indexToCoordinates(origin.x, origin.y);
            while (index < steps && pathIsFree)
            {
                current_point += increment;
                index++;
                Vector2Int pair = grid.metadata.coordinatesToIndex(current_point.x, current_point.y);
                pathIsFree = indicesInBounds(pair, grid.metadata) && grid.freeAt(pair.x, pair.y);
            }

            return pathIsFree;
        }
        static bool indicesInBounds(const Vector2Int indices, const GridMetadata& metadata);
        
    };

} // namespace GSL