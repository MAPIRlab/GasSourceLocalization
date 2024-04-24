#pragma once
#include <gsl_server/algorithms/Common/Grid.hpp>
#include <gsl_server/algorithms/PMFS/internal/Simulations.hpp>
#include <gsl_server/algorithms/PMFS/internal/HitProbability.hpp>
#include <gsl_server/algorithms/PMFS/internal/Settings.hpp>
#include <gsl_server/algorithms/PMFS/internal/HitProbKernel.hpp>
#include <gsl_server/algorithms/PMFS/internal/PublishersAndSubscribers.hpp>
#include <gsl_server/algorithms/PMFS/internal/VisibilityMap.hpp>
#include <gsl_server/core/ros_typedefs.hpp>
#include <gsl_server/algorithms/Algorithm.hpp>
#include <gsl_server/core/ConditionalMacros.hpp>

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
        static void estimateHitProbabilities(Grid<HitProbability>& hitLocalVariable, const VisibilityMap& visibilityMap, PMFS_internal::HitProbabilitySettings& settings,
                                             bool hit, double wind_direction, double wind_speed, Vector2Int robot_pos);

        // returns the sum of all auxWeights, for normalization purposes
        static double propagateProbabilities(Grid<HitProbability>& var, const PMFS_internal::HitProbabilitySettings& settings,
                                             HashSet& openSet, HashSet& closedSet, HashSet& activeSet, const HitProbKernel& kernel);
        static double applyFalloffLogOdds(Vector2 originalVectorScaled, const HitProbKernel& kernel,
                                          const PMFS_internal::HitProbabilitySettings& settings);

        static void initMetadata(GridMetadata& metadata, const OccupancyGrid& map, int scale);
        static void initializeMap(Algorithm& algorithm, Grid<HitProbability> grid, PMFS_internal::Simulations& simulations,
                                  VisibilityMap& visibilityMap);
        static void initializeWindPredictions(Algorithm& algorithm, Grid<Vector2> grid, std::shared_ptr<WindEstimation::Request>& GMRFRequest
                                            IF_GADEN(, std::shared_ptr<gaden_player::srv::WindPosition::Request>& groundTruthWindRequest)
        );

        static void estimateWind(bool useGroundTruth, Grid<Vector2> estimatedWind, std::shared_ptr<rclcpp::Node> node,
                               PMFS_internal::GMRFWind& gmrf
                               IF_GADEN(, PMFS_internal::GroundTruthWind& groundTruth)
        );

        static bool pathFree(GridMetadata metadata, const std::vector<Occupancy>& occupancy, const Vector2Int& originInd, const Vector2Int& endInd);
        
        static void normalizeSourceProb(Grid<double>& variable);
        
        static void GetSimulationSettings(Algorithm& algorithm, PMFS_internal::SimulationSettings& settings);
        static void GetHitProbabilitySettings(Algorithm& algorithm, PMFS_internal::HitProbabilitySettings& settings);
    };

} // namespace GSL