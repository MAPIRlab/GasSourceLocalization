#pragma once
#include <gsl_server/algorithms/Common/Grid2D.hpp>
#include <gsl_server/algorithms/PMFS/internal/Simulations.hpp>
#include <gsl_server/algorithms/PMFS/internal/HitProbability.hpp>
#include <gsl_server/algorithms/PMFS/internal/Settings.hpp>
#include <gsl_server/algorithms/PMFS/internal/HitProbKernel.hpp>
#include <gsl_server/algorithms/PMFS/internal/PublishersAndSubscribers.hpp>
#include <gsl_server/algorithms/PMFS/internal/VisibilityMap.hpp>
#include <gsl_server/core/ros_typedefs.hpp>
#include <gsl_server/algorithms/Common/Algorithm.hpp>
#include <gsl_server/core/ConditionalMacros.hpp>

#include <gmrf_wind_mapping/srv/wind_estimation.hpp>

#ifdef USE_GADEN
#include <gaden_msgs/srv/wind_position.hpp>
#endif

namespace GSL
{
    // The core of the PMFS algorithm, implemented in stateless functions.
    // This is useful because it means that to make a variant of PMFS (for example, the semantics version) you do not need to inherit from the PMFS class
    // Your variant will need to have much of the same stuff as PMFS, though (Settings, HitProbability Grid, etc)
    class PMFSLib
    {
        using WindEstimation = gmrf_wind_mapping::srv::WindEstimation;
        using HashSet = std::unordered_set<Vector2Int>;

        using HitProbability = PMFS_internal::HitProbability;
        using HitProbKernel = PMFS_internal::HitProbKernel;

    public:
        static void InitMetadata(Grid2DMetadata& metadata, const OccupancyGrid& map, int scale);
        static void InitializeMap(Algorithm& algorithm, Grid2D<HitProbability> grid, PMFS_internal::Simulations& simulations,
                                  VisibilityMap& visibilityMap);
        static void InitializeWindPredictions(Algorithm& algorithm, Grid2D<Vector2> grid, WindEstimation::Request::SharedPtr& GMRFRequest
                                              IF_GADEN(, gaden_msgs::srv::WindPosition::Request::SharedPtr& groundTruthWindRequest)
                                             );
        static void InitializePublishers(PMFS_internal::PublishersAndSubscribers& pubs, rclcpp::Node::SharedPtr node);

        static void EstimateHitProbabilities(Grid2D<HitProbability>& hitLocalVariable, const VisibilityMap& visibilityMap, PMFS_internal::HitProbabilitySettings& settings,
                                             bool hit, double windDirection, double windSpeed, Vector2Int robotPosition);

        // returns the sum of all auxWeights, for normalization purposes
        static double PropagateProbabilities(Grid2D<HitProbability>& var, const PMFS_internal::HitProbabilitySettings& settings,
                                             HashSet& openSet, HashSet& closedSet, HashSet& activeSet, const HitProbKernel& kernel);

        //Get the updated wind map from the service (Gaden, if using groundTruth, GMRF otherwise)
        static void EstimateWind(bool useGroundTruth, Grid2D<Vector2> estimatedWind, rclcpp::Node::SharedPtr node,
                                 PMFS_internal::GMRFWind& gmrf
                                 IF_GADEN(, PMFS_internal::GroundTruthWind& groundTruth)
                                );

        static void GetSimulationSettings(Algorithm& algorithm, PMFS_internal::SimulationSettings& settings);
        static void GetHitProbabilitySettings(Algorithm& algorithm, PMFS_internal::HitProbabilitySettings& settings);

    private:

        static double applyFalloffLogOdds(Vector2 originalVectorScaled, const HitProbKernel& kernel,
                                          const PMFS_internal::HitProbabilitySettings& settings);
    };

} // namespace GSL