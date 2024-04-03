#pragma once
#include <gsl_server/algorithms/Common/Grid.hpp>
#include <gsl_server/algorithms/PMFS/internal/Simulations.hpp>
#include <gsl_server/algorithms/PMFS/internal/Cell.hpp>
#include <gsl_server/algorithms/PMFS/internal/Settings.hpp>
#include <gsl_server/algorithms/PMFS/internal/HitProbKernel.hpp>
#include <gsl_server/core/ros_typedefs.hpp>
#include <gsl_server/algorithms/Algorithm.hpp>

namespace GSL
{

class PMFSLib
{
    using hashSet = std::unordered_set<Vector2Int>;

    template <typename T>
    using Grid = Grid<T>;
    using HitProbability = PMFS_internal::HitProbability;
    using HitProbKernel = PMFS_internal::HitProbKernel;

public:
    static void estimateHitProbabilities(Grid<HitProbability>& hitLocalVariable, PMFS_internal::Settings::HitProbabilitySettings& settings, bool hit, double wind_direction, double wind_speed,
                                      Vector2Int robot_pos);

    // returns the sum of all auxWeights, for normalization purposes
    static double propagateProbabilities(Grid<HitProbability>& var, PMFS_internal::Settings::HitProbabilitySettings& settings, hashSet& openSet, hashSet& closedSet, hashSet& activeSet,
                                    const HitProbKernel& kernel);
    static double applyFalloffLogOdds(Vector2 originalVectorScaled, const HitProbKernel& kernel, PMFS_internal::Settings::HitProbabilitySettings& settings);

    static void initMetadata(GridMetadata& metadata, const OccupancyGrid& map,  int scale);
    void initializeMap(std::shared_ptr<Algorithm> algorithm, Grid<HitProbability> grid, const PMFS_internal::Settings& settings, PMFS_internal::Simulations& simulations);
    void initializeWindPredictions();
};

}