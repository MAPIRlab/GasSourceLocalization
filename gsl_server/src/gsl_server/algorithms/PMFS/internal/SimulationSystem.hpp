#pragma once
#include <gsl_server/algorithms/Common/Simulation.hpp>
#include <gsl_server/algorithms/Common/NQAQuadtree.hpp>
#include <gsl_server/algorithms/PMFS/internal/HitProbability.hpp>
#include <gsl_server/algorithms/PMFS/internal/Settings.hpp>
#include <gsl_server/algorithms/Common/VisibilityMap.hpp>
#include <opencv2/core.hpp>

namespace GSL
{
    class PMFS;
}

namespace GSL::PMFS_internal
{
    class SimulationSystem
    {
        using HashSet = std::unordered_set<Vector2Int>;

        struct SimulationResult
        {
            bool valid = false;
            std::vector<float> hitMap;
            long double sourceProb;
        };

    public:
        SimulationSystem(Grid2D<HitProbability> _measuredHitProb, Grid2D<double> _sourceProb, Grid2D<Vector2> _wind,
                         const PMFS_internal::SimulationSettings& _settings)
            : settings(_settings), measuredHitProb(_measuredHitProb), sourceProb(_sourceProb), wind(_wind)
        {}

        void initializeMap(const std::vector<std::vector<uint8_t>>& occupancyMap);
        void updateSourceProbability(float refineFraction);
        void makeSimulationImage(const SimulationSource& source);
        double probabilitySingleFrequency(double measured, double simulated) const;
        double probabilityFromSingleCell(HitProbability measured, double simulated) const;
        long double sourceProbFromMaps(const Grid2D<HitProbability>& hitRandomVariable, const std::vector<float>& hitMap) const;

        std::vector<std::vector<Utils::NQA::Node*>> mapSegmentation;
        std::unique_ptr<Utils::NQA::Quadtree> quadtree;
        std::vector<Utils::NQA::Node> QTleaves;
        std::vector<double> varianceOfHitProb; // calculated from the simulations, used for movement
        std::vector<SimulationResult> resultsFirstLevel;
        VisibilityMap* visibilityMap;

    protected:
        struct LeafScore
        {
            long double score;
            Utils::NQA::Node* leaf;
        };

        std::vector<long double> sourceProbInternal; // calculated from the simulations, used for movement
        const PMFS_internal::SimulationSettings& settings;
        Grid2D<HitProbability> measuredHitProb;
        Grid2D<double> sourceProb;
        Grid2D<Vector2> wind;
        std::optional<SimulationBlurMask> blurredMask; // used for smoothing out the results of the filament simulation

        SimulationResult runSimulation(std::vector<LeafScore>& nodes, size_t index);
    };
} // namespace GSL::PMFS_internal