#pragma once
#include <gsl_server/core/Vectors.hpp>
#include <gsl_server/Utils/RosUtils.hpp>

namespace GSL::PMFS_internal
{
    struct DeclarationSettings
    {
        double threshold;
        int steps;

        enum DeclarationMode
        {
            Variance,
            Stability,
            Entropy
        } mode = Variance;
    };
    struct HitProbabilitySettings
    {
        int localEstimationWindowSize;
        int maxUpdatesPerStop;
        double prior;

        double kernelSigma;
        double kernelStretchConstant;

        double confidenceSigmaSpatial;      // the sigma of the confidence normal associated with each measurement -- confidence as a function of
                                              // physical proximity to the measurement location, this is not variance or anything like that
        double confidenceMeasurementWeight; // sigma_omega in kernel DMV. Controls how much confidence you gain from one measurement (affects
                                              // all distances)
    };
    struct SimulationSettings
    {
        bool useWindGroundTruth;
        int maxRegionSize;
        int stepsBetweenSourceUpdates;
        double sourceDiscriminationPower; // higher values here will lead to a larger difference in the estimated source probability from one cell
                                          // to another
        double refineFraction;            // proportion of the cells that will be subdivided for the finer simulation

        int maxWarmupIterations;
        int iterationsToRecord;
        double deltaTime;
        double noiseSTDev;
    };
    struct MovementSettings
    {
        double explorationProbability;
        int openMoveSetExpasion;
        int initialExplorationMoves;
    };
    struct VisualizationSettings
    {
        bool headless;
        Utils::valueColorMode hitMode = Utils::valueColorMode::Linear;
        Utils::valueColorMode sourceMode = Utils::valueColorMode::Logarithmic;

        // range of values that will be mapped to blue-red
        Vector2 hitLimits = Vector2(0, 1);
        Vector2 sourceLimits = Vector2(0.00001, 0.1);
        double markers_height;
    };

    struct Settings
    {
        DeclarationSettings declaration;
        MovementSettings movement;
        HitProbabilitySettings hitProbability;
        SimulationSettings simulation;
        VisualizationSettings visualization;
    };

} // namespace GSL::PMFS_internal