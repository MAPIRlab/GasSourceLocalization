#pragma once
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>
#include <gsl_server/core/Vectors.hpp>

namespace GSL::PMFS_internal
{
    struct DeclarationSettings
    {
        double threshold = 1.0;

        enum DeclarationMode
        {
            Variance,
            Entropy
        } mode = Variance;
    };
    struct HitProbabilitySettings
    {
        int localEstimationWindowSize = 3;
        int maxUpdatesPerStop = 5;
        double prior = 0.3;

        double kernelSigma = 1.0;
        double kernelStretchConstant = 1.0;

        double confidenceSigmaSpatial = 1.0;      // the sigma of the confidence normal associated with each measurement -- confidence as a function of physical proximity to the measurement location, this is not variance or anything like that
        double confidenceMeasurementWeight = 1.0; // sigma_omega in kernel DMV. Controls how much confidence you gain from one measurement (affects all distances)
    };
    struct SimulationSettings
    {
        bool useWindGroundTruth = false;
        int maxRegionSize = 10;
        int stepsBetweenSourceUpdates = 3;      // make negative to disable updates. Useful if you want to always trigger them manually from UI
        double sourceDiscriminationPower = 0.2; // higher values here will lead to a larger difference in the estimated source probability from one cell to another
        double refineFraction = 0.25;           // proportion of the cells that will be subdivided for the finer simulation

        size_t maxWarmupIterations = 500;
        size_t minWarmupIterations = 0;
        size_t iterationsToRecord = 100;
        double deltaTime = 0.1;
        double noiseSTDev = 0.2;

        double blurSigmaX = 0;
        double blurSigmaY = 0;
    };
    struct MovementSettings
    {
        double explorationProbability = 0.05;
        int openMoveSetExpasion = 5;
        int initialExplorationMoves = 3;
        double distanceWeight = 0; // how much to favor nearby positions, even if the far ones have a bit more interest. 0 to ignore distance and always maximize interest
    };
    struct VisualizationSettings
    {
        bool headless = false;
        Utils::valueColorMode hitMode = Utils::valueColorMode::Linear;
        Utils::valueColorMode sourceMode = Utils::valueColorMode::Logarithmic;

        // range of values that will be mapped to blue-red
        Vector2 hitLimits = Vector2(0, 1);
        Vector2 sourceLimits = Vector2(0.00001, 0.1);
        double markers_height = 0;
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