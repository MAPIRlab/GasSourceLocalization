#pragma once
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>
#include <gsl_server/core/Vectors.hpp>

namespace GSL::PMFS_internal
{
    struct DeclarationSettings
    {
        float threshold = 1.0;

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

        float kernelSigma = 1.0f;
        float kernelStretchConstant = 1.0f;
        float evidenceMassObservation = 1.0f;
    };

    struct SimulationSettings
    {
        bool useWindGroundTruth = false;
        int maxRegionSize = 10;
        int stepsBetweenSourceUpdates = 3;
        float sourceDiscriminationPower = 0.2; // higher values here will lead to a larger difference in the estimated source probability from one cell to another
        float refineFraction = 0.25;           // proportion of the cells that will be subdivided for the finer simulation

        int maxWarmupIterations = 500;
        int minWarmupIterations = 0;
        int iterationsToRecord = 100;
        float deltaTime = 0.1;
        float noiseSTDev = 0.2;

        float blurSigmaX = 0;
        float blurSigmaY = 0;
    };

    struct MovementSettings
    {
        float explorationProbability = 0.05;
        int openMoveSetExpasion = 5;
        int initialExplorationMoves = 3;
        float distanceWeight = 0; // how much to favor nearby positions, even if the far ones have a bit more interest. 0 to ignore distance and always maximize interest
    };

    struct VisualizationSettings
    {
        bool headless = false;
        Utils::valueColorMode hitMode = Utils::valueColorMode::Linear;
        Utils::valueColorMode sourceMode = Utils::valueColorMode::Logarithmic;

        // range of values that will be mapped to blue-red
        Vector2 hitLimits = Vector2(0, 1);
        Vector2 sourceLimits = Vector2(0.00001, 0.1);
        float markers_height = 0;
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