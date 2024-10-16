#pragma once
#include <gsl_server/core/Vectors.hpp>
#include <gsl_server/Utils/RosUtils.hpp>

namespace GSL::PMFS_internal
{
    struct Settings
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
        } declaration;

        // movement
        struct MovementSettings
        {
            double explorationProbability;
            int openMoveSetExpasion;
            int initialExplorationMoves;
        } movement;

        struct HitProbabilitySettings
        {
            int localEstimationWindowSize;
            int max_updates_per_stop;
            double prior;

            double kernel_sigma;
            double kernel_stretch_constant;

            double confidence_sigma_spatial; // the sigma of the confidence normal associated with each measurement -- confidence as a function of
                                             // physical proximity to the measurement location, this is not variance or anything like that
            double confidence_measurement_weight; // sigma_omega in kernel DMV. Controls how much confidence you gain from one measurement (affects
                                                  // all distances)
        } hitProbability;

        struct SimulationSettings
        {
            bool useWindGroundTruth;
            int maxRegionSize;
            int steps_between_source_updates;
            double sourceDiscriminationPower; // higher values here will lead to a larger difference in the estimated source probability from one cell
                                              // to another
            double refineFraction;            // proportion of the cells that will be subdivided for the finer simulation

            int maxWarmupIterations;
            int iterationsToRecord;
            double deltaTime;
            double noiseSTDev;
        } simulation;

        struct VisualizationSettings
        {
			bool headless;
            Utils::valueColorMode hitMode = Utils::valueColorMode::Linear;
            Utils::valueColorMode sourceMode = Utils::valueColorMode::Logarithmic;

            // range of values that will be mapped to blue-red
            Vector2 hitLimits = Vector2(0, 1);
            Vector2 sourceLimits = Vector2(0.00001, 0.1);
            double markers_height;
        } visualization;
    };

} // namespace GSL