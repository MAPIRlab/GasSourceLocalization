#pragma once

namespace PMFS
{

    struct GridSettings
    {
        int scale;

        int localEstimationWindowSize;

        // measuring parameters
        double stop_and_measure_time;
        double th_gas_present;
        double th_wind_present;

        double markers_height;

        struct DeclarationSettings
        {
            double threshold;
            int steps;

            enum DeclarationMode
            {
                Variance,
                Stability,
                Entropy
            };
            DeclarationMode mode = Variance;
        };
        DeclarationSettings declaration;

        // movement
        struct MovementSettings
        {
            double explorationProbability;
            bool allowMovementRepetition;
            int openMoveSetExpasion;
            int initialExplorationMoves;
        };
        MovementSettings movement;

        struct HitProbabilitySettings
        {
            int max_updates_per_stop;
            double prior;

            double kernel_sigma;
            double kernel_stretch_constant;

            double confidence_sigma_spatial; // the sigma of the confidence normal associated with each measurement -- confidence as a function of
                                             // physical proximity to the measurement location, this is not variance or anything like that
            double confidence_measurement_weight; // sigma_omega in kernel DMV. Controls how much confidence you gain from one measurement (affects
                                                  // all distances)
        };
        HitProbabilitySettings hitProbability;

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
        };
        SimulationSettings simulation;

        struct VisualizationSettings
        {
            Utils::valueColorMode hitMode = Utils::valueColorMode::Linear;
            Utils::valueColorMode sourceMode = Utils::valueColorMode::Logarithmic;

            // range of values that will be mapped to blue-red
            Utils::Vector2 hitLimits = Utils::Vector2(0, 1);
            Utils::Vector2 sourceLimits = Utils::Vector2(0.00001, 0.1);
        };
        VisualizationSettings visualization;
    };

} // namespace PMFS