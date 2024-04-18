#include <gsl_server/algorithms/PMFS/PMFS.hpp>
#include <gsl_server/Utils/Math.hpp>
#include <angles/angles.h>
#include <gsl_server/algorithms/PMFS/PMFSLib.hpp>

namespace GSL
{
    using HashSet = std::unordered_set<Vector2Int>;

    void PMFS::OnUpdate()
    {
        if(!paused)
            Algorithm::OnUpdate();
        else
        {
            showWeights();
            dynamic_cast<MovingStatePMFS*>(movingState.get())->publishMarkers();
        }

        functionQueue.run();
    }

    void PMFS::processGasAndWindMeasurements(double concentration, double wind_speed, double wind_direction)
    {
        static int number_of_updates = 0;

        Grid<HitProbability> grid(hitProbability, simulationOccupancy, gridMetadata);
        if (concentration > thresholdGas)
        {
            // Gas & wind
            PMFSLib::estimateHitProbabilities(grid, visibilityMap, settings.hitProbability, true, wind_direction, wind_speed, currentPosIndex());
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "GAS HIT");
        }
        else
        {
            // Nothing
            PMFSLib::estimateHitProbabilities(grid, visibilityMap, settings.hitProbability, false, wind_direction, wind_speed, currentPosIndex());
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "NOTHING ");
        }

        PMFSLib::estimateWind(settings.simulation.useWindGroundTruth, 
                            Grid<Vector2>(estimatedWindVectors, simulationOccupancy, gridMetadata), 
                            node,
                            pubs.gmrfWind, pubs.groundTruthWind);
        plotWindVectors();

        number_of_updates++;

        if (number_of_updates >= settings.hitProbability.max_updates_per_stop)
        {
            number_of_updates = 0;
            bool timeToSimulate = iterationsCounter >= settings.movement.initialExplorationMoves &&
                                  iterationsCounter % settings.simulation.steps_between_source_updates == 0;
            if (timeToSimulate)
            {
                // simulations.compareRefineFractions();
                simulations.updateSourceProbability(settings.simulation.refineFraction);
            }

            auto movingStatePMFS = dynamic_cast<MovingStatePMFS*>(movingState.get());
            if (iterationsCounter > settings.movement.initialExplorationMoves)
                movingStatePMFS->currentMovement = MovingStatePMFS::MovementType::Search;
            else
                movingStatePMFS->currentMovement = MovingStatePMFS::MovementType::Exploration;

            movingStatePMFS->chooseGoalAndMove();
            movingStatePMFS->publishMarkers();
        }
        else
            stateMachine.forceResetState(stopAndMeasureState.get());

        showWeights();
    }

    float PMFS::gasCallback(olfaction_msgs::msg::GasSensor::SharedPtr msg) 
    {
        float ppm = Algorithm::gasCallback(msg);
        IF_GUI(ui.addConcentrationReading(ppm));
        return ppm;
    }


} // namespace GSL