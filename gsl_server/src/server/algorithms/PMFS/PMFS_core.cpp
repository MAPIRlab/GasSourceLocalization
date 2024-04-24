#include <gsl_server/algorithms/PMFS/PMFS.hpp>
#include <gsl_server/Utils/Math.hpp>
#include <angles/angles.h>
#include <gsl_server/algorithms/PMFS/PMFSLib.hpp>
#include <gsl_server/algorithms/PMFS/PMFSViz.hpp>

namespace GSL
{
    using HashSet = std::unordered_set<Vector2Int>;

    void PMFS::OnUpdate()
    {
        if(!paused)
            Algorithm::OnUpdate();

        dynamic_cast<MovingStatePMFS*>(movingState.get())->publishMarkers();
        PMFSViz::ShowHitProb(Grid<HitProbability>(hitProbability, occupancy, gridMetadata), settings.visualization, pubs);
        PMFSViz::ShowSourceProb(Grid<double>(sourceProbability, occupancy, gridMetadata), settings.visualization, pubs);
        functionQueue.run();
    }

    void PMFS::processGasAndWindMeasurements(double concentration, double wind_speed, double wind_direction)
    {
        static int number_of_updates = 0;

        Grid<HitProbability> grid(hitProbability, occupancy, gridMetadata);
        if (concentration > thresholdGas)
        {
            // Gas & wind
            PMFSLib::estimateHitProbabilities(grid, *visibilityMap, settings.hitProbability, true, wind_direction, wind_speed,
                                              gridMetadata.coordinatesToIndex(currentRobotPose));
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "GAS HIT");
        }
        else
        {
            // Nothing
            PMFSLib::estimateHitProbabilities(grid, *visibilityMap, settings.hitProbability, false, wind_direction, wind_speed,
                                              gridMetadata.coordinatesToIndex(currentRobotPose));
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "NOTHING ");
        }

        PMFSLib::estimateWind(settings.simulation.useWindGroundTruth, 
                            Grid<Vector2>(estimatedWindVectors, occupancy, gridMetadata), 
                            node,
                            pubs.gmrfWind
                            IF_GADEN(, pubs.groundTruthWind));
        PMFSViz::PlotWindVectors(Grid<Vector2>(estimatedWindVectors, occupancy, gridMetadata), settings.visualization, pubs);

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
        

        PMFSViz::ShowHitProb(Grid<HitProbability>(hitProbability, occupancy, gridMetadata), settings.visualization, pubs);
        PMFSViz::ShowSourceProb(Grid<double>(sourceProbability, occupancy, gridMetadata), settings.visualization, pubs);
    }

    float PMFS::gasCallback(olfaction_msgs::msg::GasSensor::SharedPtr msg) 
    {
        float ppm = Algorithm::gasCallback(msg);
        IF_GUI(ui.addConcentrationReading(ppm));
        return ppm;
    }


} // namespace GSL