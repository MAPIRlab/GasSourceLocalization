#include "gsl_server/algorithms/Common/Grid2D.hpp"
#include "gsl_server/algorithms/Common/Utils/RosUtils.hpp"
#include "gsl_server/algorithms/Common/Utils/Time.hpp"
#include "gsl_server/core/Logging.hpp"
#include "gsl_server/core/ros_typedefs.hpp"
#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <gsl_server/algorithms/PMFS/PMFSLib.hpp>
#include <gsl_server/algorithms/PMFS/PMFSViz.hpp>
#include <gsl_server/algorithms/Semantics/SemanticPMFS/SemanticPMFS.hpp>
#include <gsl_server/algorithms/Semantics/Semantics/Common/SemanticsType.hpp>

namespace GSL
{

    SemanticPMFS::SemanticPMFS(std::shared_ptr<rclcpp::Node> _node)
        : Algorithm(_node),
          simulations(Grid2D<HitProbability>(hitProbability, simulationOccupancy, gridMetadata),
                      Grid2D<double>(sourceProbabilityPMFS, simulationOccupancy, gridMetadata),
                      Grid2D<Vector2>(estimatedWindVectors, simulationOccupancy, gridMetadata), settings.simulation),
          pubs(node->get_clock())
              IF_GUI(, ui(this))
    {}

    void SemanticPMFS::Initialize()
    {
        Algorithm::Initialize();

        PMFSLib::InitializePublishers(pubs.pmfsPubs, node);

        waitForGasState = std::make_unique<WaitForGasState>(this);
        waitForMapState = std::make_unique<WaitForMapState>(this);
        waitForMapState->shouldWaitForGas = false;

        stopAndMeasureState = std::make_unique<StopAndMeasureState>(this);
        // TODO
        movingState = std::make_unique<MovingStateSemanticPMFS>(this);
        stateMachine.forceSetState(waitForMapState.get());

#if USE_GUI
        if (!settings.visualization.headless)
            ui.run();
#endif
    }

    void SemanticPMFS::OnUpdate()
    {
        if (!paused)
        {
            Algorithm::OnUpdate();
            if (semantics) // TODO allow this to run slower that the update loop? kinda messes up the callback-based one
            {
                semantics->OnUpdate();
                updateSourceFromSemantics();
            }
        }

        functionQueue.run();

#define DEBUG_VISUALIZATION 0
#if DEBUG_VISUALIZATION
        {
            std::vector<ColorRGBA> colors(sourceProbabilityPMFS.size());
            for (int i = 0; i < sourceProbabilityPMFS.size(); i++)
                colors[i] = Utils::valueToColor(sourceProbabilityPMFS[i],
                                                settings.visualization.sourceLimits.x,
                                                settings.visualization.sourceLimits.y,
                                                settings.visualization.sourceMode);
            Utils::publishDebugMarkers(Grid2D<ColorRGBA>(colors, simulationOccupancy, gridMetadata), "sourceOlfactionOnly");

            std::ofstream outf("progression.csv", std::ios_base::app);
            Vector2 expecOlfOnly = Utils::ExpectedValue(AsGrid(sourceProbabilityPMFS, simulationOccupancy), 1);
            double varianceOlfOnly = Utils::Variance(AsGrid(sourceProbabilityPMFS, simulationOccupancy));
            double errorOlfOnly = vmath::length(expecOlfOnly - resultLogging.sourcePositionGT);

            Vector2 expecBoth = Utils::ExpectedValue(AsGrid(combinedSourceProbability, simulationOccupancy), 1);
            double varianceBoth = Utils::Variance(AsGrid(combinedSourceProbability, simulationOccupancy));
            double errorBoth = vmath::length(expecBoth - resultLogging.sourcePositionGT);
            outf << fmt::format("{};{};  {};{};\n", errorOlfOnly, varianceOlfOnly, errorBoth, varianceBoth);
            outf.close();

            Utils::publishDebugSingleMarker(vmath::WithZ(expecOlfOnly, 0.0),
                                            Utils::create_color(1, 0, 0, 1),
                                            "EXPECTED_OLFACTION");
            Utils::publishDebugSingleMarker(vmath::WithZ(expecBoth, 0.0),
                                            Utils::create_color(1, 0, 1, 1),
                                            "EXPECTED_BOTH");
        }
#endif
    }

    GSLResult SemanticPMFS::checkSourceFound()
    {
        if (stateMachine.getCurrentState() == waitForMapState.get())
            return GSLResult::Running;

        rclcpp::Duration time_spent = node->now() - startTime;
        if (time_spent.seconds() > resultLogging.maxSearchTime)
        {
            saveResultsToFile(GSLResult::Failure);
            return GSLResult::Failure;
        }

        if (resultLogging.navigationTime == -1)
        {
            if (sqrt(pow(currentRobotPose.pose.pose.position.x - resultLogging.sourcePositionGT.x, 2) +
                     pow(currentRobotPose.pose.pose.position.y - resultLogging.sourcePositionGT.y, 2)) < 0.5)
            {
                resultLogging.navigationTime = time_spent.seconds();
            }
        }

        double variance = Utils::Variance(AsGrid(combinedSourceProbability, simulationOccupancy));
        GSL_INFO("Variance: {:.2f}", variance);

        if (variance < 1.f) // TODO parameter
        {
            saveResultsToFile(GSLResult::Success);
            return GSLResult::Success;
        }

        return GSLResult::Running;
    }

    void SemanticPMFS::declareParameters()
    {
        Algorithm::declareParameters();
        PMFSLib::GetHitProbabilitySettings(*this, settings.hitProbability);
        PMFSLib::GetSimulationSettings(*this, settings.simulation);
        settings.visualization.markers_height = getParam<double>("markers_height", 0);
        // number of cells in each direction that we add to the open move set in each step
        settings.movement.openMoveSetExpasion = getParam<int>("openMoveSetExpasion", 5);
        settings.movement.explorationProbability = getParam<double>("explorationProbability", 0.1);
        settings.movement.initialExplorationMoves = getParam<int>("initialExplorationMoves", 5);
        settings.movement.distanceWeight = getParam<double>("distanceWeight", 0.1);
    }

    void SemanticPMFS::onGetMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        Algorithm::onGetMap(msg);

        int scale = getParam<int>("scale", 65); // scale for dynamic map reduction
        PMFSLib::InitMetadata(gridMetadata, map, scale);

        // resize all the vectors to the dimensions of the environment
        //----------------------------------
        hitProbability.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y);
        sourceProbabilityPMFS.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y);
        sourceProbSemantics.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y);
        navigationOccupancy.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y);
        combinedSourceProbability.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y);
        simulationOccupancy = Utils::parseMapImage(getParam<std::string>("wallsOccupancyFile", "?"), gridMetadata);

        visibilityMap.emplace(gridMetadata.dimensions.x, gridMetadata.dimensions.y,
                              std::max(settings.hitProbability.localEstimationWindowSize, settings.movement.openMoveSetExpasion));
        // visibilityMap.range = std::max(settings.movement.openMoveSetExpasion, settings.hitProbability.localEstimationWindowSize);

        // initialize the hit probability and the simulationOccupancy maps
        //----------------------------------
        PMFSLib::InitializeMap(
            Grid2D<HitProbability>(hitProbability, simulationOccupancy, gridMetadata),
            simulations,
            *visibilityMap,
            currentCoordinates());

        for (double& p : sourceProbabilityPMFS)
            p = 1.0 / gridMetadata.numFreeCells;

        for (double& p : sourceProbSemantics)
            p = 1.0 / gridMetadata.numFreeCells;

        // initialize the navigation occupancy
        //----------------------------------
        GridUtils::reduceOccupancyMap(map.data, map.info.width, navigationOccupancy, gridMetadata);
        PMFSLib::PruneUnreachableCells(
            navigationOccupancy,
            gridMetadata,
            currentCoordinates());

        // the wind estimation stuff requires spinning, so it must be done through the function queue
        //----------------------------------
        functionQueue.submit([this]()
                             {
                                 Grid<Vector2> windGrid(estimatedWindVectors, simulationOccupancy, gridMetadata);
                                 PMFSLib::InitializeWindPredictions(
                                     *this,
                                     windGrid,
                                     pubs.pmfsPubs.gmrfWind.request
                                         IF_GADEN(, pubs.pmfsPubs.groundTruthWind.request));
                                 PMFSLib::EstimateWind(
                                     settings.simulation.useWindGroundTruth,
                                     windGrid,
                                     node,
                                     pubs.pmfsPubs.gmrfWind
                                         IF_GADEN(, pubs.pmfsPubs.groundTruthWind));
                                 PMFSLib::EstimatePrior(AsGrid(hitProbability, simulationOccupancy), simulations);
                                 stateMachine.forceSetState(stopAndMeasureState.get());
                             });

        // SEMANTICS
        //----------------------
        std::string semanticsTypeParam = node->declare_parameter<std::string>("semanticsType", "ClassMap2D");
        SemanticsType semanticsType = ParseSemanticsType(semanticsTypeParam);

        if (semanticsType == SemanticsType::ClassMap2D)
            createClassMap2D();
        else if (semanticsType == SemanticsType::ClassMapVoxeland)
            createClassMapVoxeland();
    }

    void SemanticPMFS::updateSourceFromSemantics()
    {
        // wait until we have received the map and initialized everything
        if (!semantics)
            return;

        // don't do this at more than 2 hz
        static Utils::Time::Countdown cd;
        if (!cd.isDone())
            return;
        cd.Restart(0.5);

        semantics->GetSourceProbabilityInPlace(sourceProbSemantics);
#pragma omp parallel for
        for (size_t i = 0; i < sourceProbSemantics.size(); i++)
        {
            combinedSourceProbability[i] = sourceProbabilityPMFS[i] * sourceProbSemantics[i];
        }
        Utils::NormalizeDistribution(combinedSourceProbability, simulationOccupancy);

        PMFSViz::ShowSourceProb(
            Grid2D<double>(combinedSourceProbability, simulationOccupancy, gridMetadata),
            settings.visualization,
            pubs.pmfsPubs);
    }

    void SemanticPMFS::processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection)
    {
        static uint number_of_updates = 0;
        static uint numberHits = 0;

        if (concentration > thresholdGas)
        {
            numberHits++;
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "GAS HIT");
        }
        else
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "NOTHING");

        // If we have already taken enough measurements in this position, process them and get ready to move to the next location
        // ------------------------------
        number_of_updates++;
        if (number_of_updates >= settings.hitProbability.maxUpdatesPerStop)
        {
            float hitFrequency = static_cast<float>(numberHits) / number_of_updates;
            number_of_updates = 0;
            numberHits = 0;

            // Update the gas presence map
            //  ------------------------------
            Grid2D<HitProbability> grid(hitProbability, simulationOccupancy, gridMetadata);
            // Gas & wind
            PMFSLib::EstimateHitProbabilities(grid, *visibilityMap, settings.hitProbability, hitFrequency, windDirection, windSpeed,
                                              gridMetadata.coordinatesToIndices(currentRobotPose.pose.pose));
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "GAS HIT");

            // Update the wind estimations
            //  ------------------------------
            PMFSLib::EstimateWind(settings.simulation.useWindGroundTruth,
                                  AsGrid(estimatedWindVectors, simulationOccupancy),
                                  node,
                                  pubs.pmfsPubs.gmrfWind
                                      IF_GADEN(, pubs.pmfsPubs.groundTruthWind));

            bool timeToSimulate = iterationsCounter >= settings.movement.initialExplorationMoves &&
                                  iterationsCounter % settings.simulation.stepsBetweenSourceUpdates == 0;

            if (timeToSimulate)
                simulations.updateSourceProbability(settings.simulation.refineFraction);

            auto movingStatePMFS = dynamic_cast<MovingStateSemanticPMFS*>(movingState.get());
            if (iterationsCounter > settings.movement.initialExplorationMoves)
                movingStatePMFS->currentMovement = MovingStateSemanticPMFS::MovementType::Search;
            else
                movingStatePMFS->currentMovement = MovingStateSemanticPMFS::MovementType::Exploration;
            movingStatePMFS->chooseGoalAndMove();
            movingStatePMFS->publishMarkers();
        }
        else
            stateMachine.forceResetState(stopAndMeasureState.get());

        PMFSViz::ShowHitProb(
            Grid2D<HitProbability>(hitProbability, simulationOccupancy, gridMetadata),
            settings.visualization,
            pubs.pmfsPubs);
        PMFSViz::ShowSourceProb(
            Grid2D<double>(combinedSourceProbability, simulationOccupancy, gridMetadata),
            settings.visualization,
            pubs.pmfsPubs);
    }

} // namespace GSL