#include <fstream>
#include <gsl_server/algorithms/Common/Grid2D.hpp>
#include <gsl_server/algorithms/Common/States/ManualNavigation.hpp>
#include <gsl_server/algorithms/Common/States/NoNavigation.hpp>
#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>
#include <gsl_server/algorithms/Common/Utils/Time.hpp>
#include <gsl_server/algorithms/PMFS/PMFSLib.hpp>
#include <gsl_server/algorithms/PMFS/PMFSViz.hpp>
#include <gsl_server/algorithms/Semantics/SemanticPMFS/SemanticPMFS.hpp>
#include <gsl_server/algorithms/Semantics/Semantics/Common/SemanticsType.hpp>
#include <gsl_server/core/Logging.hpp>
#include <gsl_server/core/ros_typedefs.hpp>

#define DEBUG_VISUALIZATION 1
namespace GSL
{
    static std::ofstream progressionFile;
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
#if DISABLE_NAVIGATION
        movingState = std::make_unique<ManualNavigationState>(this);
#else
        movingState = std::make_unique<MovingStateSemanticPMFS>(this);
#endif
        stateMachine.forceSetState(waitForMapState.get());

#if USE_GUI
        if (!settings.visualization.headless)
            ui.run();
#endif
        std::string progresionFileName = getParam<std::string>("progressionFileName", "progression.csv");
        progressionFile.open(progresionFileName, std::ios_base::app);
        progressionFile << "New run\n";
        progressionFile << "...............................\n";
        progressionFile << "errorOlfOnly; varianceOlfOnly; errorBoth; varianceBoth\n";
        progressionFile << "-------------------------------\n";
        progressionFile.flush();
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
    }

    void SemanticPMFS::declareParameters()
    {
        Algorithm::declareParameters();
        PMFSLib::GetHitProbabilitySettings(*this, settings.hitProbability);
        PMFSLib::GetSimulationSettings(*this, settings.simulation);
        PMFSLib::GetDeclarationSettings(*this, settings.declaration);
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

        // set all variables to the prior probability
        for (HitProbability& h : hitProbability)
            h.setProbability(settings.hitProbability.prior);

        for (double& p : sourceProbabilityPMFS)
            p = 1.0 / gridMetadata.numFreeCells;

        for (double& p : sourceProbSemantics)
            p = 1.0 / gridMetadata.numFreeCells;

        // initialize the navigation occupancy
        //----------------------------------

        // create the navigation map from the OccupancyGrid published by map_server
        GridUtils::reduceOccupancyMap(map.data, map.info.width, navigationOccupancy, gridMetadata);

        // read a version of the map that is specifically for navigation, rather than using whatever the map server published (which may be modified for GMRF)
        // navigationOccupancy = Utils::parseMapImage(getParam<std::string>("navigationOccupancyFile", "?"), gridMetadata);

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
                                     settings.simulation,
                                     windGrid,
                                     pubs.pmfsPubs.gmrfWind.request
                                         IF_GADEN(, pubs.pmfsPubs.groundTruthWind.request));
                                 PMFSLib::EstimateWind(
                                     settings.simulation.useWindGroundTruth,
                                     windGrid,
                                     node,
                                     pubs.pmfsPubs.gmrfWind
                                         IF_GADEN(, pubs.pmfsPubs.groundTruthWind));
                                 stateMachine.forceSetState(stopAndMeasureState.get());
#if DEBUG_VISUALIZATION
                                 logProgressionAndVisualize();
#endif
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

        for (size_t i = 0; i < sourceProbSemantics.size(); i++)
            sourceProbSemantics[i] = 0;

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

    void SemanticPMFS::logProgressionAndVisualize()
    {
        GSL_INFO("Logging progression to file '{}'", getParam<std::string>("progressionFileName", "progression.csv"));
        updateSourceFromSemantics();
        std::vector<ColorRGBA> colors(sourceProbabilityPMFS.size());
        for (int i = 0; i < sourceProbabilityPMFS.size(); i++)
            colors[i] = Utils::valueToColor(sourceProbabilityPMFS[i],
                                            settings.visualization.sourceLimits.x,
                                            settings.visualization.sourceLimits.y,
                                            settings.visualization.sourceMode);
        Utils::publishDebugMarkers(Grid2D<ColorRGBA>(colors, simulationOccupancy, gridMetadata), "sourceOlfactionOnly");

        Vector2 expecOlfOnly = Utils::ExpectedValue(AsGrid(sourceProbabilityPMFS, simulationOccupancy), 1);
        Utils::CovarianceMatrix varOlfOnly = Utils::Covariance(AsGrid(sourceProbabilityPMFS, simulationOccupancy));
        double errorOlfOnly = vmath::length(expecOlfOnly - resultLogging.sourcePositionGT);

        Vector2 expecBoth = Utils::ExpectedValue(AsGrid(combinedSourceProbability, simulationOccupancy), 1);
        Utils::CovarianceMatrix varBoth = Utils::Covariance(AsGrid(combinedSourceProbability, simulationOccupancy));
        double errorBoth = vmath::length(expecBoth - resultLogging.sourcePositionGT);
        progressionFile << fmt::format("{:.2f}; ({:.2f}, {:.2f}, {:.2f});  {:.2f}; ({:.2f}, {:.2f}, {:.2f});\n",
                                       errorOlfOnly, varOlfOnly.x, varOlfOnly.y, varOlfOnly.covariance,
                                       errorBoth, varBoth.x, varBoth.y, varBoth.covariance);
        progressionFile.flush();

        Utils::publishDebugSingleMarker(vmath::WithZ(expecOlfOnly, 0.0),
                                        Utils::create_color(1, 0, 0, 1),
                                        "EXPECTED_OLFACTION");
        Utils::publishDebugSingleMarker(vmath::WithZ(expecBoth, 0.0),
                                        Utils::create_color(1, 0, 1, 1),
                                        "EXPECTED_BOTH");
    }

    void SemanticPMFS::processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection)
    {
        static int number_of_updates = 0;

        Grid<HitProbability> grid(hitProbability, simulationOccupancy, gridMetadata);
        if (concentration > thresholdGas)
        {
            // Gas & wind
            PMFSLib::EstimateHitProbabilities(grid, *visibilityMap, settings.hitProbability, true, windDirection, windSpeed,
                                              gridMetadata.coordinatesToIndices(currentRobotPose.pose.pose));
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "GAS HIT");
        }
        else
        {
            // Nothing
            PMFSLib::EstimateHitProbabilities(grid, *visibilityMap, settings.hitProbability, false, windDirection, windSpeed,
                                              gridMetadata.coordinatesToIndices(currentRobotPose.pose.pose));
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "NOTHING ");
        }

        number_of_updates++;

        if (number_of_updates >= settings.hitProbability.maxUpdatesPerStop)
        {
            PMFSLib::EstimateWind(
                settings.simulation.useWindGroundTruth,
                AsGrid(estimatedWindVectors, simulationOccupancy),
                node,
                pubs.pmfsPubs.gmrfWind
                    IF_GADEN(, pubs.pmfsPubs.groundTruthWind));
            PMFSViz::PlotWindVectors(
                AsGrid(estimatedWindVectors, simulationOccupancy),
                settings.visualization,
                pubs.pmfsPubs);

            number_of_updates = 0;
            bool timeToSimulate = iterationsCounter >= settings.movement.initialExplorationMoves &&
                                  iterationsCounter % settings.simulation.stepsBetweenSourceUpdates == 0;

            //             if (timeToSimulate)
            //             {
            //                 simulations.updateSourceProbability(settings.simulation.refineFraction);
            // #if DEBUG_VISUALIZATION
            //                 logProgressionAndVisualize();
            // #endif
            //             }

            movingState->chooseGoalAndMove();
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

        iterationsCounter++;
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

        double variance = Utils::Variance(Grid2D<double>(combinedSourceProbability, simulationOccupancy, gridMetadata));
        GSL_INFO("Variance: {:.2f}", variance);

        if (variance < settings.declaration.threshold)
        {
            saveResultsToFile(GSLResult::Success);
            return GSLResult::Success;
        }

        return GSLResult::Running;
    }

    void SemanticPMFS::saveResultsToFile(GSLResult result)
    {
        progressionFile.close();

        auto grid = AsGrid(combinedSourceProbability, simulationOccupancy);
        // 1. Search time.
        rclcpp::Duration time_spent = node->now() - startTime;
        double search_t = time_spent.seconds();

        Vector2 sourceLocationAll = Utils::ExpectedValue(grid, 1);
        Vector2 sourceLocation = Utils::ExpectedValue(grid, 0.05);

        double error = sqrt(pow(resultLogging.sourcePositionGT.x - sourceLocation.x, 2) + pow(resultLogging.sourcePositionGT.y - sourceLocation.y, 2));
        double errorAll = sqrt(pow(resultLogging.sourcePositionGT.x - sourceLocationAll.x, 2) + pow(resultLogging.sourcePositionGT.y - sourceLocationAll.y, 2));

        std::string resultString = fmt::format("RESULT IS: Success={}, Search_t={:.2f}, Error={:.2f}", (int)result, search_t, error);
        GSL_INFO_COLOR(fmt::terminal_color::blue, "{}", resultString);

        // Save to file
        if (resultLogging.resultsFile != "")
        {
            std::ofstream file;
            file.open(resultLogging.resultsFile, std::ios_base::app);
            if (result != GSLResult::Success)
                file << "FAILED ";

            file << resultLogging.navigationTime << " " << search_t << " " << errorAll << " " << error << " " << iterationsCounter << " "
                 << Utils::Variance(AsGrid(combinedSourceProbability, simulationOccupancy)) << "\n";
            file.close();
        }
        else
            GSL_WARN("No file provided for logging result. Skipping it.");

        if (resultLogging.navigationPathFile != "")
        {
            std::ofstream file;
            file.open(resultLogging.navigationPathFile, std::ios_base::app);
            file << "------------------------\n";
            for (PoseWithCovarianceStamped p : resultLogging.robotPosesVector)
                file << p.pose.pose.position.x << ", " << p.pose.pose.position.y << "\n";
            file.close();
        }
        else
            GSL_WARN("No file provided for logging path. Skipping it.");
    }

} // namespace GSL