#include "SemanticGrGSL.hpp"
#include "gsl_server/algorithms/GrGSL/MovingStateGrGSL.hpp"
#include "gsl_server/algorithms/Common/States/ManualNavigation.hpp"
#include "gsl_server/algorithms/Common/States/NoNavigation.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"
#include "gsl_server/algorithms/Common/Utils/RosUtils.hpp"
#include "gsl_server/algorithms/Semantics/Semantics/Common/ResultsLogging.hpp"
#include "gsl_server/algorithms/Semantics/Semantics/Common/SemanticsType.hpp"
#include <fstream>
#include <gsl_server/algorithms/GrGSL/GrGSLLib.hpp>

namespace GSL
{
    using namespace GrGSL_internal;
    void SemanticGrGSL::OnUpdate()
    {
        Algorithm::OnUpdate();

        if (semantics) // TODO allow this to run slower that the update loop? kinda messes up the callback-based one
        {
            semantics->OnUpdate();
            updateSourceFromSemantics();
        }

        GrGSLLib::VisualizeMarkers(
            Grid2D<double>(combinedSourceProbability, simulationOccupancy, gridMetadata),
            markers,
            node,
            settings.colorScaleLimits);
    }

    void SemanticGrGSL::Initialize()
    {
        Algorithm::Initialize();

        markers.probabilityMarkers = node->create_publisher<Marker>("probabilityMarkers", 10);
        markers.estimationMarkers = node->create_publisher<Marker>("estimationMarkers", 10);

        exploredCells = 0;

        waitForMapState = std::make_unique<WaitForMapState>(this);
        waitForGasState = std::make_unique<WaitForGasState>(this);
        stopAndMeasureState = std::make_unique<StopAndMeasureState>(this);

#if DISABLE_NAVIGATION
        movingState = std::make_unique<NoNavigationState>(this);
#else
        movingState = std::make_unique<MovingStateGrGSL>(this,
                                                         GrGSLData{
                                                             .node = node,
                                                             .settings = settings,
                                                             .cells = cells,
                                                             .occupancy = navigationOccupancy,
                                                             .gridMetadata = gridMetadata,
                                                             .currentRobotPosition = currentRobotPosition,
                                                             .positionOfLastHit = positionOfLastHit});
#endif
        stateMachine.forceSetState(waitForMapState.get());

        std::string progresionFileName = getParam<std::string>("progressionFileName", "progression.csv");
        SemanticsResults::InitFile(progresionFileName, resultLogging.sourcePositionGT);
        IF_GUI(ui.run());
    }

    void SemanticGrGSL::declareParameters()
    {
        Algorithm::declareParameters();
        GrGSLLib::GetSettings(node, settings, markers);
    }

    void SemanticGrGSL::onGetMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        Algorithm::onGetMap(msg);
        GrGSLLib::initMetadata(gridMetadata, map, Utils::getParam(node, "scale", 20));
        cells.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y);
        navigationOccupancy.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y);
        combinedSourceProbability.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y);
        simulationOccupancy = Utils::parseMapImage(getParam<std::string>("wallsOccupancyFile", "?"), gridMetadata);

        GridUtils::reduceOccupancyMap(map.data, map.info.width, navigationOccupancy, gridMetadata);
        GrGSLLib::initializeMap(*this,
                                Grid2D<Cell>(cells, simulationOccupancy, gridMetadata));
        positionOfLastHit = Vector2(currentRobotPose.pose.pose.position.x, currentRobotPose.pose.pose.position.y);

        // SEMANTICS
        //----------------------
        std::string semanticsTypeParam = node->declare_parameter<std::string>("semanticsType", "ClassMap2D");
        SemanticsType semanticsType = ParseSemanticsType(semanticsTypeParam);

        if (semanticsType == SemanticsType::ClassMap2D)
            createClassMap2D();
        else if (semanticsType == SemanticsType::ClassMapVoxeland)
            createClassMapVoxeland();

        stateMachine.forceSetState(stopAndMeasureState.get());
    }

    void SemanticGrGSL::processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection)
    {
        bool gasHit = concentration > thresholdGas;
        bool significantWind = windSpeed > thresholdWind;

        if (gasHit)
            positionOfLastHit = Vector2(currentRobotPose.pose.pose.position.x, currentRobotPose.pose.pose.position.y);

        if (gasHit && significantWind)
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "GAS HIT");
        else if (gasHit)
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "GAS BUT NO WIND");
        else if (significantWind)
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "ONLY WIND");
        else
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "NOTHING");

        GrGSLLib::estimateProbabilitiesfromGasAndWind(
            Grid2D<Cell>(cells, simulationOccupancy, gridMetadata),
            settings,
            gasHit,
            gasHit ? significantWind : true,
            windDirection,
            positionOfLastHit,
            gridMetadata.coordinatesToIndices(currentRobotPose.pose.pose));

        movingState->chooseGoalAndMove();
        logProgressionAndVisualize();
        exploredCells++;
    }

    void SemanticGrGSL::logProgressionAndVisualize()
    {
        GSL_INFO("Logging progression to file '{}'", getParam<std::string>("progressionFileName", "progression.csv"));
        updateSourceFromSemantics();
        std::vector<double> sourceProbabilityGrGSL(cells.size());
        std::transform(cells.begin(),  cells.end(), sourceProbabilityGrGSL.begin(), [](const Cell& cell){return cell.sourceProb;});

        
        SemanticsResults::LogResult(AsGrid(sourceProbabilityGrGSL, simulationOccupancy),
                                    AsGrid(combinedSourceProbability, simulationOccupancy),
                                    resultLogging.sourcePositionGT,
                                    settings.colorScaleLimits);
    }

    void SemanticGrGSL::updateSourceFromSemantics()
    {
        // wait until we have received the map and initialized everything
        if (!semantics)
            return;

        std::vector<double> sourceProbSemantics = semantics->GetSourceProbability();
#pragma omp parallel for
        for (size_t i = 0; i < sourceProbSemantics.size(); i++)
        {
            combinedSourceProbability[i] = cells[i].sourceProb * sourceProbSemantics[i];
        }
        Utils::NormalizeDistribution(combinedSourceProbability, simulationOccupancy);
    }

    GSLResult SemanticGrGSL::checkSourceFound()
    {
        if (stateMachine.getCurrentState() == waitForMapState.get() || stateMachine.getCurrentState() == waitForGasState.get())
            return GSLResult::Running;
        Grid2D<double> grid(combinedSourceProbability, simulationOccupancy, gridMetadata);
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

        double variance = Utils::Variance(grid);
        GSL_INFO("Variance: {}", variance);

        if (variance < settings.convergence_thr)
        {
            saveResultsToFile(GSLResult::Success);
            return GSLResult::Success;
        }

        return GSLResult::Running;
    }

    void SemanticGrGSL::saveResultsToFile(GSLResult result)
    {
        Grid2D<double> grid(combinedSourceProbability, simulationOccupancy, gridMetadata);
        // 1. Search time.
        rclcpp::Duration time_spent = node->now() - startTime;
        double search_t = time_spent.seconds();

        Vector2 sourceLocation = Utils::ExpectedValue(grid, 1);

        double error = sqrt(pow(resultLogging.sourcePositionGT.x - sourceLocation.x, 2) + pow(resultLogging.sourcePositionGT.y - sourceLocation.y, 2));

        std::string resultString = fmt::format("RESULT IS: Success={}, Search_t={:.2f}, Error={:.2f}", (int)result, search_t, error);
        GSL_INFO_COLOR(fmt::terminal_color::blue, "{}", resultString);

        // Save to file
        if (resultLogging.resultsFile != "")
        {
            std::ofstream file;
            file.open(resultLogging.resultsFile, std::ios_base::app);
            if (result != GSLResult::Success)
                file << "FAILED ";

            file << fmt::format("nav time:{:.2f},\terror:{:.2f},\titerations:{},var:{:.2f}\n",
                                search_t,
                                error,
                                exploredCells,
                                Utils::Variance(grid));
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
