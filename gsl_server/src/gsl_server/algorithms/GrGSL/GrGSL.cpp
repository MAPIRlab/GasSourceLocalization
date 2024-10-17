#include <gsl_server/algorithms/GrGSL/GrGSL.hpp>
#include <gsl_server/algorithms/GrGSL/MovingStateGrGSL.hpp>
#include <gsl_server/algorithms/GrGSL/GrGSLLib.hpp>
#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>
#include <angles/angles.h>
#include <fstream>

namespace GSL
{
    using namespace GrGSL_internal;

    void GrGSL::Initialize()
    {
        Algorithm::Initialize();

        markers.probabilityMarkers = node->create_publisher<Marker>("probabilityMarkers", 10);
        markers.estimationMarkers = node->create_publisher<Marker>("estimationMarkers", 10);

        exploredCells = 0;

        waitForMapState = std::make_unique<WaitForMapState>(this);
        waitForGasState = std::make_unique<WaitForGasState>(this);
        stopAndMeasureState = std::make_unique<StopAndMeasureState>(this);
        movingState = std::make_unique<MovingStateGrGSL>(this);
        stateMachine.forceSetState(waitForMapState.get());
    }

    void GrGSL::declareParameters()
    {
        Algorithm::declareParameters();
        GrGSLLib::GetSettings(node, settings, markers);
    }

    void GrGSL::OnUpdate()
    {
        Algorithm::OnUpdate();
        functionQueue.run();
    }

    void GrGSL::onGetMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        Algorithm::onGetMap(msg);
        GrGSLLib::initMetadata(gridMetadata, map, Utils::getParam(node, "scale", 20));
        cells.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y);
        occupancy.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y);


        GridUtils::reduceOccupancyMap(map.data, map.info.width, occupancy, gridMetadata);
        GrGSLLib::initializeMap(*this,
                                Grid2D<Cell>(cells, occupancy, gridMetadata));
        positionOfLastHit = {currentRobotPose.pose.pose.position.x, currentRobotPose.pose.pose.position.y};
    }



    void GrGSL::processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection)
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
            Grid2D<Cell>(cells, occupancy, gridMetadata),
            settings,
            gasHit,
            gasHit ? significantWind : true,
            windDirection,
            positionOfLastHit,
            gridMetadata.coordinatesToIndices(currentRobotPose.pose.pose));

        dynamic_cast<MovingStateGrGSL*>(movingState.get())->chooseGoalAndMove();
        GrGSLLib::VisualizeMarkers(
            Grid2D<Cell>(cells, occupancy, gridMetadata),
            markers,
            node
        );
    }

    double GrGSL::probability(const Vector2Int& indices) const
    {
        size_t index = gridMetadata.indexOf(indices);
        return cells[index].sourceProb;
    }

    GSLResult GrGSL::checkSourceFound()
    {
        if (stateMachine.getCurrentState() == waitForMapState.get() || stateMachine.getCurrentState() == waitForGasState.get())
            return GSLResult::Running;
        Grid2D<Cell> grid(cells, occupancy, gridMetadata);
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

        double variance = GrGSLLib::varianceSourcePosition(grid);
        GSL_INFO("Variance: {}", variance);

        if (variance < settings.convergence_thr)
        {
            saveResultsToFile(GSLResult::Success);
            return GSLResult::Success;
        }

        return GSLResult::Running;
    }

    void GrGSL::saveResultsToFile(GSLResult result)
    {
        Grid2D<Cell> grid(cells, occupancy, gridMetadata);
        // 1. Search time.
        rclcpp::Duration time_spent = node->now() - startTime;
        double search_t = time_spent.seconds();


        Vector2 sourceLocationAll = GrGSLLib::expectedValueSource(grid,1);
        Vector2 sourceLocation = GrGSLLib::expectedValueSource(grid,0.05);

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

            file << resultLogging.navigationTime << " " << search_t << " " << errorAll << " " << error << " " << exploredCells << " "
                 << GrGSLLib::varianceSourcePosition(grid) << "\n";
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