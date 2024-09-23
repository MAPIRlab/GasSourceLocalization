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
        settings.useDiffusionTerm = getParam<bool>("useDiffusionTerm", false); //experimental way to extract useful info from low-wind measurements, not very reliable
        settings.stdevHit = getParam<double>("stdevHit", 1.0);
        settings.stdevMiss = getParam<double>("stdevMiss", 2.0);
        settings.infoTaxis = getParam<bool>("infoTaxis", false);
        settings.allowMovementRepetition = getParam<bool>("allowMovementRepetition", true);
        settings.convergence_thr = getParam<double>("convergence_thr", 0.5); // threshold for source declaration
        markers.markersHeight = getParam<float>("markers_height", 0);
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
        grid.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y);
        occupancy.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y);
        GrGSLLib::initializeMap(*this,
                                Grid2D<Cell>(grid, occupancy, gridMetadata));
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
            Grid2D<Cell>(grid, occupancy, gridMetadata),
            settings,
            gasHit,
            gasHit ? significantWind : true,
            windDirection,
            positionOfLastHit,
            gridMetadata.coordinatesToIndex(currentRobotPose.pose.pose));

        dynamic_cast<MovingStateGrGSL*>(movingState.get())->chooseGoalAndMove();
        GrGSLLib::VisualizeMarkers(
            Grid2D<Cell>(grid, occupancy, gridMetadata),
            markers,
            node
        );
    }

    double GrGSL::informationGain(const WindVector& windVec)
    {
        auto predictionCells = grid; // temp copy of the matrix of cells that we can modify to simulate the effect of a measurement
        auto accessProb = [](const Cell & cell)
                          {
                              return cell.sourceProb;
                          };

        //simulate a hit in the considered position and see how much info that gives us
        GrGSLLib::estimateProbabilitiesfromGasAndWind(
            Grid2D<Cell>(predictionCells, occupancy, gridMetadata),
            settings,
            true,
            true,
            windVec.angle,
            positionOfLastHit,
            Vector2Int(windVec.col, windVec.row));
        double infoHit = Utils::KLD<Cell>(predictionCells, grid, occupancy, accessProb);

        //simulate a miss and repeat
        predictionCells = grid;
        GrGSLLib::estimateProbabilitiesfromGasAndWind(
            Grid2D<Cell>(predictionCells, occupancy, gridMetadata),
            settings,
            false,
            true,
            windVec.angle,
            positionOfLastHit,
            Vector2Int(windVec.col, windVec.row));
        double infoMiss = Utils::KLD<Cell>(predictionCells, grid, occupancy, accessProb);

        //expected value of info, considering that the probability of a hit can be equated to the currently estimated source prob... which is a hack
        size_t index = gridMetadata.indexOf({windVec.col, windVec.row});
        return grid[index].sourceProb * infoHit + (1 - grid[index].sourceProb) * infoMiss;
    }

    double GrGSL::probability(const Vector2Int& indices) const
    {
        size_t index = gridMetadata.indexOf(indices);
        return grid[index].sourceProb;
    }

    GSLResult GrGSL::checkSourceFound()
    {
        if (stateMachine.getCurrentState() == waitForMapState.get() || stateMachine.getCurrentState() == waitForGasState.get())
            return GSLResult::Running;

        rclcpp::Duration time_spent = node->now() - startTime;
        if (time_spent.seconds() > resultLogging.maxSearchTime)
        {
            Vector2 locationAll = expectedValueSource(1);
            Vector2 location = expectedValueSource(0.05);
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

        double variance = varianceSourcePosition();
        GSL_INFO("Variance: {}", variance);

        Vector2 locationAll = expectedValueSource(1);

        Vector2 location = expectedValueSource(0.05);

        if (variance < settings.convergence_thr)
        {
            saveResultsToFile(GSLResult::Success);
            return GSLResult::Success;
        }

        return GSLResult::Running;
    }

    Vector2 GrGSL::expectedValueSource(double proportionBest)
    {
        struct CellData
        {
            Vector2Int indices;
            double probability;
            CellData(Vector2Int ind, double prob)
            {
                indices = ind;
                probability = prob;
            }
        };
        std::vector<CellData> data;
        for (int i = 0; i < grid.size(); i++)
        {
            if (occupancy[i] == Occupancy::Free)
            {
                CellData cd(gridMetadata.indices2D(i), grid[i].sourceProb);
                data.push_back(cd);
            }
        }

        std::sort(data.begin(), data.end(), [](const CellData & a, const CellData & b)
                  {
                      return a.probability > b.probability;
                  });

        double averageX = 0, averageY = 0;
        double sum = 0;

        for (int i = 0; i < data.size() * proportionBest; i++)
        {
            CellData& cd = data[i];
            Vector2 coord = gridMetadata.indexToCoordinates(cd.indices.x, cd.indices.y);
            averageX += cd.probability * coord.x;
            averageY += cd.probability * coord.y;
            sum += cd.probability;
        }
        return Vector2(averageX / sum, averageY / sum);
    }

    double GrGSL::varianceSourcePosition()
    {
        Vector2 expected = expectedValueSource(1);
        double x = 0, y = 0;
        for (int i = 0; i < grid.size(); i++)
        {
            if (occupancy[i] == Occupancy::Free)
            {
                Vector2Int indices = gridMetadata.indices2D(i);
                Vector2 coords = gridMetadata.indexToCoordinates(indices);
                double p = probability(indices);
                x += std::pow(coords.x - expected.x, 2) * p;
                y += std::pow(coords.y - expected.y, 2) * p;
            }
        }
        return x + y;
    }

    void GrGSL::saveResultsToFile(GSLResult result)
    {
        // 1. Search time.
        rclcpp::Duration time_spent = node->now() - startTime;
        double search_t = time_spent.seconds();


        Vector2 sourceLocationAll = expectedValueSource(1);
        Vector2 sourceLocation = expectedValueSource(0.05);

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
                 << varianceSourcePosition() << "\n";
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