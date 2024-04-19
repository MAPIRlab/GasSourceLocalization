#include <gsl_server/algorithms/PMFS/PMFS.hpp>
#include <fstream>

namespace GSL
{
    void PMFS::OnCompleteNavigation(GSLResult result, State* previousState)
    {
        if (result == GSLResult::Success)
            stateMachine.forceSetState(stopAndMeasureState.get());
        else
        {
            auto m = dynamic_cast<MovingStatePMFS*>(movingState.get());
            functionQueue.submit(std::bind(&MovingStatePMFS::chooseGoalAndMove, m));
        }
    }


    GSLResult PMFS::checkSourceFound()
    {
        if (stateMachine.getCurrentState() == waitForMapState.get())
            return GSLResult::Running;

        rclcpp::Duration time_spent = node->now() - start_time;
        if (time_spent.seconds() > resultLogging.max_search_time)
        {
            Vector2 locationAll = expectedValueSource(1);
            Vector2 location = expectedValueSource(0.05);
            saveResultsToFile(GSLResult::Failure);
            return GSLResult::Failure;
        }

        if (resultLogging.navigationTime == -1)
        {
            if (sqrt(pow(currentRobotPose.pose.pose.position.x - resultLogging.source_pose.x, 2) +
                     pow(currentRobotPose.pose.pose.position.y - resultLogging.source_pose.y, 2)) < 0.5)
            {
                resultLogging.navigationTime = time_spent.seconds();
            }
        }

        double variance = varianceSourcePosition();
        GSL_INFO("Variance: {}", variance);

        Vector2 locationAll = expectedValueSource(1);

        Vector2 location = expectedValueSource(0.05);

        if (variance < settings.declaration.threshold)
        {
            saveResultsToFile(GSLResult::Success);
            return GSLResult::Success;
        }

        return GSLResult::Running;
    }

    Vector2 PMFS::expectedValueSource(double proportionBest)
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
        for (int i = 0; i < gridMetadata.height; i++)
        {
            for (int j = 0; j < gridMetadata.width; j++)
            {
                if (occupancy[gridMetadata.indexOf(i,j)] == Occupancy::Free)
                {
                    CellData cd(Vector2Int(i, j), sourceProbability[gridMetadata.indexOf(i, j)]);
                    data.push_back(cd);
                }
            }
        }

        std::sort(data.begin(), data.end(), [](const CellData& a, const CellData& b) { return a.probability > b.probability; });

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

    double PMFS::varianceSourcePosition()
    {
        Vector2 expected = expectedValueSource(1);
        double x = 0, y = 0;
        for (int i = 0; i < gridMetadata.height; i++)
        {
            for (int j = 0; j < gridMetadata.width; j++)
            {
                if (occupancy[gridMetadata.indexOf(i,j)] == Occupancy::Free)
                {
                    Vector2 coords = gridMetadata.indexToCoordinates(i, j);
                    double p = sourceProbability[gridMetadata.indexOf(i, j)];
                    x += pow(coords.x - expected.x, 2) * p;
                    y += pow(coords.y - expected.y, 2) * p;
                }
            }
        }
        return x + y;
    }

    void PMFS::saveResultsToFile(GSLResult result)
    {
        // 1. Search time.
        rclcpp::Duration time_spent = node->now() - start_time;
        double search_t = time_spent.seconds();

        Vector2 sourceLocationAll = expectedValueSource(1);
        Vector2 sourceLocation = expectedValueSource(0.05);

        double error = sqrt(pow(resultLogging.source_pose.x - sourceLocation.x, 2) + pow(resultLogging.source_pose.y - sourceLocation.y, 2));
        double errorAll = sqrt(pow(resultLogging.source_pose.x - sourceLocationAll.x, 2) + pow(resultLogging.source_pose.y - sourceLocationAll.y, 2));
        
		std::string resultString = fmt::format("RESULT IS: Success={}, Search_t={:.2f}, Error={:.2f}", (int)result, search_t, error);
        GSL_INFO_COLOR(fmt::terminal_color::blue, "{}", resultString);

        // Save to file
        if (resultLogging.results_file != "")
        {
            std::ofstream file;
            file.open(resultLogging.results_file, std::ios_base::app);
            if (result != GSLResult::Success)
                file << "FAILED ";

            file << resultLogging.navigationTime << " " << search_t << " " << errorAll << " " << error << " " << iterationsCounter << " "
                 << varianceSourcePosition() << "\n";
            file.close();
        }
        else
            GSL_WARN("No file provided for logging result. Skipping it.");

        if (resultLogging.path_file != "")
        {
            std::ofstream file;
            file.open(resultLogging.path_file, std::ios_base::app);
            file << "------------------------\n";
            for (PoseWithCovarianceStamped p : resultLogging.robot_poses_vector)
                file << p.pose.pose.position.x << ", " << p.pose.pose.position.y << "\n";
            file.close();
        }
        else
            GSL_WARN("No file provided for logging path. Skipping it.");
    }

} // namespace GSL