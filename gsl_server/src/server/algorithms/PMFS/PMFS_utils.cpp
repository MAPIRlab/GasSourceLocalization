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

    // this will be overriden when there are several variables affecting the final probability
    double PMFS::sourceProbability(int i, int j)
    {
        return grid[i][j].sourceProbability;
    }

    bool PMFS::indicesInBounds(const Vector2Int indices) const
    {
        return indices.x >= 0 && indices.x < grid.size() && indices.y >= 0 && indices.y < grid[0].size();
    }

    bool PMFS::pathFree(const Vector2Int& origin, const Vector2Int& end)
    {
        // check there are no obstacles between origin and end
        if (!(grid[origin.x][origin.y].free && grid[end.x][end.y].free))
            return false;

        bool pathIsFree = true;
        Vector2 vector = gridData.indexToCoordinates(end.x, end.y) - gridData.indexToCoordinates(origin.x, origin.y);
        Vector2 increment = glm::normalize(vector) * (gridData.cellSize);
        int steps = glm::length(vector) / (gridData.cellSize);
        int index = 0;
        Vector2 current_point = gridData.indexToCoordinates(origin.x, origin.y);
        while (index < steps && pathIsFree)
        {
            current_point += increment;
            index++;
            Vector2Int pair = gridData.coordinatesToIndex(current_point.x, current_point.y);
            pathIsFree = indicesInBounds(pair) && grid[pair.x][pair.y].free;
        }

        return pathIsFree;
    }

    void PMFS::normalizeSourceProb(std::vector<std::vector<Cell>>& variable)
    {
        // we account for the possibility of having positive and negative values by offsetting everything by the value of the minimum (capped at 0)
        // so, [-1, -0.5, 1, 2] would become [0, 0.5, 2, 3] before the normalization happens
        double total = 0;
        int count = 0;
        for (int i = 0; i < variable.size(); i++)
        {
            for (int j = 0; j < variable[0].size(); j++)
            {
                if (grid[i][j].free)
                {
                    total += variable[i][j].sourceProbability;
                    count++;
                }
            }
        }

#pragma omp parallel for collapse(2)
        for (int i = 0; i < variable.size(); i++)
        {
            for (int j = 0; j < variable[0].size(); j++)
            {
                if (grid[i][j].free)
                {
                    variable[i][j].sourceProbability = (variable[i][j].sourceProbability) / total;
                }
            }
        }
    }

    void PMFS::estimateWind(bool groundTruth)
    {
        // if not compiled with gaden support, you have no choice but to use GMRF :)
#ifdef USE_GADEN
        if (!groundTruth)
        {
#endif
            auto& clientWindGMRF = pubs.clientWindGMRF;
            auto& GMRFRequest = pubs.GMRFRequest;
            // ask the gmrf_wind service for the estimated wind vector in cell i,j
            auto future = clientWindGMRF->async_send_request(GMRFRequest);
            auto result = rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(5));

            if (result == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = future.get();
                for (int ind = 0; ind < GMRFRequest->x.size(); ind++)
                {
                    Vector2Int pair = gridData.coordinatesToIndex(GMRFRequest->x[ind], GMRFRequest->y[ind]);
                    estimatedWindVectors[pair.x][pair.y] = Vector2(std::cos(response->v[ind]), std::sin(response->v[ind])) * (float)response->u[ind];
                }
            }
            else
                GSL_WARN("CANNOT READ ESTIMATED WIND VECTORS");

#ifdef USE_GADEN
        }

        else
        {

            auto& clientWindGroundTruth = pubs.clientWindGroundTruth;
            auto& groundTruthWindRequest = pubs.groundTruthWindRequest;

            auto future = clientWindGroundTruth->async_send_request(groundTruthWindRequest);
            auto result = rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(5));
            if (result == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = future.get();
                for (int ind = 0; ind < groundTruthWindRequest->x.size(); ind++)
                {
                    Vector2Int pair = gridData.coordinatesToIndex(groundTruthWindRequest->x[ind], groundTruthWindRequest->y[ind]);
                    estimatedWindVectors[pair.x][pair.y] = Vector2(response->u[ind], response->v[ind]);
                }
            }
            else
                GSL_WARN("CANNOT READ ESTIMATED WIND VECTORS");
        }
#endif
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
            debugCreateMapState->printHitmap();
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
        for (int i = 0; i < grid.size(); i++)
        {
            for (int j = 0; j < grid[0].size(); j++)
            {
                if (grid[i][j].free)
                {
                    CellData cd(Vector2Int(i, j), sourceProbability(i, j));
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
            Vector2 coord = gridData.indexToCoordinates(cd.indices.x, cd.indices.y);
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
        for (int i = 0; i < grid.size(); i++)
        {
            for (int j = 0; j < grid[0].size(); j++)
            {
                if (grid[i][j].free)
                {
                    Vector2 coords = gridData.indexToCoordinates(i, j);
                    double p = sourceProbability(i, j);
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