#include <algorithms/PMFS/PMFS.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
namespace PMFS
{

    // this will be overriden when there are several variables affecting the final probability
    double PMFS_GSL::sourceProbability(int i, int j)
    {
        return cells[i][j].sourceProbability;
    }

    State PMFS_GSL::getState()
    {
        return current_state;
    }

    Vector2Int PMFS_GSL::coordinatesToIndex(double x, double y) const
    {
        return Vector2Int((y - map_.info.origin.position.y) / (settings.scale * map_.info.resolution),
                          (x - map_.info.origin.position.x) / (settings.scale * map_.info.resolution));
    }

    Utils::Vector2 PMFS_GSL::indexToCoordinates(int i, int j, bool centerOfCell) const
    {
        float offset = centerOfCell ? 0.5 : 0;
        return Utils::Vector2(map_.info.origin.position.x + (j + offset) * settings.scale * map_.info.resolution,
                              map_.info.origin.position.y + (i + offset) * settings.scale * map_.info.resolution);
    }

    bool PMFS_GSL::indicesInBounds(const Vector2Int indices) const
    {
        return indices.x >= 0 && indices.x < cells.size() && indices.y >= 0 && indices.y < cells[0].size();
    }

    Utils::Vector2 PMFS_GSL::expectedValueSource()
    {
        double averageX = 0, averageY = 0;
        double sum = 0;
        for (int i = 0; i < cells.size(); i++)
        {
            for (int j = 0; j < cells[0].size(); j++)
            {
                if (cells[i][j].free)
                {
                    double p = sourceProbability(i, j);
                    Utils::Vector2 coord = indexToCoordinates(i, j);
                    averageX += p * coord.x;
                    averageY += p * coord.y;
                    sum += p;
                }
            }
        }
        return Utils::Vector2(averageX / sum, averageY / sum);
    }

    Utils::Vector2 PMFS_GSL::expectedValueSource(double proportionBest)
    {

        std::vector<CellData> data;
        for (int i = 0; i < cells.size(); i++)
        {
            for (int j = 0; j < cells[0].size(); j++)
            {
                if (cells[i][j].free)
                {
                    CellData cd(Vector2Int(i, j), sourceProbability(i, j));
                    data.push_back(cd);
                }
            }
        }

        std::sort(data.begin(), data.end(), [](CellData a, CellData b) { return a.probability > b.probability; });

        double averageX = 0, averageY = 0;
        double sum = 0;

        for (int i = 0; i < data.size() * proportionBest; i++)
        {
            CellData& cd = data[i];
            Utils::Vector2 coord = indexToCoordinates(cd.indices.x, cd.indices.y);
            averageX += cd.probability * coord.x;
            averageY += cd.probability * coord.y;
            sum += cd.probability;
        }
        return Utils::Vector2(averageX / sum, averageY / sum);
    }

    double PMFS_GSL::varianceSourcePosition()
    {
        Utils::Vector2 expected = expectedValueSource();
        double x = 0, y = 0;
        for (int i = 0; i < cells.size(); i++)
        {
            for (int j = 0; j < cells[0].size(); j++)
            {
                if (cells[i][j].free)
                {
                    Utils::Vector2 coords = indexToCoordinates(i, j);
                    double p = sourceProbability(i, j);
                    x += pow(coords.x - expected.x, 2) * p;
                    y += pow(coords.y - expected.y, 2) * p;
                }
            }
        }
        return x + y;
    }

    void PMFS_GSL::submitToMainThread(const std::function<void()>& func)
    {
        mainThreadQueueMtx.lock();
        mainThreadExecQueue.push_back(func);
        mainThreadQueueMtx.unlock();
    }

    void PMFS_GSL::runSubmitedQueue()
    {
        mainThreadQueueMtx.lock();
        auto queueCopy = mainThreadExecQueue;
        mainThreadExecQueue.clear();
        mainThreadQueueMtx.unlock();

        for (auto& func : queueCopy)
        {
            func();
        }
    }

    int PMFS_GSL::checkSourceFound()
    {

        if (current_state != State::WAITING_FOR_MAP)
        {
            rclcpp::Duration time_spent = node->now() - start_time;
            if (time_spent.seconds() > max_search_time)
            {
                Utils::Vector2 locationAll = expectedValueSource();
                Utils::Vector2 location = expectedValueSource(0.05);
                save_results_to_file(0, location.x, location.y, locationAll.x, locationAll.y);
                return 0;
            }

            if (!reached)
            {
                if (std::sqrt(std::pow(current_robot_pose.pose.pose.position.x - ground_truth_x, 2) +
                              std::pow(current_robot_pose.pose.pose.position.y - ground_truth_y, 2)) < 0.5)
                {
                    t1 = time_spent.seconds();
                    reached = true;
                }
            }

            double variance = varianceSourcePosition();
            spdlog::info("Variance: {:.3}", variance);
            Utils::Vector2 locationAll = expectedValueSource();
            bool converged = false;
            if (settings.declaration.mode == GridSettings::DeclarationSettings::Stability)
            {
                if (estimatedSourceLocations.size() >= settings.declaration.steps)
                {
                    converged = true;
                    for (int i = 0; i < estimatedSourceLocations.size(); i++)
                    {
                        double distance = (locationAll - estimatedSourceLocations[i]).norm();
                        if (distance > settings.declaration.threshold)
                        {
                            converged = false;
                        }
                    }
                    estimatedSourceLocations.pop_front();
                }
            }
            else if (settings.declaration.mode == GridSettings::DeclarationSettings::Variance)
            {
                if (variance < settings.declaration.threshold)
                    converged = true;
            }

            Utils::Vector2 location = expectedValueSource(0.05);
            estimatedSourceLocations.push_back(locationAll);
            errorOverTimeAll.push_back({(locationAll - Utils::Vector2(source_pose_x, source_pose_y)).norm(), static_cast<float>(variance)});
            errorOverTime.push_back({(location - Utils::Vector2(source_pose_x, source_pose_y)).norm(), static_cast<float>(variance)});

            if (converged)
            {
                save_results_to_file(1, location.x, location.y, locationAll.x, locationAll.y);
                return 1;
            }
        }
        return -1;
    }

    bool PMFS_GSL::pathFree(const Vector2Int& origin, const Vector2Int& end)
    {
        // check there are no obstacles between origin and end
        if (!(cells[origin.x][origin.y].free && cells[end.x][end.y].free))
            return false;

        bool pathIsFree = true;
        Utils::Vector2 vector = indexToCoordinates(end.x, end.y) - indexToCoordinates(origin.x, origin.y);
        Utils::Vector2 increment = vector.normalized() * (settings.scale * map_.info.resolution);
        int steps = vector.norm() / (settings.scale * map_.info.resolution);
        int index = 0;
        Utils::Vector2 current_point = indexToCoordinates(origin.x, origin.y);
        while (index < steps && pathIsFree)
        {
            current_point += increment;
            index++;
            Vector2Int pair = coordinatesToIndex(current_point.x, current_point.y);
            pathIsFree = indicesInBounds(pair) && cells[pair.x][pair.y].free;
        }

        return pathIsFree;
    }

    void PMFS_GSL::normalizeSourceProb(std::vector<std::vector<Cell>>& variable)
    {
        // we account for the possibility of having positive and negative values by offsetting everything by the value of the minimum (capped at 0)
        // so, [-1, -0.5, 1, 2] would become [0, 0.5, 2, 3] before the normalization happens
        double total = 0;
        double min = 0;
        int count = 0;
        for (int i = 0; i < variable.size(); i++)
        {
            for (int j = 0; j < variable[0].size(); j++)
            {
                if (cells[i][j].free)
                {
                    total += variable[i][j].sourceProbability;
                    min = std::min(min, variable[i][j].sourceProbability);
                    count++;
                }
            }
        }

        total = total - min * count;
#pragma omp parallel for collapse(2)
        for (int i = 0; i < variable.size(); i++)
        {
            for (int j = 0; j < variable[0].size(); j++)
            {
                if (cells[i][j].free)
                {
                    variable[i][j].sourceProbability = (variable[i][j].sourceProbability - min) / total;
                }
            }
        }
    }

    void PMFS_GSL::save_results_to_file(int result, double i, double j, double allI, double allJ)
    {
        nav_client->async_cancel_all_goals();

        // 1. Search time.
        rclcpp::Duration time_spent = node->now() - start_time;
        double search_t = time_spent.seconds();

        spdlog::info("RESULT IS: Success={0}, Search_t={1} s, Error={2} m", result, search_t, errorOverTime.back().error);

        double error = sqrt(pow(ground_truth_x - i, 2) + pow(ground_truth_y - j, 2));
        double errorAll = sqrt(pow(ground_truth_x - allI, 2) + pow(ground_truth_y - allJ, 2));
        // Save to file
        if (results_file != "")
        {
            std::ofstream file;
            file.open(results_file, std::ios_base::app);
            if (result == 1)
                file << t1 << " " << search_t << " " << errorAll << " " << error << " " << exploredCells << " " << varianceSourcePosition() << "\n";
            else
                file << "FAILED " << t1 << " " << search_t << " " << errorAll << " " << error << " " << exploredCells << "\n";
        }
        else
            spdlog::warn("No file provided for logging result. Skipping it.");

        if (errors_file != "")
        {
            std::ofstream file;
            file.open(errors_file, std::ios_base::app);
            file << "\n\n========================\n";
            for (int i = 0; i < errorOverTime.size(); i++)
                file << i << ", " << errorOverTime[i].error << " " << errorOverTime[i].variance << "\n";

            file << "-------All cells--------\n";
            for (int i = 0; i < errorOverTimeAll.size(); i++)
                file << i << ", " << errorOverTimeAll[i].error << " " << errorOverTimeAll[i].variance << "\n";
        }
        else
            spdlog::warn("No file provided for logging errors. Skipping it.");

        if (path_file != "")
        {
            std::ofstream file;
            file.open(path_file, std::ios_base::app);
            file << "------------------------\n";
            for (PoseWithCovarianceStamped& p : robot_poses_vector)
                file << p.pose.pose.position.x << ", " << p.pose.pose.position.y << "\n";
        }
        else
            spdlog::warn("No file provided for logging path. Skipping it.");
    }

    void PMFS_GSL::estimateWind(bool groundTruth)
    {

        // if not compiled with gaden support, you have no choice but to use ground truth :)

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
                    Vector2Int pair = coordinatesToIndex(GMRFRequest->x[ind], GMRFRequest->y[ind]);
                    estimatedWindVectors[pair.x][pair.y] = Utils::Vector2(std::cos(response->v[ind]), std::sin(response->v[ind])) * response->u[ind];
                }
            }
            else
                spdlog::warn("CANNOT READ ESTIMATED WIND VECTORS");

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
                    Vector2Int pair = coordinatesToIndex(groundTruthWindRequest->x[ind], groundTruthWindRequest->y[ind]);
                    estimatedWindVectors[pair.x][pair.y] = Utils::Vector2(response->u[ind], response->v[ind]);
                }
            }
            else
                spdlog::warn("CANNOT READ ESTIMATED WIND VECTORS");
        }
#endif
    }

} // namespace PMFS