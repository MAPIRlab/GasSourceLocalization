#include <gsl_server/algorithms/GrGSL/MovingStateGrGSL.hpp>
#include <gsl_server/Utils/RosUtils.hpp>
#include <angles/angles.h>

namespace GSL
{
    MovingStateGrGSL::MovingStateGrGSL(Algorithm* _algorithm) : MovingState(_algorithm)
    {
        grgsl = dynamic_cast<GrGSL*>(algorithm);
        clientWind = grgsl->node->create_client<GrGSL::WindEstimation>("/WindEstimation");
    }

    void MovingStateGrGSL::chooseGoalAndMove()
    {
        const auto& settings = grgsl->settings;
        const auto& grid = grgsl->grid;
        const auto& gridData = grgsl->gridData;

        // update sets
        {
            Vector2Int currentPosition = grgsl->currentPosIndex();
            int i = currentPosition.x, j = currentPosition.y;
            int expansionSize = 5;

            int oI = std::max(0, i - expansionSize);
            int fI = std::min((int)grid.size() - 1, i + expansionSize);
            int oJ = std::max(0, j - expansionSize);
            int fJ = std::min((int)grid[0].size() - 1, j + expansionSize);

            for (int r = oI; r <= fI; r++)
            {
                for (int c = oJ; c <= fJ; c++)
                {
                    Vector2Int p(r, c);
                    if (closedMoveSet.find(p) == closedMoveSet.end() && grid[r][c].free && grid[r][c].distance < 5)
                    {
                        openMoveSet.insert(p);
                    }
                }
            }
        }

        NavigateToPose::Goal goal;
        if (settings.infoTaxis)
            goal = getInfotaxisGoal();
        else
            goal = getNormalGoal();

        grgsl->exploredCells++;

        Vector2Int indices = gridData.coordinatesToIndex(goal.pose.pose.position.x, goal.pose.pose.position.y);
        closedMoveSet.insert(indices);
        openMoveSet.erase(indices);

        GSL_INFO("MOVING TO {:.2f},{:.2f}", goal.pose.pose.position.x, goal.pose.pose.position.y);
        sendGoal(goal);

        // close nearby cells to avoid repeating the same pose with only minor variations
        if (!settings.allowMovementRepetition)
        {
            Vector2Int goalIndices = gridData.coordinatesToIndex(goal.pose.pose.position.x, goal.pose.pose.position.y);
            int i = goalIndices.x, j = goalIndices.y;
            int oI = std::max(0, i - 1);
            int fI = std::min((int)grid.size() - 1, i + 1);
            int oJ = std::max(0, j - 1);
            int fJ = std::min((int)grid[0].size() - 1, j + 1);

            for (int r = oI; r <= fI; r++)
            {
                for (int c = oJ; c <= fJ; c++)
                {
                    Vector2Int p(r, c);
                    if (grid[r][c].free)
                    {
                        openMoveSet.erase(p);
                        closedMoveSet.insert(p);
                    }
                }
            }
        }
    }

    NavigateToPose::Goal MovingStateGrGSL::getNormalGoal()
    {
        const auto& settings = grgsl->settings;
        const auto& grid = grgsl->grid;
        const auto& gridData = grgsl->gridData;
        // Graph exploration
        NavigateToPose::Goal goal;

        if (!openMoveSet.empty())
        {
            double max = 0;
            double maxDist = 0;
            for (auto& p : openMoveSet)
            {
                if (grgsl->probability(p) > max || (grgsl->probability(p) == max && grid[p.x][p.y].distance > maxDist))
                {

                    NavigateToPose::Goal tempGoal = indexToGoal(p.x, p.y);
                    if (checkGoal(tempGoal))
                    {
                        max = grgsl->probability(p);
                        maxDist = grid[p.x][p.y].distance;
                        goal = tempGoal;
                    }
                }
            }
        }
        else
            GSL_ERROR("Set of open nodes is empty! Are you certain the source is reachable?");

        return goal;
    }

    NavigateToPose::Goal MovingStateGrGSL::getInfotaxisGoal()
    {
        const auto& settings = grgsl->settings;
        const auto& grid = grgsl->grid;
        const auto& gridData = grgsl->gridData;

        // Infotactic navigation
        NavigateToPose::Goal goal;
        std::vector<GrGSL::WindVector> wind = estimateWind();
        std::mutex mtx;
        double ent = -DBL_MAX;
        double maxDist = 0;
        rclcpp::Time tstart = grgsl->node->now();
        if (!wind.empty())
        {
#pragma omp parallel for
            for (int index = 0; index < wind.size(); index++)
            {
                int r = wind[index].i;
                int c = wind[index].j;
                NavigateToPose::Goal tempGoal = indexToGoal(r, c);

                mtx.lock();
                bool pathExists = checkGoal(tempGoal);
                mtx.unlock();
                if (pathExists)
                {
                    double entAux = grgsl->informationGain(wind[index]);
                    mtx.lock();
                    if (entAux > ent || (ent == entAux && grid[r][c].distance > maxDist))
                    {

                        ent = entAux;
                        maxDist = grid[r][c].distance;
                        goal = tempGoal;
                    }
                    mtx.unlock();
                }
            }
        }
        else
            GSL_ERROR("Set of open nodes is empty! Are you certain the source is reachable?");

        double timeInfotaxis = (grgsl->node->now() - tstart).seconds();
        GSL_INFO("Time infotaxis: {}", timeInfotaxis);
        GSL_INFO("Number of considered cells: {}", wind.size());

        return goal;
    }

    NavigateToPose::Goal MovingStateGrGSL::indexToGoal(int i, int j)
    {

        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = grgsl->node->now();

        Vector2 pos = grgsl->gridData.indexToCoordinates(i, j);
        Vector2 coordR = {grgsl->currentRobotPose.pose.pose.position.x, grgsl->currentRobotPose.pose.pose.position.y};

        double move_angle = (atan2(pos.y - coordR.y, pos.x - coordR.x));
        goal.pose.pose.position.x = pos.x;
        goal.pose.pose.position.y = pos.y;
        goal.pose.pose.orientation = Utils::createQuaternionMsgFromYaw(angles::normalize_angle(move_angle));
#ifdef USE_NAV_ASSISTANT
        goal.turn_before_nav = true;
#endif
        return goal;
    }

    std::vector<GrGSL::WindVector> MovingStateGrGSL::estimateWind()
    {
        const auto& grid = grgsl->grid;
        const auto& gridData = grgsl->gridData;

        // ask the gmrf_wind service for the estimated wind vector in cell i,j
        auto request = std::make_shared<GrGSL::WindEstimation::Request>();

        std::vector<Vector2Int> indices;
        for (auto& p : openMoveSet)
        {
            if (grid[p.x][p.y].distance < 10)
            {
                Vector2 coords = gridData.indexToCoordinates(p.x, p.y);
                request->x.push_back(coords.x);
                request->y.push_back(coords.y);
                indices.push_back(p);
            }
        }

        std::vector<GrGSL::WindVector> result(indices.size());

        auto future = clientWind->async_send_request(request);
        auto future_result = rclcpp::spin_until_future_complete(grgsl->node, future, std::chrono::seconds(1));
        if (future_result == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            for (int ind = 0; ind < indices.size(); ind++)
            {
                result[ind].i = indices[ind].x;
                result[ind].j = indices[ind].y;
                result[ind].speed = response->u[ind];
                result[ind].angle = angles::normalize_angle(response->v[ind] + M_PI);
            }
        }
        else
            GSL_WARN("CANNOT READ ESTIMATED WIND VECTORS");

        return result;
    }
} // namespace GSL