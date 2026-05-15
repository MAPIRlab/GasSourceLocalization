#include <angles/angles.h>
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>
#include <gsl_server/algorithms/GrGSL/GrGSLLib.hpp>
#include "MovingStateSemanticGrGSL.hpp"

namespace GSL
{
    using namespace GrGSL_internal;
    MovingStateSemanticGrGSL::MovingStateSemanticGrGSL(Algorithm* _algorithm)
        : MovingState(_algorithm)
    {
        grgsl = dynamic_cast<SemanticGrGSL*>(algorithm);
        clientWind = grgsl->node->create_client<WindEstimation>("/WindEstimation");
    }

    void MovingStateSemanticGrGSL::chooseGoalAndMove()
    {
        const auto& settings = grgsl->settings;
        const Grid2D<Cell> grid(grgsl->cells, grgsl->navigationOccupancy, grgsl->gridMetadata);

        // update sets
        {
            Vector2Int currentPosition = grgsl->gridMetadata.coordinatesToIndices(grgsl->currentRobotPose.pose.pose);
            int expansionSize = 5;

            // loop limits
            size_t startC = std::max(0, currentPosition.x - expansionSize);
            size_t endC = std::min(grgsl->gridMetadata.dimensions.x - 1, currentPosition.x + expansionSize);
            size_t startR = std::max(0, currentPosition.y - expansionSize);
            size_t endR = std::min(grgsl->gridMetadata.dimensions.y - 1, currentPosition.y + expansionSize);

            for (int row = startR; row <= endR; row++)
            {
                for (int col = startC; col <= endC; col++)
                {
                    Vector2Int colRow(col, row);
                    if (closedMoveSet.find(colRow) == closedMoveSet.end() && grid.occupancyAt(colRow) && grid.dataAt(colRow).distance < 5)
                    {
                        openMoveSet.insert(colRow);
                    }
                }
            }
        }

        std::optional<NavigateToPose::Goal> goal;
        if (settings.infoTaxis)
            goal = getInfotaxisGoal();
        else
            goal = getNormalGoal();

        if (!goal)
        {
            GSL_ERROR("Cannot move anywhere! Probably got stuck on an obstacle. Aborting movement.");
            Fail();
            return;
        }

        grgsl->exploredCells++;

        Vector2Int indices = grid.metadata.coordinatesToIndices(goal->pose.pose.position.x, goal->pose.pose.position.y);
        closedMoveSet.insert(indices);
        openMoveSet.erase(indices);

        GSL_INFO("MOVING TO {:.2f},{:.2f}", goal->pose.pose.position.x, goal->pose.pose.position.y);
        sendGoal(*goal);

        // close nearby cells to avoid repeating the same pose with only minor variations
        if (!settings.allowMovementRepetition)
        {
            Vector2Int goalIndices = grid.metadata.coordinatesToIndices(goal->pose.pose.position.x, goal->pose.pose.position.y);

            // loop limits
            size_t startC = std::max(0, goalIndices.x - 1);
            size_t endC = std::min(grgsl->gridMetadata.dimensions.x - 1, goalIndices.x + 1);
            size_t startR = std::max(0, goalIndices.y - 1);
            size_t endR = std::min(grgsl->gridMetadata.dimensions.y - 1, goalIndices.y + 1);

            for (int row = startR; row <= endR; row++)
            {
                for (int col = startC; col <= endC; col++)
                {
                    Vector2Int colRow(col, row);
                    if (grid.occupancyAt(colRow))
                    {
                        openMoveSet.erase(colRow);
                        closedMoveSet.insert(colRow);
                    }
                }
            }
        }
    }

    std::optional<NavigateToPose::Goal> MovingStateSemanticGrGSL::getNormalGoal()
    {
        const auto& settings = grgsl->settings;
        const Grid2D<double> grid(grgsl->combinedSourceProbability, grgsl->navigationOccupancy, grgsl->gridMetadata);

        // Graph exploration
        std::optional<NavigateToPose::Goal> goal = std::nullopt;

        if (!openMoveSet.empty())
        {
            double max = 0;
            double maxDist = 0;
            for (auto& p : openMoveSet)
            {
                double sourceProb = grid.dataAt(p);
                Cell& cell = grgsl->cells.at(grgsl->gridMetadata.indexOf(p));
                if (sourceProb > max || (sourceProb == max && cell.distance > maxDist))
                {
                    NavigateToPose::Goal tempGoal = indexToGoal(p.x, p.y);
                    if (checkGoal(tempGoal))
                    {
                        max = sourceProb;
                        maxDist = cell.distance;
                        goal = tempGoal;
                    }
                }
            }
        }
        else
            GSL_ERROR("Set of open nodes is empty! Are you certain the source is reachable?");

        return goal;
    }

    std::optional<NavigateToPose::Goal> MovingStateSemanticGrGSL::getInfotaxisGoal()
    {
        const auto& settings = grgsl->settings;
        const Grid2D<Cell> grid(grgsl->cells, grgsl->navigationOccupancy, grgsl->gridMetadata);

        // it is never helpful to stop the propagation for each of the considered movement positions, so temporarily turn debug mode off while we do this
        bool wasDebugging = GrGSLLib::debuggingPropagation;
        GrGSLLib::debuggingPropagation = false;

        // Infotactic navigation
        std::optional<NavigateToPose::Goal> goal = std::nullopt;
        std::vector<WindVector> wind = estimateWind();
        std::mutex mtx;
        double ent = -DBL_MAX;
        double maxDist = 0;
        rclcpp::Time tstart = grgsl->node->now();
        if (!wind.empty())
        {
#pragma omp parallel for
            for (int index = 0; index < wind.size(); index++)
            {
                int col = wind[index].col;
                int row = wind[index].row;

                double entAux = GrGSLLib::informationGain(wind[index], grid, grgsl->settings, grgsl->positionOfLastHit);
                mtx.lock();
                if (entAux > ent || (ent == entAux && grid.dataAt(col, row).distance > maxDist))
                {
                    NavigateToPose::Goal tempGoal = indexToGoal(col, row);
                    bool pathExists = checkGoal(tempGoal);
                    if (pathExists)
                    {
                        ent = entAux;
                        maxDist = grid.dataAt(col, row).distance;
                        goal = tempGoal;
                    }
                }
                mtx.unlock();
            }
        }
        else
            GSL_ERROR("Set of open nodes is empty! Are you certain the source is reachable?");

        double timeInfotaxis = (grgsl->node->now() - tstart).seconds();
        GSL_INFO("Time infotaxis: {}", timeInfotaxis);
        GSL_INFO("Number of considered cells: {}", wind.size());

        GrGSLLib::debuggingPropagation = wasDebugging;
        return goal;
    }

    NavigateToPose::Goal MovingStateSemanticGrGSL::indexToGoal(int i, int j)
    {
        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = grgsl->node->now();

        Vector2 pos = grgsl->gridMetadata.indicesToCoordinates(i, j);
        Vector2 coordR = Vector2(grgsl->currentRobotPose.pose.pose.position.x, grgsl->currentRobotPose.pose.pose.position.y);

        double move_angle = (atan2(pos.y - coordR.y, pos.x - coordR.x));
        goal.pose.pose.position.x = pos.x;
        goal.pose.pose.position.y = pos.y;
        goal.pose.pose.orientation = Utils::createQuaternionMsgFromYaw(angles::normalize_angle(move_angle));
#ifdef USE_NAV_ASSISTANT
        goal.turn_before_nav = true;
#endif
        return goal;
    }

    std::vector<WindVector> MovingStateSemanticGrGSL::estimateWind()
    {
        const Grid2D<Cell> grid(grgsl->cells, grgsl->navigationOccupancy, grgsl->gridMetadata);

        std::vector<Vector2Int> indices;
        for (auto& p : openMoveSet)
        {
            if (grid.dataAt(p).distance < 5)
            {
                indices.push_back(p);
            }
        }

        std::vector<WindVector> result = getWindVectors(indices);
        return result;
    }

    std::vector<GrGSL_internal::WindVector> MovingStateSemanticGrGSL::getWindVectors(const std::vector<Vector2Int>& indices)
    {
        const Grid2D<Cell> grid(grgsl->cells, grgsl->navigationOccupancy, grgsl->gridMetadata);
        auto request = std::make_shared<WindEstimation::Request>();
        for (const auto index : indices)
        {
            Vector2 coords = grid.metadata.indicesToCoordinates(index.x, index.y);
            request->x.push_back(coords.x);
            request->y.push_back(coords.y);
        }

        std::vector<WindVector> result(indices.size());
        auto future = clientWind->async_send_request(request);
        auto future_result = rclcpp::spin_until_future_complete(grgsl->node, future, std::chrono::seconds(1));
        if (future_result == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            for (int ind = 0; ind < indices.size(); ind++)
            {
                result[ind].col = indices[ind].x;
                result[ind].row = indices[ind].y;
                result[ind].speed = response->u[ind];
                result[ind].angle = angles::normalize_angle(response->v[ind] + M_PI);
            }
        }
        else
            GSL_WARN("CANNOT READ ESTIMATED WIND VECTORS");
        
        return result;
    }

} // namespace GSL