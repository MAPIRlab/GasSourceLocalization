#include <gsl_server/algorithms/PMFS/MovingStatePMFS.hpp>
#include <gsl_server/algorithms/PMFS/PMFS.hpp>
#include <gsl_server/Utils/Math.hpp>
#include <angles/angles.h>

namespace GSL
{
    MovingStatePMFS::MovingStatePMFS(Algorithm* _algorithm) : MovingState(_algorithm)
    {
        pmfs = dynamic_cast<PMFS*>(_algorithm);
    }

    void MovingStatePMFS::chooseGoalAndMove()
    {
        {
            int i = pmfs->currentPosIndex().x, j = pmfs->currentPosIndex().y;

            int openMoveSetExpasion = pmfs->settings.movement.openMoveSetExpasion;
            int oI = std::max(0, i - openMoveSetExpasion);
            int fI = std::min((int)pmfs->grid.size() - 1, i + openMoveSetExpasion);
            int oJ = std::max(0, j - openMoveSetExpasion);
            int fJ = std::min((int)pmfs->grid[0].size() - 1, j + openMoveSetExpasion);

            for (int r = oI; r <= fI; r++)
            {
                for (int c = oJ; c <= fJ; c++)
                {
                    Vector2Int p(r, c);
                    hashSet& visibilitySet = pmfs->visibilityMap.at(Vector2Int(i, j));
                    if (closedMoveSet.find(p) == closedMoveSet.end() && visibilitySet.find(p) != visibilitySet.end())
                        openMoveSet.insert(p);
                }
            }
        }

        openMoveSet.erase(pmfs->currentPosIndex());
        NavigateToPose::Goal goal;
        int goalI = -1, goalJ = -1;
        double interest = -DBL_MAX;
        double maxDist = 0;

        float explorationC = Utils::uniformRandom(0, 1);

        for (const auto& indices : openMoveSet)
        {
            int r = indices.x;
            int c = indices.y;
            NavigateToPose::Goal tempGoal = indexToGoal(r, c);

            double explorationTerm = explorationValue(r, c);
            double varianceTerm = pmfs->simulations.varianceOfHitProb[r][c] * (1 - pmfs->grid[r][c].hitProbability.confidence);

            double this_interest =
                currentMovement == MovementType::Exploration || varianceTerm == 0 || explorationC < pmfs->settings.movement.explorationProbability
                    ? explorationTerm
                    : varianceTerm;

            if (this_interest > interest)
            {
                if (checkGoal(tempGoal))
                {
                    interest = this_interest;
                    goalI = r;
                    goalJ = c;
                    maxDist = pmfs->grid[r][c].distanceFromRobot;
                    goal = tempGoal;
                }
            }
        }

        if (closedMoveSet.find(pmfs->currentPosIndex()) == closedMoveSet.end())
            openMoveSet.insert(pmfs->currentPosIndex());
        GSL_ASSERT_MSG(closedMoveSet.find({goalI, goalJ}) == closedMoveSet.end(), "Goal is in closed set, what the hell");

        pmfs->iterationsCounter++;
        sendGoal(goal);
    }

    double MovingStatePMFS::explorationValue(int i, int j)
    {
        double sum = 0;
        Vector2Int ij(i, j);
        auto& set = pmfs->visibilityMap.at(ij);
        for (const auto& p : set)
        {
            float distance = glm::length(Vector2(ij - p)); // not the navigable distance, but we are close enough that it does not matter
            sum += (1 - pmfs->grid[p.x][p.y].hitProbability.confidence) * std::exp(-distance);
        }
        return sum;
    }

    NavigateToPose::Goal MovingStatePMFS::indexToGoal(int i, int j)
    {
        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = pmfs->node->now();

        Vector2 pos = pmfs->gridData.indexToCoordinates(i, j);
        Vector2 coordR = {pmfs->currentRobotPose.pose.pose.position.x, pmfs->currentRobotPose.pose.pose.position.y};

        double move_angle = (std::atan2(pos.y - coordR.y, pos.x - coordR.x));
        goal.pose.pose.position.x = pos.x;
        goal.pose.pose.position.y = pos.y;
        goal.pose.pose.orientation = Utils::createQuaternionMsgFromYaw(angles::normalize_angle(move_angle));
#ifdef USE_NAV_ASSISTANT
        goal.turn_before_nav = true;
#endif
        return goal;
    }

    void MovingStatePMFS::debugMoveTo(int i, int j)
    {
        GSL_INFO("Sending a goal from UI. This is not part of the normal execution flow of the algorithm");
        NavigateToPose::Goal goal = indexToGoal(i, j);
        sendGoal(goal);
    }

    void MovingStatePMFS::Fail()
    {
        // There is a somewhat inconsistent behaviour when a goal times out and is manually cancelled.
        // sometimes we get a callback notifying of the cancelation, sometimes not
        // when we do, Fail() gets called twice: once by us, once inside the callback
        // if this is the second time, we should not do anything
        if (!currentGoal.has_value())
            return;

        Vector2Int indicesGoal =
            pmfs->gridData.coordinatesToIndex(currentGoal.value().pose.pose.position.x, currentGoal.value().pose.pose.position.y);
        openMoveSet.erase(indicesGoal);
        closedMoveSet.insert(indicesGoal);
        MovingState::Fail();
    }

    void MovingStatePMFS::publishMarkers()
    {
        const auto& grid = pmfs->grid;
        const auto& gridData = pmfs->gridData;

        Marker explorationMarker = Utils::emptyMarker({0.2, 0.2}, pmfs->node->get_clock());

        Marker varianceMarker = explorationMarker;

        Marker movementSetsMarker = explorationMarker;

        double maxExpl = -DBL_MAX;
        double maxVar = -DBL_MAX;
        double minExpl = DBL_MAX;
        double minVar = DBL_MAX;
        for (int a = 0; a < grid.size(); a++)
        {
            for (int b = 0; b < grid[0].size(); b++)
            {
                if (!grid[a][b].free)
                    continue;
                maxExpl = std::max(maxExpl, explorationValue(a, b));
                maxVar = std::max(maxVar, pmfs->simulations.varianceOfHitProb[a][b] * (1 - grid[a][b].hitProbability.confidence));

                minExpl = std::min(minExpl, explorationValue(a, b));
                minVar = std::min(minVar, pmfs->simulations.varianceOfHitProb[a][b] * (1 - grid[a][b].hitProbability.confidence));
            }
        }

        for (int a = 0; a < grid.size(); a++)
        {
            for (int b = 0; b < grid[0].size(); b++)
            {
                if (!grid[a][b].free)
                    continue;
                auto coords = gridData.indexToCoordinates(a, b);
                Point p;
                p.x = coords.x;
                p.y = coords.y;
                p.z = pmfs->settings.visualization.markers_height;

                std_msgs::msg::ColorRGBA explorationColor;
                std_msgs::msg::ColorRGBA advantageColor;
                std_msgs::msg::ColorRGBA varianceColor;

                if (openMoveSet.find(Vector2Int(a, b)) == openMoveSet.end())
                {
                    explorationColor.r = 0;
                    explorationColor.g = 0;
                    explorationColor.b = 0;
                    explorationColor.a = 1;
                    advantageColor = explorationColor;
                }
                else
                {
                    explorationColor = Utils::valueToColor(explorationValue(a, b), minExpl, maxExpl, Utils::valueColorMode::Linear);
                }
                varianceColor = Utils::valueToColor(pmfs->simulations.varianceOfHitProb[a][b] * (1 - grid[a][b].hitProbability.confidence), minVar,
                                                    maxVar, Utils::valueColorMode::Linear);

                explorationMarker.points.push_back(p);
                explorationMarker.colors.push_back(explorationColor);

                p.z = pmfs->settings.visualization.markers_height - 0.1;
                varianceMarker.points.push_back(p);
                varianceMarker.colors.push_back(varianceColor);

                movementSetsMarker.points.push_back(p);
                if (openMoveSet.find({a, b}) != openMoveSet.end())
                    movementSetsMarker.colors.push_back(Utils::create_color(0, 1, 0, 1));
                else if (closedMoveSet.find(Vector2Int(a, b)) != closedMoveSet.end())
                    movementSetsMarker.colors.push_back(Utils::create_color(1, 0, 0, 1));
                else
                    movementSetsMarker.colors.push_back(Utils::create_color(0, 0, 1, 1));
            }
        }
        pmfs->pubs.markers.debug.explorationValue->publish(explorationMarker);
        pmfs->pubs.markers.debug.varianceHit->publish(varianceMarker);
        pmfs->pubs.markers.debug.movementSets->publish(movementSetsMarker);
    }

} // namespace GSL