#include <angles/angles.h>
#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <gsl_server/algorithms/Common/Grid.hpp>
#include <gsl_server/algorithms/PMFS/MovingStatePMFS.hpp>
#include <gsl_server/algorithms/PMFS/PMFS.hpp>
#include <gsl_server/algorithms/PMFS/internal/HitProbability.hpp>

namespace GSL
{
    MovingStatePMFS::MovingStatePMFS(Algorithm* _algorithm) : MovingState(_algorithm)
    {
        pmfs = dynamic_cast<PMFS*>(_algorithm);

        publishers.explorationValue = pmfs->node->create_publisher<Marker>("explorationValue", 1);
        publishers.varianceHit = pmfs->node->create_publisher<Marker>("varianceHit", 1);
        publishers.movementSets = pmfs->node->create_publisher<Marker>("movementSets", 1);
    }

    void MovingStatePMFS::chooseGoalAndMove()
    {
        auto& gridMetadata = pmfs->gridMetadata;

        // Add nearby cells to the open set
        {
            int i = pmfs->gridMetadata.coordinatesToIndex(pmfs->currentRobotPose.pose.pose).x,
                j = pmfs->gridMetadata.coordinatesToIndex(pmfs->currentRobotPose.pose.pose).y;

            int openMoveSetExpasion = pmfs->settings.movement.openMoveSetExpasion;
            int oI = std::max(0, i - openMoveSetExpasion);
            int fI = std::min((int)gridMetadata.height - 1, i + openMoveSetExpasion);
            int oJ = std::max(0, j - openMoveSetExpasion);
            int fJ = std::min((int)gridMetadata.width - 1, j + openMoveSetExpasion);

            for (int r = oI; r <= fI; r++)
            {
                for (int c = oJ; c <= fJ; c++)
                {
                    Vector2Int p(r, c);
                    if (closedMoveSet.find(p) == closedMoveSet.end() && pmfs->visibilityMap->isVisible({i, j}, p) == Visibility::Visible)
                        openMoveSet.insert(p);
                }
            }
        }

        // remove this cell from the open set
        openMoveSet.erase(pmfs->gridMetadata.coordinatesToIndex(pmfs->currentRobotPose.pose.pose));

        // Find the cell with the highest estimated information value
        //------------------------------------------------------
        NavigateToPose::Goal goal;
        int goalI = -1, goalJ = -1;
        double interest = -DBL_MAX;
        double maxDist = 0;

        // We have a small random chance of using the explorationValue instead of the proper information value even in the second phase
        // because it is beneficial to have at least some measurements spanning a large area of the map
        float explorationC = Utils::uniformRandom(0, 1);

        for (const auto& indices : openMoveSet)
        {
            int r = indices.x;
            int c = indices.y;
            NavigateToPose::Goal tempGoal = indexToGoal(r, c);

            double explorationTerm = explorationValue(r, c);
            double varianceTerm = pmfs->simulations.varianceOfHitProb[gridMetadata.indexOf({r, c})] *
                                  (1 - pmfs->hitProbability[gridMetadata.indexOf({r, c})].confidence);

            double this_interest =
                currentMovement == MovementType::Exploration || varianceTerm == 0 || explorationC < pmfs->settings.movement.explorationProbability
                ? explorationTerm
                : varianceTerm;

            //keep the best so far as the goal
            if (this_interest > interest)
            {
                if (checkGoal(tempGoal))
                {
                    interest = this_interest;
                    goalI = r;
                    goalJ = c;
                    maxDist = pmfs->hitProbability[gridMetadata.indexOf({r, c})].distanceFromRobot;
                    goal = tempGoal;
                }
            }
        }

        if (closedMoveSet.find(pmfs->gridMetadata.coordinatesToIndex(pmfs->currentRobotPose.pose.pose)) == closedMoveSet.end())
            openMoveSet.insert(pmfs->gridMetadata.coordinatesToIndex(pmfs->currentRobotPose.pose.pose));
        GSL_ASSERT_MSG(closedMoveSet.find({goalI, goalJ}) == closedMoveSet.end(), "Goal is in closed set, what the hell");

        pmfs->iterationsCounter++;
        sendGoal(goal);
    }

    double MovingStatePMFS::explorationValue(int i, int j)
    {
        // the exploration value is the sum of the uncertainty about the hit probability for all cells around (i,j)

        Vector2Int ij(i, j);
        auto range = pmfs->visibilityMap->at(ij);

        double sum = 0;
        for (const auto& p : range)
        {
            float distance = vmath::length(Vector2(ij - p)); // not the navigable distance, but we are close enough that it does not matter
            sum += (1 - pmfs->hitProbability[pmfs->gridMetadata.indexOf(p)].confidence) * std::exp(-distance);
            GSL_ASSERT(sum > 0);
        }
        return sum;
    }

    NavigateToPose::Goal MovingStatePMFS::indexToGoal(int i, int j)
    {
        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = pmfs->node->now();

        Vector2 pos = pmfs->gridMetadata.indexToCoordinates(i, j);
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
        currentGoal = std::nullopt;
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
            pmfs->gridMetadata.coordinatesToIndex(currentGoal.value().pose.pose.position.x, currentGoal.value().pose.pose.position.y);
        openMoveSet.erase(indicesGoal);
        closedMoveSet.insert(indicesGoal);
        MovingState::Fail();
    }

    void MovingStatePMFS::publishMarkers()
    {
        GridMetadata& gridMetadata = pmfs->gridMetadata;
        Grid<PMFS_internal::HitProbability> grid(pmfs->hitProbability, pmfs->occupancy, gridMetadata);

        Marker explorationMarker = Utils::emptyMarker({0.2, 0.2}, pmfs->node->get_clock());

        Marker varianceMarker = explorationMarker;

        Marker movementSetsMarker = explorationMarker;

        double maxExpl = -DBL_MAX;
        double maxVar = -DBL_MAX;
        double minExpl = DBL_MAX;
        double minVar = DBL_MAX;
        for (int a = 0; a < gridMetadata.height; a++)
        {
            for (int b = 0; b < gridMetadata.width; b++)
            {
                if (!grid.freeAt(a, b))
                    continue;
                maxExpl = std::max(maxExpl, explorationValue(a, b));
                maxVar = std::max(maxVar, pmfs->simulations.varianceOfHitProb[gridMetadata.indexOf({a, b})] * (1 - grid.dataAt(a, b).confidence));

                minExpl = std::min(minExpl, explorationValue(a, b));
                minVar = std::min(minVar, pmfs->simulations.varianceOfHitProb[gridMetadata.indexOf({a, b})] * (1 - grid.dataAt(a, b).confidence));
            }
        }

        for (int a = 0; a < gridMetadata.height; a++)
        {
            for (int b = 0; b < gridMetadata.width; b++)
            {
                if (!grid.freeAt(a, b))
                    continue;
                auto coords = gridMetadata.indexToCoordinates(a, b);
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
                varianceColor =
                    Utils::valueToColor(pmfs->simulations.varianceOfHitProb[gridMetadata.indexOf({a, b})] * (1 - grid.dataAt(a, b).confidence),
                                        minVar, maxVar, Utils::valueColorMode::Linear);

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
        publishers.explorationValue->publish(explorationMarker);
        publishers.varianceHit->publish(varianceMarker);
        publishers.movementSets->publish(movementSetsMarker);
    }

} // namespace GSL