#include <algorithms/PMFS/PMFS.h>

namespace PMFS
{

    //----------------
    // NAVIGATION
    //----------------

    void PMFS_GSL::cancel_navigation(bool succeeded)
    {
        nav_client->async_cancel_all_goals(); // Cancel current navigations
        inMotion = false;

        // Start a new measurement-phase while standing
        stop_and_measure_gas_v.clear();
        stop_and_measure_windS_v.clear();
        stop_and_measure_windD_v.clear();
        time_stopped = node->now(); // Start timer for initial wind measurement

        previous_state = current_state;
        if (succeeded)
            current_state = State::STOP_AND_MEASURE;
        else
        {
            openMoveSet.erase(currentPosIndex);
            closedMoveSet.insert(currentPosIndex);
        }
    }

    void PMFS_GSL::updateSets()
    {
        int i = currentPosIndex.x, j = currentPosIndex.y;

        int oI = std::max(0, i - settings.movement.openMoveSetExpasion);
        int fI = std::min((int)cells.size() - 1, i + settings.movement.openMoveSetExpasion);
        int oJ = std::max(0, j - settings.movement.openMoveSetExpasion);
        int fJ = std::min((int)cells[0].size() - 1, j + settings.movement.openMoveSetExpasion);

        for (int r = oI; r <= fI; r++)
        {
            for (int c = oJ; c <= fJ; c++)
            {
                Vector2Int p(r, c);
                hashSet& visibilitySet = visibilityMap.at(Vector2Int(i, j));
                if (closedMoveSet.find(p) == closedMoveSet.end() && visibilitySet.find(p) != visibilitySet.end())
                {
                    openMoveSet.insert(p);
                }
            }
        }
    }

    void PMFS_GSL::setGoal()
    {
        if (inMotion)
            return;
        updateSets();
        openMoveSet.erase(currentPosIndex);
        NavAssistant::Goal goal;
        int goalI = -1, goalJ = -1;
        double interest = -DBL_MAX;
        double maxDist = 0;

        float explorationC = Utils::uniformRandom(0, 1);

        if (debugStuff.GoalSet)
        {
            goal = indexToGoal(debugStuff.debugGoal.x, debugStuff.debugGoal.y);
            debugStuff.GoalSet = false;
        }
        else
        {
            for (const auto& indices : openMoveSet)
            {
                int r = indices.x;
                int c = indices.y;
                NavAssistant::Goal tempGoal = indexToGoal(r, c);

                double explorationTerm = explorationValue(r, c);
                double varianceTerm = simulations.varianceOfHitProb[r][c] * (1 - cells[r][c].hitProbability.confidence);

                double this_interest =
                    current_state == State::EXPLORATION || varianceTerm == 0 || explorationC < settings.movement.explorationProbability
                        ? explorationTerm
                        : varianceTerm;

                if (this_interest > interest)
                {
                    if (checkGoal(tempGoal))
                    {
                        interest = this_interest;
                        goalI = r;
                        goalJ = c;
                        maxDist = cells[r][c].distanceFromRobot;
                        goal = tempGoal;
                    }
                }
            }
        }

        if (closedMoveSet.find(currentPosIndex) == closedMoveSet.end())
            openMoveSet.insert(currentPosIndex);
        GSL_ASSERT_MSG(closedMoveSet.find({goalI, goalJ}) == closedMoveSet.end(), "Goal is in closed set, what the hell");

        exploredCells++;
        moveTo(goal);
    }

    void PMFS_GSL::moveTo(NavAssistant::Goal& goal)
    {

        inMotion = true;
        sendGoal(goal);

        currentPosIndex = coordinatesToIndex(goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);

        if (!settings.movement.allowMovementRepetition)
        {
            // close nearby cells to avoid repeating the same pose with only minor variations
            int i = currentPosIndex.x, j = currentPosIndex.y;
            int oI = std::max(0, i - 1);
            int fI = std::min((int)cells.size() - 1, i + 1);
            int oJ = std::max(0, j - 1);
            int fJ = std::min((int)cells[0].size() - 1, j + 1);

            for (int r = oI; r <= fI; r++)
            {
                for (int c = oJ; c <= fJ; c++)
                {
                    Vector2Int p(r, c);
                    if (cells[r][c].free)
                    {
                        openMoveSet.erase(p);
                        closedMoveSet.insert(p);
                    }
                }
            }
        }
    }

    NavAssistant::Goal PMFS_GSL::indexToGoal(int i, int j)
    {
        NavAssistant::Goal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = node->now();

        Utils::Vector2 pos = indexToCoordinates(i, j);
        Utils::Vector2 coordR = indexToCoordinates(currentPosIndex.x, currentPosIndex.y);

        double move_angle = (std::atan2(pos.y - coordR.y, pos.x - coordR.x));
        goal.target_pose.pose.position.x = pos.x;
        goal.target_pose.pose.position.y = pos.y;
        goal.target_pose.pose.orientation = Utils::createQuaternionMsgFromYaw(angles::normalize_angle(move_angle));
        goal.turn_before_nav = true;
        return goal;
    }

    //----------------
    // INFOTAXIS
    //----------------

    double PMFS_GSL::explorationValue(int i, int j)
    {
        double sum = 0;
        Vector2Int ij(i, j);
        auto& set = visibilityMap.at(ij);
        for (const auto& p : set)
        {
            float distance = (ij - p).norm(); // not the navigable distance, but we are close enough that it does not matter
            sum += (1 - cells[p.x][p.y].hitProbability.confidence) * std::exp(-distance);
        }
        return sum;
    }

} // namespace PMFS