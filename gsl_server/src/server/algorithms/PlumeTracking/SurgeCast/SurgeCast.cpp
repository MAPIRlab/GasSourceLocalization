#include <gsl_server/algorithms/PlumeTracking/SurgeCast/SurgeCast.hpp>
#include <gsl_server/algorithms/PlumeTracking/MovingStatePlumeTracking.hpp>
#include <gsl_server/Utils/RosUtils.hpp>
#include <gsl_server/Utils/Math.hpp>
#include <angles/angles.h>

namespace GSL
{
    void SurgeCast::setCastGoal(double downWind_direction)
    {
        constexpr int maxNumberOfCasts = 2;
        double movement_dir;
        double current_step;

        static int numberOfConsecutiveCasts = 0;
        if (dynamic_cast<MovingStatePlumeTracking*>(movingState.get())->currentMovement != PTMovement::RecoverPlume)
        {
            // move right
            movement_dir = angles::normalize_angle(downWind_direction + 3.14159 / 2);
            current_step = surgeStepSize;
            numberOfConsecutiveCasts = 1;
        }
        else if (numberOfConsecutiveCasts <= maxNumberOfCasts)
        {
            // move left
            numberOfConsecutiveCasts++;
            movement_dir = angles::normalize_angle(downWind_direction - 3.14159 / 2);
            current_step = 2 * surgeStepSize;
        }
        else
        {
            GSL_WARN("Gas plume completely Lost!");
            setExplorationGoal();
            return;
        }

        GSL_INFO_COLOR(fmt::terminal_color::yellow, "Cast");

        NavigateToPose::Goal goal;

        constexpr int safetyLimit = 10;
        for (int i = 0; i < safetyLimit; i++)
        {
            goal.pose.header.frame_id = "map";
            goal.pose.header.stamp = node->now();

            // Set a goal in the crosswind direction
            goal.pose.pose.position.x = currentRobotPose.pose.pose.position.x + current_step * cos(movement_dir);
            goal.pose.pose.position.y = currentRobotPose.pose.pose.position.y + current_step * sin(movement_dir);
            goal.pose.pose.orientation = Utils::createQuaternionMsgFromYaw(angles::normalize_angle(movement_dir));

            // reduce step (in case goal is an obstacle or out of bounds)
            if (current_step <= 0)
            {
                GSL_WARN("Cannot move further CrossWind!");
                setExplorationGoal();
                return;
            }
            current_step = current_step - 0.3;
            if (movingState->checkGoal(goal))
                break;
        }

        dynamic_cast<MovingStatePlumeTracking*>(movingState.get())->currentMovement = PTMovement::RecoverPlume;
        movingState->sendGoal(goal);
    }

} // namespace GSL