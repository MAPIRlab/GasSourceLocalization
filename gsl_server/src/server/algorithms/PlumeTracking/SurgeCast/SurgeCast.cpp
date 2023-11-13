#include <gsl_server/algorithms/PlumeTracking/SurgeCast/SurgeCast.h>
#include <gsl_server/algorithms/PlumeTracking/MovingStatePlumeTracking.h>
#include <gsl_server/Utils/RosUtils.h>
#include <gsl_server/Utils/Math.h>
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

        GSL_INFO("Cast");

        NavigateToPose::Goal goal;
        do
        {
            goal.pose.header.frame_id = "map";
            goal.pose.header.stamp = node->now();

            // Set a goal in the crosswind direction
            goal.pose.pose.position.x = current_robot_pose.pose.pose.position.x + current_step * cos(movement_dir);
            goal.pose.pose.position.y = current_robot_pose.pose.pose.position.y + current_step * sin(movement_dir);
            goal.pose.pose.orientation = Utils::createQuaternionMsgFromYaw(angles::normalize_angle(movement_dir));

            // reduce step (in case goal is an obstacle or out of bounds)
            if (current_step <= 0)
            {
                GSL_WARN("Cannot move further CrossWind!");
                setExplorationGoal();
                return;
            }
            current_step = current_step - 0.3;
        } while (!movingState->checkGoal(goal));

        dynamic_cast<MovingStatePlumeTracking*>(movingState.get())->currentMovement = PTMovement::RecoverPlume;
        movingState->sendGoal(goal);
    }

} // namespace GSL