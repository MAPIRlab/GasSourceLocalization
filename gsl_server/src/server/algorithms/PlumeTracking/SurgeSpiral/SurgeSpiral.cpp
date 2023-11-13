#include <gsl_server/algorithms/PlumeTracking/SurgeSpiral/SurgeSpiral.h>
#include <gsl_server/algorithms/PlumeTracking/MovingStatePlumeTracking.h>
#include <gsl_server/Utils/RosUtils.h>
#include <gsl_server/Utils/Math.h>
#include <angles/angles.h>

namespace GSL
{
    void SurgeSpiral::declareParameters()
    {
        PlumeTracking::declareParameters();
        initSpiralStepSize = getParam<double>("initSpiralStep", 1);
        spiralStep_increment = getParam<double>("spiralStep_increment", 0.4);
    }

    void SurgeSpiral::resetSpiral()
    {
        GSL_INFO("Resetting spiral movement");
        spiralStepSize = initSpiralStepSize;
        spiral_iter = 1;
    }

    NavigateToPose::Goal SurgeSpiral::nextGoalSpiral(Pose initial)
    {
        double yaw = Utils::getYaw(initial.orientation);
        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = node->now();
        if (spiral_iter % 2 == 0)
        {
            spiralStepSize += spiralStep_increment;
        }
        goal.pose.pose.position.x = initial.position.x - (-spiralStepSize) * sin(yaw);
        goal.pose.pose.position.y = initial.position.y + (-spiralStepSize) * cos(yaw);
        goal.pose.pose.orientation = Utils::createQuaternionMsgFromYaw(yaw - M_PI / 2);
        spiral_iter++;

        GSL_INFO("New Goal [{:.2}, {:.2}]", goal.pose.pose.position.x, goal.pose.pose.position.y);
        return goal;
    }

    void SurgeSpiral::setCastGoal(double downWind_direction)
    {
        // if entering spiral phase
        if (dynamic_cast<MovingStatePlumeTracking*>(movingState.get())->currentMovement != PTMovement::RecoverPlume)
            resetSpiral();

        NavigateToPose::Goal goal = nextGoalSpiral(current_robot_pose.pose.pose);
        int numberOfTries = 0;
        bool hasAlreadyReset = false;
        while (rclcpp::ok() && !movingState->checkGoal(goal))
        {
            GSL_INFO("SKIPPING NEXT POINT IN SPIRAL (OBSTACLES)");
            goal = nextGoalSpiral(goal.pose.pose);
            numberOfTries++;
            if (numberOfTries > 3)
            {
                numberOfTries = 0;
                if (hasAlreadyReset)
                {
                    setExplorationGoal();
                    return;
                }
                else
                {
                    GSL_INFO("UNABLE TO CONTINUE SPIRAL (OBSTACLES)");
                    resetSpiral();
                    hasAlreadyReset = true;
                }
            }
        }
        movingState->sendGoal(goal);
    }

} // namespace GSL