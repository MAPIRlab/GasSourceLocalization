#include <gsl_server/algorithms/Spiral/Spiral.hpp>
#include <gsl_server/algorithms/Spiral/StopAndMeasureStateSpiral.hpp>
#include <gsl_server/Utils/RosUtils.hpp>

namespace GSL
{
    void Spiral::initialize()
    {
        Algorithm::initialize();
        resetSpiral();
        waitForMapState = std::make_unique<WaitForMapState>(this);
        waitForGasState = std::make_unique<WaitForGasState>(this);
        stopAndMeasureState = std::make_unique<StopAndMeasureStateSpiral>(this);
        movingState = std::make_unique<MovingState>(this);
        stateMachine.forceSetState(waitForMapState.get());
    }

    void Spiral::declareParameters()
    {
        Algorithm::declareParameters();

        initSpiralStepSize = getParam<double>("initial_step", 0.6);
        stepIncrement = getParam<double>("step_increment", 0.3);
        Kmu = getParam<double>("Kmu", 0.5);
        Kp = getParam<double>("Kp", 1);
    }

    void Spiral::processGasAndWindMeasurements(double concentration, double wind_speed, double wind_direction)
    {
        static double previousPI = 0;
        static int consecutive_misses = 0;
        double PI = getProximityIndex();

        if (PI > previousPI)
        {
            previousPI = PI;
            resetSpiral();
            GSL_WARN("NEW SPIRAL --> Proximity Index:{:.2f}", PI);
        }
        else
        {
            GSL_WARN("CONTINUE --> Proximity Index:{:.2f}", PI);
            if (consecutive_misses > 3)
            {
                previousPI = previousPI / 2.0;
                GSL_WARN("PI reduced to:{:.2f}", previousPI);

                consecutive_misses = 0;
            }
            else
                consecutive_misses++;
        }

        doSpiral();
    }

    void Spiral::resetSpiral()
    {
        spiralStepSize = initSpiralStepSize;
        spiral_iter = -1;
    }

    double Spiral::getProximityIndex()
    {
        auto stopAndMeasure = dynamic_cast<StopAndMeasureStateSpiral*>(stopAndMeasureState.get());
        double P = stopAndMeasure->getSumOfLocalMaxima();

        // Proximity Index (used to determine if we are getting closer to the source)
        return Kmu * stopAndMeasureState->average_concentration() + Kp * P * stopAndMeasure->intervalLength;
    }

    NavigateToPose::Goal Spiral::nextGoalSpiral(const Pose& initial)
    {
        double yaw = Utils::getYaw(initial.orientation);
        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = node->now();
        if (spiral_iter == -1)
        {
            goal.pose.pose.position.x = initial.position.x + spiralStepSize * cos(yaw) - spiralStepSize * sin(yaw);
            goal.pose.pose.position.y = initial.position.y + spiralStepSize * sin(yaw) + spiralStepSize * cos(yaw);
            goal.pose.pose.orientation = Utils::createQuaternionMsgFromYaw(yaw + M_PI / 4);
            spiral_iter = 1;
        }
        else
        {
            if (spiral_iter % 2 == 0)
            {
                spiralStepSize += spiralStep_increment;
            }
            goal.pose.pose.position.x = initial.position.x - (-spiralStepSize) * sin(yaw);
            goal.pose.pose.position.y = initial.position.y + (-spiralStepSize) * cos(yaw);
            goal.pose.pose.orientation = Utils::createQuaternionMsgFromYaw(yaw - M_PI / 2);
            spiral_iter++;
        }

        GSL_INFO("New Goal [{:.2f}, {:.2f}]", goal.pose.pose.position.x, goal.pose.pose.position.y);
        return goal;
    }

    void Spiral::doSpiral()
    {
        NavigateToPose::Goal goal = nextGoalSpiral(currentRobotPose.pose.pose);
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
                    goal.pose = getRandomPoseInMap();
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