#include <gsl_server/algorithms/PlumeTracking/PlumeTracking.hpp>
#include <gsl_server/algorithms/PlumeTracking/MovingStatePlumeTracking.hpp>
#include <gsl_server/Utils/RosUtils.hpp>
#include <gsl_server/Utils/Math.hpp>
#include <angles/angles.h>

namespace GSL
{
    static constexpr int sizeOfGasMsgBuffer = 10;
    void PlumeTracking::initialize()
    {
        Algorithm::initialize();
        lastConcentrationReadings.resize(sizeOfGasMsgBuffer);

        waitForMapState = std::make_unique<WaitForMapState>(this);
        waitForGasState = std::make_unique<WaitForGasState>(this);
        stopAndMeasureState = std::make_unique<StopAndMeasureState>(this);
        movingState = std::make_unique<MovingStatePlumeTracking>(this);
        stateMachine.forceSetState(waitForMapState.get());
        start_time = node->now();
    }

    void PlumeTracking::declareParameters()
    {
        Algorithm::declareParameters();
        surgeStepSize = getParam<double>("step", 1);
    }
    
    void PlumeTracking::OnUpdate()
    {
        Algorithm::OnUpdate();
        updateProximityResults();
    }

    void PlumeTracking::processGasAndWindMeasurements(double concentration, double wind_speed, double wind_direction)
    {
        if (concentration > thresholdGas && wind_speed > thresholdWind)
        {
            setSurgeGoal(wind_direction);
        }
        else if (wind_speed > thresholdWind)
        {
            if (dynamic_cast<MovingStatePlumeTracking*>(movingState.get())->currentMovement == PTMovement::FollowPlume)
                setCastGoal(wind_direction);

            else
                setExplorationGoal();
        }
        else
        {
            setExplorationGoal();
        }
    }

    void PlumeTracking::setSurgeGoal(double upwind_dir)
    {
        GSL_INFO_COLOR(fmt::terminal_color::yellow, "Surge");

        // Set goal in the Upwind direction
        NavigateToPose::Goal goal;
        double current_step = surgeStepSize;

        constexpr int safetyLimit = 10;
        for(int i = 0; i<safetyLimit;i++)
        {
            goal.pose.header.frame_id = "map";
            goal.pose.header.stamp = node->now();

            goal.pose.pose.position.x = currentRobotPose.pose.pose.position.x + current_step * cos(upwind_dir);
            goal.pose.pose.position.y = currentRobotPose.pose.pose.position.y + current_step * sin(upwind_dir);
            goal.pose.pose.orientation = Utils::createQuaternionMsgFromYaw(angles::normalize_angle(upwind_dir));

            if (current_step <= 0.0)
            {
                GSL_ERROR("PlumeTracking - Cannot move further Upwind!");
                setExplorationGoal();
                return;
            }
            current_step = current_step - 0.05;
            if(movingState->checkGoal(goal))
                break;
        }

        // Send goal to the Move_Base node for execution
        dynamic_cast<MovingStatePlumeTracking*>(movingState.get())->currentMovement = PTMovement::FollowPlume;
        movingState->sendGoal(goal);
    }

    void PlumeTracking::setExplorationGoal()
    {
        GSL_INFO_COLOR(fmt::terminal_color::yellow, "Exploration");
        NavigateToPose::Goal goal;
        goal.pose = getRandomPoseInMap();

        dynamic_cast<MovingStatePlumeTracking*>(movingState.get())->currentMovement = PTMovement::Exploration;
        movingState->sendGoal(goal);
    }

    float PlumeTracking::gasCallback(const olfaction_msgs::msg::GasSensor::SharedPtr msg)
    {
        float ppm = Algorithm::gasCallback(msg);
        if (lastConcentrationReadings.size() >= sizeOfGasMsgBuffer)
            lastConcentrationReadings.pop_front();

        lastConcentrationReadings.push_back(ppm);
        return ppm;
    }

} // namespace GSL