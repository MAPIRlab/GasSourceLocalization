#include <algorithms/gsl_plume_tracking.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::placeholders;

PlumeTracking::PlumeTracking(std::shared_ptr<rclcpp::Node> _node) : GSLAlgorithm(_node)
{
}

void PlumeTracking::initialize()
{
    GSLAlgorithm::initialize();
    // Subscribers
    //------------

    constexpr int size_vector_moving_measurements = 40;
    // Init variables
    gasConcentration_v.resize(size_vector_moving_measurements, 0.0);
    windSpeed_v.resize(size_vector_moving_measurements, 0.0);
    windDirection_v.resize(size_vector_moving_measurements, 0.0);

    // Init State
    previous_state = PT_state::WAITING_FOR_MAP;
    current_state = PT_state::WAITING_FOR_MAP;
    spdlog::info("INITIALIZATON COMPLETED--> WAITING_FOR_MAP");
    gasHit = false;
    gasFound = false;

    recoveryTimestamp = node->now();
    movingTimestamp = node->now();
}

void PlumeTracking::declareParameters()
{
    GSLAlgorithm::declareParameters();
    th_gas_present = getParam<double>("th_gas_present", 0.3);
    th_wind_present = getParam<double>("th_wind_present", 0.0);
    stop_and_measure_time = getParam<double>("stop_and_measure_time", 2);
    inspection_radius = getParam<double>("inspection_radius", 1.0);
    step = getParam<double>("step", 1.0);
    timeout_cast = getParam<double>("timeout_cast", 60.0);
}

PlumeTracking::~PlumeTracking()
{
    spdlog::info("- Closing...");
}

//------------------------

// CallBack functions

//------------------------
void PlumeTracking::gasCallback(const olfaction_msgs::msg::GasSensor::SharedPtr msg)
{
    static std::vector<float>::iterator gas_it = gasConcentration_v.begin();

    // spdlog::info("PlumeTracking - {} - Got a new gas observation!", __FUNCTION__);

    //[always] Add obs to the vector of the last N gas concentrations
    *gas_it = msg->raw;
    gas_it++;
    if (gas_it == gasConcentration_v.end())
        gas_it = gasConcentration_v.begin();

    // Only if we are in the Stop_and_Measure
    if (this->current_state == PT_state::STOP_AND_MEASURE)
    {
        stop_and_measure_gas_v.push_back(msg->raw);
    }
    if (current_state != PT_state::WAITING_FOR_MAP && !gasFound && msg->raw > th_gas_present)
    {
        gasFound = true;
        if (verbose)
            spdlog::info("GAS HIT!");
        gasHit = true;
        cancel_navigation(); // Stop Robot
        previous_state = current_state;
        current_state = PT_state::STOP_AND_MEASURE;
        if (verbose)
            spdlog::warn("[SurgeCastPT] New state --> STOP_AND_MEASURE");
    }
}

void PlumeTracking::windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg)
{
    static std::vector<float>::iterator windS_it = windSpeed_v.begin();
    static std::vector<float>::iterator windD_it = windDirection_v.begin();
    // 1. Add obs to the vector of the last N wind speeds
    *windS_it = msg->wind_speed;

    // 2. Add obs to the vector of the last N wind directions
    /*
      Wind direction is reported by the direction from which it originates
      That is, upwind in the anemometer reference system
      Being positive to the right, negative to the left, range [-pi,pi] (This is contrary to Pose conventions!!)
      Instead, we store the downWind direction following the ROS convention positive to the left, negative to the right (right hand rule), range
      [-pi,pi]
    */
    float downWind_direction = angles::normalize_angle(msg->wind_direction + M_PI);
    // Transform from anemometer ref_system to map ref_system using TF
    geometry_msgs::msg::PoseStamped anemometer_downWind_pose, map_downWind_pose;
    try
    {
        anemometer_downWind_pose.header.frame_id = msg->header.frame_id;
        anemometer_downWind_pose.pose.position.x = 0.0;
        anemometer_downWind_pose.pose.position.y = 0.0;
        anemometer_downWind_pose.pose.position.z = 0.0;
        anemometer_downWind_pose.pose.orientation = Utils::createQuaternionMsgFromYaw(downWind_direction);

        map_downWind_pose = tf_buffer->transform(anemometer_downWind_pose, "map");
    }
    catch (tf2::TransformException& ex)
    {
        spdlog::error("SurgeCastPT - {} - Error: {}", __FUNCTION__, ex.what());
        return;
    }

    *windD_it = Utils::getYaw(map_downWind_pose.pose.orientation);
    //*windD_it = msg->wind_direction;

    // Update iterators
    windS_it++;
    if (windS_it == windSpeed_v.end())
        windS_it = windSpeed_v.begin();
    windD_it++;
    if (windD_it == windDirection_v.end())
        windD_it = windDirection_v.begin();

    // Only if we are in the Stop_and_Measure
    if (this->current_state == PT_state::STOP_AND_MEASURE)
    {
        stop_and_measure_windS_v.push_back(msg->wind_speed);
        stop_and_measure_windD_v.push_back(Utils::getYaw(map_downWind_pose.pose.orientation));
    }
}

void PlumeTracking::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_ = *msg;

    if (verbose)
    {
        spdlog::info("--------------GSL---------------");
        spdlog::info("Occupancy Map dimensions:");
        spdlog::info("x_min:{:.2} x_max:{:.2}   -   y_min:{:.2} y_max:{:.2}", map_.info.origin.position.x,
                     map_.info.origin.position.x + map_.info.width * map_.info.resolution, map_.info.origin.position.y,
                     map_.info.origin.position.y + map_.info.height * map_.info.resolution);
        spdlog::info("--------------------------------");
    }

    // Start the fun!!
    cancel_navigation();
    previous_state = current_state;
    current_state = PT_state::STOP_AND_MEASURE;
    start_time = node->now();   // start measuring time
    robot_poses_vector.clear(); // start measuring distance
    inExecution = true;
    inMotion = false;
    spdlog::warn("STARTING THE SEARCH --> STOP_AND_MEASURE");
}

// Move Base CallBacks
void PlumeTracking::goalDoneCallback(const rclcpp_action::ClientGoalHandle<NavAssistant>::WrappedResult& result)
{
    inMotion = false;
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        spdlog::debug("PlumeTracking - {} - Target achieved!", __FUNCTION__);
        // This makes the search slower, but more robust!
        if (current_state != PT_state::CROSSWIND_CAST)
        {
            cancel_navigation();
            previous_state = current_state;
            current_state = PT_state::STOP_AND_MEASURE;
            spdlog::warn("New state --> STOP_AND_MEASURE");
        }
    }
    else
        spdlog::debug("PlumeTracking - {} - UPS! Couldn't reach the target.", __FUNCTION__);
}

//----------------------------------------------

// Public Functions

//----------------------------------------------
PT_state PlumeTracking::get_state()
{
    return current_state;
}

// Stop the robot
void PlumeTracking::cancel_navigation()
{
    nav_client->async_cancel_all_goals(); // Cancel current navigations
    inMotion = false;
    current_step = step; // set max step allowed for surge/cast targets
    cast_movement = 0;   // for crosswind movement

    // Start a new measurement-phase while standing
    stop_and_measure_gas_v.clear();
    stop_and_measure_windS_v.clear();
    stop_and_measure_windD_v.clear();
    time_stopped = node->now(); // Start timer for initial wind measurement
}

// Stop the robot and measure gas and wind for X seconds (standing)
// Then switch to corresponding search state
void PlumeTracking::getGasWindObservations()
{

    if ((node->now() - time_stopped).seconds() >= stop_and_measure_time)
    {
        movingTimestamp = node->now();
        movingPose = current_robot_pose;
        recoveryTimestamp = node->now();

        // Get averaged values of the observations taken while standing
        // Wind direction is reported as DownWind in the map frame_id
        // Being positive to the right, negative to the left, range [-pi,pi]
        average_concentration = get_average_vector(stop_and_measure_gas_v);
        average_wind_direction = get_average_wind_direction(stop_and_measure_windD_v);
        average_wind_spped = get_average_vector(stop_and_measure_windS_v);

        // Check thresholds and set new search-state
        if (verbose)
            spdlog::info("avg_gas={:.2}    avg_wind_speed={:.2}     avg_wind_dir={:.2}", average_concentration, average_wind_spped,
                         average_wind_direction);

        if (average_concentration > th_gas_present && average_wind_spped > th_wind_present)
        {
            // Gas & wind
            previous_state = current_state;
            current_state = PT_state::UPWIND_SURGE;
            if (verbose)
                spdlog::warn("New state --> UPWIND_SURGE");
        }
        else if (average_concentration > th_gas_present)
        {
            // Only gas
            previous_state = current_state;
            current_state = PT_state::INSPECTION;
            inspection_iter = 0;
            if (verbose)
                spdlog::warn("New state --> INSPECTION");
        }
        else if (average_wind_spped > th_wind_present)
        {
            // Only Wind
            if (previous_state == PT_state::UPWIND_SURGE)
            {
                previous_state = current_state;
                current_state = PT_state::CROSSWIND_CAST;
                if (verbose)
                    spdlog::warn("New state --> CROSSWIND_CAST");
            }
            else
            {
                if (gasHit)
                {
                    previous_state = current_state;
                    current_state = PT_state::CROSSWIND_CAST;
                    if (verbose)
                        spdlog::warn("New state --> CROSSWIND_CAST");
                    gasHit = false;
                }
                else
                {
                    previous_state = current_state;
                    current_state = PT_state::EXPLORATION;
                    if (verbose)
                        spdlog::warn("New state --> EXPLORATION");
                }
            }
        }
        else
        {
            // Nothing
            previous_state = current_state;
            current_state = PT_state::EXPLORATION;
            if (verbose)
                spdlog::warn("New state --> EXPLORATION - no gas or wind found");
        }
    }
}

// Set a target within the map to search gas clues
void PlumeTracking::setExplorationGoal()
{
    if (gasFound)
    {
        setRandomGoal();
    }
}

// Set up to 4 different goals to inspect area close to current location
void PlumeTracking::setInspectionGoal()
{
    if (inspection_iter < 4)
    {
        NavAssistant::Goal goal;
        current_step = inspection_radius; // meters
        do
        {
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = node->now();
            switch (inspection_iter)
            {
            case 0: //(x+1,y)
                goal.target_pose.pose.position.x = current_robot_pose.pose.pose.position.x + current_step;
                goal.target_pose.pose.position.y = current_robot_pose.pose.pose.position.y;
                goal.target_pose.pose.orientation = Utils::createQuaternionMsgFromYaw(3 * M_PI / 4);
                break;
            case 1: //(x,y+1)
                goal.target_pose.pose.position.x = current_robot_pose.pose.pose.position.x - current_step;
                goal.target_pose.pose.position.y = current_robot_pose.pose.pose.position.y + current_step;
                goal.target_pose.pose.orientation = Utils::createQuaternionMsgFromYaw(-3 * M_PI / 4);
                break;
            case 2: //(x-1,y)
                goal.target_pose.pose.position.x = current_robot_pose.pose.pose.position.x - current_step;
                goal.target_pose.pose.position.y = current_robot_pose.pose.pose.position.y - current_step;
                goal.target_pose.pose.orientation = Utils::createQuaternionMsgFromYaw(-M_PI / 4);
                break;
            case 3: //(x,y-1)
                goal.target_pose.pose.position.x = current_robot_pose.pose.pose.position.x + current_step;
                goal.target_pose.pose.position.y = current_robot_pose.pose.pose.position.y - current_step;
                goal.target_pose.pose.orientation = Utils::createQuaternionMsgFromYaw(M_PI / 4);
                break;
            default:
                spdlog::warn("ERROR IN SEARCH-STATE INSPECTION");
            }

            // reduce step (in case goal is an obstacle)
            current_step = current_step - 0.1;
        } while (!checkGoal(goal));

        // Send goal to the Move_Base node for execution
        if (verbose)
            spdlog::debug("INSPECTION - Sending robot to {} {}", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);

        sendGoal(goal);
        inMotion = true;
        inspection_iter++;
    }
    else
    {
        // End of inspection, return to Stop&Measure
        cancel_navigation();
        previous_state = current_state;
        current_state = PT_state::STOP_AND_MEASURE;
        if (verbose)
            spdlog::warn("New state --> STOP_AND_MEASURE");
    }
}

// Set a random goal within the map (EXPLORATION)
void PlumeTracking::setRandomGoal()
{
    NavAssistant::Goal goal;
    goal.target_pose = get_random_pose_environment();

    // Send goal to the Move_Base node for execution
    if (verbose)
        spdlog::info("- {} - Sending robot to {} {}", __FUNCTION__, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    sendGoal(goal);
    inMotion = true;
}

geometry_msgs::msg::PoseStamped PlumeTracking::get_random_pose_environment()
{
    int idx = 0;
    NavAssistant::Goal goal;
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp = node->now();
    double randomPoseDistance = 1;
    do
    {
        p.pose.position.x =
            fRand(current_robot_pose.pose.pose.position.x - randomPoseDistance, current_robot_pose.pose.pose.position.x + randomPoseDistance);
        p.pose.position.y =
            fRand(current_robot_pose.pose.pose.position.y - randomPoseDistance, current_robot_pose.pose.pose.position.y + randomPoseDistance);
        p.pose.orientation = Utils::createQuaternionMsgFromYaw(0.0);
        if (idx % 5 == 0)
        {
            randomPoseDistance += 0.5;
        }
        idx++;
        goal.target_pose = p;
    } while (!checkGoal(goal));

    // show content
    if (verbose)
        spdlog::info("[Plume-Tracking] Random Goal pose =[{:.2}, {:.2}, {:.2}]", p.pose.position.x, p.pose.position.y, p.pose.orientation.z);
    return p;
}

float PlumeTracking::get_average_wind_direction(std::vector<float> const& v)
{
    // Average of wind direction, avoiding the problems of +/- pi angles.
    float x = 0.0, y = 0.0;
    for (std::vector<float>::const_iterator i = v.begin(); i != v.end(); ++i)
    {
        x += cos(*i);
        y += sin(*i);
    }
    float average_angle = atan2(y, x);

    return average_angle;
}

// Get random number (double)
double PlumeTracking::fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}
