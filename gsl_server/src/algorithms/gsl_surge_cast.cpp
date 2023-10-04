#include <algorithms/gsl_surge_cast.h>

/*   DESCRIPTION OF SEARCH STATES
 * --------------------------------
 * WAITING_FOR_MAP: (initialization)No map is available, waiting to map_server to provide one.
 * EXPLORATION: Explore the map, looking for traces of the gas release.
 * INSPECTION: A gas hit has been detected, so search in the surroundings for the gas source.
 * PLUME (SURGE,CAT): A gas plume may exists, try navigating it towards the gas source.
 * STOP_AND_MEASURE: Stop the robot and measure wind and gas for a time-lapse.
 * */

//-----------------------------------------------------------
//						GSL-PlumeTracking
//-----------------------------------------------------------
SurgeCastPT::SurgeCastPT(std::shared_ptr<rclcpp::Node> _node) : PlumeTracking(_node)
{
}

SurgeCastPT::~SurgeCastPT()
{
    spdlog::info("- Closing...");
}

//----------------------------------------------

// Public Functions

//----------------------------------------------
void SurgeCastPT::initialize()
{
    PlumeTracking::initialize();
}

// We have wind and gas, move upwind (plume navigation)
void SurgeCastPT::setSurgeGoal()
{
    // Initially, get Upwind direction with respect reference /map
    double upwind_dir = angles::normalize_angle(average_wind_direction + 3.14159);
    // spdlog::info("[DEBUG] movement_dir in map frame = {}", movement_dir);

    // Set goal in the Upwind direction
    NavAssistant::Goal goal;
    current_step = step;
    double movement_dir = upwind_dir;
    do
    {
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = node->now();

        // Set a goal in the upwind direction
        goal.target_pose.pose.position.x = current_robot_pose.pose.pose.position.x + current_step * cos(movement_dir);
        goal.target_pose.pose.position.y = current_robot_pose.pose.pose.position.y + current_step * sin(movement_dir);
        goal.target_pose.pose.orientation = Utils::createQuaternionMsgFromYaw(angles::normalize_angle(movement_dir));

        // If goal is unreachable
        // add noise in angle (in case goal is an obstacle)
        movement_dir = upwind_dir + fRand(-1.6, 1.6);
        // reduce step
        current_step = current_step - 0.05;

        if (current_step <= 0.0)
        {
            spdlog::error("SurgeCastPT - {} - ERROR: Cannot move further Upwind!", __FUNCTION__);
            return;
        }
    } while (!checkGoal(goal));

    // Send goal to the Move_Base node for execution
    if (verbose)
        spdlog::debug("SurgeCastPT - {} - Sending robot to {} {}", __FUNCTION__, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    sendGoal(goal);
    inMotion = true;
}

void SurgeCastPT::setCastGoal()
{

    // Initially, get CrossWind direction with respect reference /map
    double movement_dir;
    if (cast_movement == 0)
    {
        // move right
        movement_dir = angles::normalize_angle(average_wind_direction + 3.14159 / 2);
        cast_movement = 1;
        current_step = step;
    }
    else if (cast_movement == 1)
    {
        // move left
        movement_dir = angles::normalize_angle(average_wind_direction - 3.14159 / 2);
        cast_movement = 2;
        current_step = 2 * step;
    }
    else
    {
        // end of cast
        if (verbose)
            spdlog::info("Gas plume completely Lost!");
        cancel_navigation(); // Stop Robot
        previous_state = current_state;
        current_state = PT_state::STOP_AND_MEASURE;
        if (verbose)
            spdlog::warn("[SurgeCastPT] New state --> STOP_AND_MEASURE");
        return;
    }

    // Set goal
    NavAssistant::Goal goal;
    do
    {
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = node->now();

        // Set a goal in the crosswind direction
        goal.target_pose.pose.position.x = current_robot_pose.pose.pose.position.x + current_step * cos(movement_dir);
        goal.target_pose.pose.position.y = current_robot_pose.pose.pose.position.y + current_step * sin(movement_dir);
        goal.target_pose.pose.orientation = Utils::createQuaternionMsgFromYaw(angles::normalize_angle(movement_dir));

        // spdlog::info("SurgeCastPT - {} - Testing {} {}...", __FUNCTION__, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);

        // reduce step (in case goal is an obstacle or out of bounds)
        current_step = current_step - 0.3;
        if (current_step <= 0)
        {
            if (verbose)
                spdlog::info("SurgeCastPT - {} - ERROR: Cannot move further CrossWind!", __FUNCTION__);
            return;
        }
    } while (!checkGoal(goal));

    // Send goal to the Move_Base node for execution
    if (verbose)
        spdlog::debug("SurgeCastPT - {} - Sending robot to {} {}", __FUNCTION__, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    sendGoal(goal);
    inMotion = true;
}

// Check gas observations to ensure we are in the correct search state,
// or prompt a new state transition
void SurgeCastPT::checkState()
{
    // If we are here, that means we are moving towards a target location reactively
    switch (current_state)
    {
    case PT_state::EXPLORATION:
        // We are looking for gas clues
        if (gasFound && get_average_vector(gasConcentration_v))
        {
            if (verbose)
                spdlog::info("GAS HIT!");
            gasHit = true;
            cancel_navigation(); // Stop Robot
            previous_state = current_state;
            current_state = PT_state::STOP_AND_MEASURE;
            if (verbose)
                spdlog::warn("[SurgeCastPT] New state --> STOP_AND_MEASURE");
        }
        break;
    case PT_state::INSPECTION:
        // Check surroundings of current position
        //  No break conditions, wait till end of inspection
        break;
    case PT_state::UPWIND_SURGE:
        // We are moving within the gas plume
        if (*max_element(gasConcentration_v.begin(), gasConcentration_v.end()) < th_gas_present)
        {
            if (verbose)
                spdlog::info("Gas plume lost!");
            cancel_navigation(); // Stop Robot
            previous_state = current_state;
            current_state = PT_state::STOP_AND_MEASURE;
            if (verbose)
                spdlog::warn("[SurgeCastPT] New state --> STOP_AND_MEASURE");
        }
        break;
    case PT_state::CROSSWIND_CAST:
        // We are trying to return to the plume
        if (get_average_vector(gasConcentration_v) >= th_gas_present)
        {
            if (verbose)
                spdlog::info("Gas plume found! - Returning to UPWIND_SURGE movement!");
            gasHit = true;
            cancel_navigation(); // Stop Robot
            previous_state = current_state;
            current_state = PT_state::STOP_AND_MEASURE;
            if (verbose)
                spdlog::warn("[SurgeCastPT] New state --> STOP_AND_MEASURE");
        }
        break;
    default:
        spdlog::error("ERROR: State undefined!");
    }
}

void SurgeCastPT::save_results_to_file(int result)
{
    nav_client->async_cancel_all_goals();

    // 1. Search time.
    rclcpp::Duration time_spent = node->now() - start_time;
    double search_t = time_spent.seconds();

    std::ofstream file;
    file.open(results_file, std::ios_base::app);

    file << search_t << "\n\n\n";
    for (PoseWithCovarianceStamped p : robot_poses_vector)
    {
        file << p.pose.pose.position.x << ", " << p.pose.pose.position.y << "\n";
    }
}

// EOF
