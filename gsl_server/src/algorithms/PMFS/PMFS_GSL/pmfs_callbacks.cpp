#include <algorithms/PMFS/PMFS.h>

namespace PMFS{

//------------------
    //CALLBACKS
//------------------

void PMFS_GSL::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    if (current_state!=State::WAITING_FOR_MAP){
        return;
    }
    
    //wait until we know where the robot is
    //I know this looks wrong, but the way ros callbacks block (or dont) the execution is really counterintuitive, so it will actually work
    rclcpp::Rate rate(1);
    while(robot_poses_vector.size()==0){
        rate.sleep();
    }

    //ROS convention is to consider cell [0,0] as the lower-left corner (see http://docs.ros.org/api/nav_msgs/html/msg/MapMetaData.html)
    map_ = *msg;

    if (verbose) spdlog::info("--------------GSL---------------");
    if (verbose) spdlog::info("Occupancy Map dimensions:");
    if (verbose) spdlog::info("x_min:{:.2} x_max:{:.2}   -   y_min:{:.2} y_max:{:.2}",map_.info.origin.position.x, map_.info.origin.position.x+map_.info.width*map_.info.resolution, map_.info.origin.position.y,map_.info.origin.position.y+map_.info.height*map_.info.resolution);
    if (verbose) spdlog::info("--------------------------------");

    current_state = State::INITIALIZING;
}


void PMFS_GSL::gasCallback(const olfaction_msgs::msg::GasSensor::SharedPtr msg)
{

    //Only if we are in the Stop_and_Measure
    if (current_state == State::STOP_AND_MEASURE)
    {
        stop_and_measure_gas_v.push_back(msg->raw);
    }
    last_concentration_reading = msg->raw;
}

void PMFS_GSL::windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg)
{
    //1. Add obs to the vector of the last N wind speeds
    float downWind_direction = angles::normalize_angle(msg->wind_direction);
    //Transform from anemometer ref_system to map ref_system using TF
    geometry_msgs::msg::PoseStamped anemometer_upWind_pose, map_upWind_pose;
    try
    {
        anemometer_upWind_pose.header.frame_id = msg->header.frame_id;
        anemometer_upWind_pose.pose.position.x = 0.0;
        anemometer_upWind_pose.pose.position.y = 0.0;
        anemometer_upWind_pose.pose.position.z = 0.0;
        anemometer_upWind_pose.pose.orientation = Utils::createQuaternionMsgFromYaw(downWind_direction);

        tf_buffer->transform(anemometer_upWind_pose, map_upWind_pose, "map");
    }
    catch(tf2::TransformException &ex)
    {
        spdlog::error("SurgeCastPT - {} - Error: {}", __FUNCTION__, ex.what());
        return;
    }

    //Only if we are in the Stop_and_Measure
    if (current_state == State::STOP_AND_MEASURE)
    {
        stop_and_measure_windS_v.push_back(msg->wind_speed);
        stop_and_measure_windD_v.push_back(angles::normalize_angle(Utils::getYaw(map_upWind_pose.pose.orientation)+M_PI));
    }

}

AveragedMeasurement PMFS_GSL::getAveragedMeasurement()
{
    //Get averaged values of the observations taken while standing
    //Wind direction is reported as DownWind in the map frame_id
    //Being positive to the right, negative to the left, range [-pi,pi]
    
    float average_concentration = *std::max_element(stop_and_measure_gas_v.begin(), stop_and_measure_gas_v.end());
    float average_wind_direction = AveragedMeasurement::get_average_wind_direction(stop_and_measure_windD_v);
    float average_wind_speed = get_average_vector(stop_and_measure_windS_v);

    stop_and_measure_gas_v.clear();
    stop_and_measure_windS_v.clear();

    //Check thresholds and set new search-state
    if (verbose) spdlog::info("avg_gas={:.2}    avg_wind_speed={:.2}     avg_wind_dir={:.2}",
                average_concentration,
                average_wind_speed,
                average_wind_direction);

    return {average_wind_speed, average_wind_direction, average_concentration};
}

float AveragedMeasurement::get_average_wind_direction(std::vector<float> const &v)
{
    //Average of wind direction, avoiding the problems of +/- pi angles.
    float x =0.0, y = 0.0;
    for(std::vector<float>::const_iterator i = v.begin(); i != v.end(); ++i)
    {
        x += cos(*i);
        y += sin(*i);
    }
    float average_angle = atan2(y, x);   


    return average_angle;
}

void PMFS_GSL::goalDoneCallback(const rclcpp_action::ClientGoalHandle<NavAssistant>::WrappedResult& result)
{
    bool succeeded;
    if(result.code != rclcpp_action::ResultCode::SUCCEEDED)
    {
        succeeded = true;
    }
    else{
        succeeded = false;
        spdlog::error("PlumeTracking - {} - UPS! Couldn't reach the target.", __FUNCTION__);
    }

    
    //Notify that the objective has been reached
    cancel_navigation(succeeded);
}

}