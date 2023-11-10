#include <gsl_server/algorithms/Algorithm.h>
#include <gsl_server/Utils/Math.h>
#include <gsl_server/Utils/RosUtils.h>
#include <angles/angles.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace GSL
{

    Algorithm::Algorithm(std::shared_ptr<rclcpp::Node> _node) : node(_node), tf_buffer(node->get_clock())
    {
    }

    void Algorithm::initialize()
    {
        declareParameters();

        // Subscribers
        //------------
        using namespace std::placeholders;
        gas_sub = node->create_subscription<olfaction_msgs::msg::GasSensor>(getParam<std::string>("enose_topic", "PID/Sensor_reading"), 1,
                                                                            std::bind(&Algorithm::gasCallback, this, _1));

        wind_sub = node->create_subscription<olfaction_msgs::msg::Anemometer>(
            getParam<std::string>("anemometer_topic", "Anemometer/WindSensor_reading"), 1, std::bind(&Algorithm::windCallback, this, _1));

        localization_sub = node->create_subscription<PoseWithCovarianceStamped>(getParam<std::string>("robot_location_topic", "amcl_pose"), 100,
                                                                                std::bind(&Algorithm::localizationCallback, this, _1));

        GSL_INFO("INITIALIZATON COMPLETED");
    }

    void Algorithm::declareParameters()
    {
        resultLogging.max_search_time = getParam<double>("max_search_time", 1000.0);
        resultLogging.distance_found = getParam<double>("distance_found", 0.5);
        resultLogging.source_pose_x = getParam<double>("ground_truth_x", 0.0);
        resultLogging.source_pose_y = getParam<double>("ground_truth_y", 0.0);
        resultLogging.results_file = getParam<std::string>("results_file", "");
        resultLogging.errors_file = getParam<std::string>("errors_file", "");
        resultLogging.path_file = getParam<std::string>("path_file", "");

        thresholdGas = getParam<double>("th_gas_present", 0.1);
        thresholdWind = getParam<double>("th_wind_present", 0.1);
    }

    void Algorithm::OnUpdate()
    {
        rclcpp::spin_some(node);
        stateMachine.getCurrentState()->OnUpdate();
    }

    bool Algorithm::hasEnded()
    {
        return getResult() != GSLResult::Running;
    }

    GSLResult Algorithm::getResult()
    {
        return currentResult;
    }

    void Algorithm::localizationCallback(const PoseWithCovarianceStamped::SharedPtr msg)
    {
        // keep the most recent robot pose
        current_robot_pose = *msg;

        // Keep all poses for later distance estimation
        resultLogging.robot_poses_vector.push_back(current_robot_pose);
    }

    void Algorithm::onGetMap(const OccupancyGrid::SharedPtr msg)
    {
        map = *msg;

        {
            GSL_INFO("--------------GSL---------------"
                     "Occupancy Map dimensions:"
                     "x_min:{:.2} x_max:{:.2}   -   y_min:{:.2} y_max:{:.2}"
                     "--------------------------------",
                     map.info.origin.position.x, map.info.origin.position.x + map.info.width * map.info.resolution, map.info.origin.position.y,
                     map.info.origin.position.y + map.info.height * map.info.resolution);
        }
    }

    void Algorithm::onGetCostMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        costmap = *msg;
    }

    void Algorithm::OnCompleteNavigation(GSLResult result)
    {
        stateMachine.forceResetState(stopAndMeasureState.get());
    }

    GSLResult Algorithm::checkSourceFound()
    {
        // 1. Check that working time < max allowed time for search
        rclcpp::Duration time_spent = node->now() - start_time;
        if (time_spent.seconds() > resultLogging.max_search_time)
        {
            // Report failure, we were too slow
            GSL_INFO("FAILURE-> Time spent ({} s) > max_search_time = {}", time_spent.seconds(), resultLogging.max_search_time);
            save_results_to_file(GSLResult::Failure);
            return GSLResult::Failure;
        }

        // 2. Distance from robot to source
        double Ax = current_robot_pose.pose.pose.position.x - resultLogging.source_pose_x;
        double Ay = current_robot_pose.pose.pose.position.y - resultLogging.source_pose_y;
        double dist = sqrt(pow(Ax, 2) + pow(Ay, 2));
        if (dist < resultLogging.distance_found)
        {
            // GSL has finished with success!
            GSL_INFO("SUCCESS -> Time spent ({} s)", time_spent.seconds());
            save_results_to_file(GSLResult::Success);
            return GSLResult::Success;
        }

        // In other case, we are still searching (keep going)
        return GSLResult::Running;
    }

    void Algorithm::save_results_to_file(GSLResult result)
    {
        rclcpp::Duration time_spent = node->now() - start_time;
        double search_t = time_spent.seconds();

        // 2. Search distance
        // Get distance from array of path followed (vector of PoseWithCovarianceStamped
        double search_d = 0;

        const auto& robot_poses_vector = resultLogging.robot_poses_vector;
        for (size_t h = 1; h < robot_poses_vector.size(); h++)
        {
            double Ax = robot_poses_vector[h - 1].pose.pose.position.x - robot_poses_vector[h].pose.pose.position.x;
            double Ay = robot_poses_vector[h - 1].pose.pose.position.y - robot_poses_vector[h].pose.pose.position.y;
            search_d += sqrt(pow(Ax, 2) + pow(Ay, 2));
        }

        // 3. Navigation distance (from robot to source)
        // Estimate the distances by getting a navigation path from Robot initial pose to Source points in the map
        double nav_d = 0;
        {
            PoseStamped source_pose;
            source_pose.header.frame_id = "map";
            source_pose.header.stamp = node->now();
            source_pose.pose.position.x = resultLogging.source_pose_x;
            source_pose.pose.position.y = resultLogging.source_pose_y;

            PoseStamped startPose;
            startPose.pose = resultLogging.robot_poses_vector[0].pose.pose;
            startPose.header = resultLogging.robot_poses_vector[0].header;
            std::optional<nav_msgs::msg::Path> plan = movingState->GetPlan(startPose, source_pose);
            if (plan)
            {
                const auto& path = plan.value().poses;
                for (int h = 1; h < path.size(); h++)
                {
                    double Ax = path[h - 1].pose.position.x - path[h].pose.position.x;
                    double Ay = path[h - 1].pose.position.y - path[h].pose.position.y;
                    nav_d += sqrt(pow(Ax, 2) + pow(Ay, 2));
                }
            }
            else
                GSL_ERROR("Could not calculate minimal navigation distance for logging results");
        }

        // 4. Nav time
        double nav_t = nav_d / 0.4; // assumming a constant speed of 0.4m/s
        std::string result_string =
            fmt::format("RESULT IS: Success={}, Search_d={}, Nav_d={}, Search_t={}, Nav_t={}\n", (int)result, search_d, nav_d, search_t, nav_t);
        GSL_INFO(result_string);

        if (FILE* output_file = fopen(resultLogging.results_file.c_str(), "w"))
        {
            fprintf(output_file, "%s", result_string.c_str());
            for (PoseWithCovarianceStamped p : robot_poses_vector)
            {
                fprintf(output_file, "%f, %f\n", p.pose.pose.position.x, p.pose.pose.position.y);
            }
            fclose(output_file);
        }
        else
            GSL_ERROR("Unable to open Results file at: {}", resultLogging.results_file.c_str());
    }

    bool Algorithm::isPointInsideMapBounds(const Vector2& point) const
    {
        const static Vector2 mapStart = {map.info.origin.position.x, map.info.origin.position.y};
        const static Vector2 mapEnd = mapStart + Vector2(map.info.width, map.info.height) * map.info.resolution;

        return point.x >= mapStart.x && point.x < mapEnd.x && point.y >= mapStart.y && point.y < mapEnd.y;
    }

    float Algorithm::ppmFromGasMsg(const olfaction_msgs::msg::GasSensor::SharedPtr msg)
    {
        // extract ppm reading from MOX sensors
        if (msg->raw_units == msg->UNITS_OHM)
        {
            double rs_r0 = msg->raw / 50000.0;
            return std::pow(rs_r0 / msg->calib_a, 1.0 / msg->calib_b);
        }
        else if (msg->raw_units == msg->UNITS_PPM)
        {
            return msg->raw;
        }
        else
        {
            GSL_ERROR("Unknown concentration unit: {}", (int)msg->raw_units);
            return 0;
        }
    }

    void Algorithm::gasCallback(const olfaction_msgs::msg::GasSensor::SharedPtr msg)
    {
        float ppm = ppmFromGasMsg(msg);
        stopAndMeasureState->addGasReading(ppm);
    }

    void Algorithm::windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg)
    {
        float downWind_direction = angles::normalize_angle(msg->wind_direction);
        // Transform from anemometer ref_system to map ref_system using TF
        PoseStamped anemometer_downWind_pose, map_downWind_pose;
        try
        {
            anemometer_downWind_pose.header.frame_id = msg->header.frame_id;
            anemometer_downWind_pose.pose.position.x = 0.0;
            anemometer_downWind_pose.pose.position.y = 0.0;
            anemometer_downWind_pose.pose.position.z = 0.0;
            anemometer_downWind_pose.pose.orientation = Utils::createQuaternionMsgFromYaw(downWind_direction);

            map_downWind_pose = tf_buffer.buffer.transform(anemometer_downWind_pose, "map");
        }
        catch (tf2::TransformException& ex)
        {
            GSL_ERROR("{} - Error: {}", __FUNCTION__, ex.what());
            return;
        }

        stopAndMeasureState->addWindReading(msg->wind_speed, Utils::getYaw(map_downWind_pose.pose.orientation));
    }

} // namespace GSL