#pragma once
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <rclcpp_action/rclcpp_action.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <angles/angles.h>

#include <olfaction_msgs/msg/gas_sensor_array.hpp>
#include <olfaction_msgs/msg/gas_sensor.hpp>
#include <olfaction_msgs/msg/anemometer.hpp>
#include "spdlog/spdlog.h"
#include "Utils/Utils.h"
#include "gsl_macros.h"

using Pose = geometry_msgs::msg::Pose;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using Point = geometry_msgs::msg::Point;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Marker = visualization_msgs::msg::Marker;

#ifdef USE_NAV_ASSISTANT

#include <nav_assistant_msgs/srv/make_plan.hpp>
#include <nav_assistant_msgs/action/nav_assistant.hpp>
using NavigateToPose = nav_assistant_msgs::action::NavAssistant;
using MakePlan = nav_assistant_msgs::srv::MakePlan;

#else

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using MakePlan = nav2_msgs::action::ComputePathToPose;

#endif

using NavigationClient = rclcpp_action::Client<NavigateToPose>::SharedPtr;

class GSLAlgorithm
{
public:
    GSLAlgorithm() = delete;
    GSLAlgorithm(std::shared_ptr<rclcpp::Node> _node);
    ~GSLAlgorithm();
    virtual void initialize();
    bool get_inMotion(); // True if we have a movement target
    virtual int checkSourceFound();
    void restart_search();
    double max_search_time;

    bool isPointInsideMapBounds(const Utils::Vector2& point) const;

protected:
    std::shared_ptr<rclcpp::Node> node;
    bool inMotion; //! Determines if a goal has been set and we are moving towards it
    std::string enose_topic, anemometer_topic, robot_location_topic, map_topic, costmap_topic;

    bool verbose;
    bool inExecution;
    rclcpp::Time start_time;
    double distance_found;
    std::vector<PoseWithCovarianceStamped> robot_poses_vector;
    double source_pose_x, source_pose_y;
    std::string results_file;
    std::string errors_file;
    std::string path_file;

    nav_msgs::msg::OccupancyGrid map_;            //! Map
    nav_msgs::msg::OccupancyGrid costmap_;        //! Map
    PoseWithCovarianceStamped movingPose;         //! Robot pose on the global frame referential
    PoseWithCovarianceStamped current_robot_pose; //! Robot pose on the global frame referential
    NavigationClient nav_client;                  //! Move Base Action Server.

    // Subscriptions
    rclcpp::Subscription<olfaction_msgs::msg::GasSensor>::SharedPtr gas_sub_;   //! Gas readings subscriber
    rclcpp::Subscription<olfaction_msgs::msg::Anemometer>::SharedPtr wind_sub_; //! Wind readings subscriber
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;     //! Map subscriber.
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_; //! CostMap subscriber.
    rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr localization_sub_;

#ifdef USE_NAV_ASSISTANT
    rclcpp::Client<MakePlan>::SharedPtr make_plan_client;
#else
    rclcpp_action::Client<MakePlan>::SharedPtr make_plan_client;
#endif

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    // CallBacks
    virtual void gasCallback(const olfaction_msgs::msg::GasSensor::SharedPtr msg) = 0;
    virtual void windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg) = 0;
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    virtual void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) = 0;
    void localizationCallback(const PoseWithCovarianceStamped::SharedPtr msg);

    virtual void goalDoneCallback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult& result);

    bool checkGoal(const NavigateToPose::Goal& goal);
    void sendGoal(const NavigateToPose::Goal& goal);
    float get_average_vector(std::vector<float> const& v);
    virtual void save_results_to_file(int result);
    virtual void declareParameters();

    template <typename T> T getParam(const std::string& name, T defaultValue)
    {
        if (node->has_parameter(name))
            return node->get_parameter_or<T>(name, defaultValue);
        else
            return node->declare_parameter<T>(name, defaultValue);
    }
};