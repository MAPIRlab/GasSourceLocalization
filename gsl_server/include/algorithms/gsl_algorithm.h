#pragma once
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_assistant_msgs/srv/make_plan.hpp>
#include <nav_assistant_msgs/action/nav_assistant.hpp>

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

typedef geometry_msgs::msg::Pose Pose;
typedef geometry_msgs::msg::PoseWithCovarianceStamped PoseWithCovarianceStamped;
typedef geometry_msgs::msg::Point Point;
typedef nav_assistant_msgs::action::NavAssistant NavAssistant;
typedef rclcpp_action::Client<NavAssistant>::SharedPtr NavAssistClient;
typedef nav_assistant_msgs::srv::MakePlan MakePlan;

using MarkerArray=visualization_msgs::msg::MarkerArray;
using Marker=visualization_msgs::msg::Marker;

class GSLAlgorithm
{
    public:
        GSLAlgorithm() = delete;
        GSLAlgorithm(std::shared_ptr<rclcpp::Node> _node);
        ~GSLAlgorithm();
        bool get_inMotion();                //True if we have a movement target
        virtual int checkSourceFound();
        void restart_search();
        double max_search_time;

        bool isPointInsideMapBounds(const Utils::Vector2& point) const;
        
    protected:
        std::shared_ptr<rclcpp::Node> node;
        bool inMotion;                                                      //! Determines if a goal has been set and we are moving towards it
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
    
        nav_msgs::msg::OccupancyGrid map_;                                       //! Map
        nav_msgs::msg::OccupancyGrid costmap_;                                       //! Map
        PoseWithCovarianceStamped movingPose;        //! Robot pose on the global frame referential
        PoseWithCovarianceStamped current_robot_pose;        //! Robot pose on the global frame referential
        NavAssistClient nav_client;                                               //! Move Base Action Server.


        //Subscriptions
        rclcpp::Subscription<olfaction_msgs::msg::GasSensor>::SharedPtr gas_sub_;                                           //! Gas readings subscriber
        rclcpp::Subscription<olfaction_msgs::msg::Anemometer>::SharedPtr wind_sub_;                                          //! Wind readings subscriber
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;                                           //! Map subscriber.
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;                                       //! CostMap subscriber.
        rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr localization_sub_;
        rclcpp::Client<MakePlan>::SharedPtr make_plan_client;
        
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;


        //CallBacks
        virtual void gasCallback(const olfaction_msgs::msg::GasSensor::SharedPtr msg)=0;
        virtual void windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg)=0;
        void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        virtual void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)=0;
        void localizationCallback(const PoseWithCovarianceStamped::SharedPtr msg);

        virtual void goalDoneCallback(const rclcpp_action::ClientGoalHandle<NavAssistant>::WrappedResult& result);

        bool checkGoal(const NavAssistant::Goal& goal);
        void sendGoal(const NavAssistant::Goal& goal);
        float get_average_vector(std::vector<float> const &v);
        virtual void save_results_to_file(int result);
        virtual void declareParameters();
};