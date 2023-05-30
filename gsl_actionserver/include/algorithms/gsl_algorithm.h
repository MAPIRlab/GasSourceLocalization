#pragma once
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>
#include <navigation_assistant/nav_assistantAction.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>

#include <olfaction_msgs/gas_sensor_array.h>
#include <olfaction_msgs/gas_sensor.h>
#include <olfaction_msgs/anemometer.h>
#include "spdlog/spdlog.h"
#include "Utils/Utils.h"
#include "gsl_macros.h"

typedef geometry_msgs::Pose Pose;
typedef geometry_msgs::Point Point;
typedef actionlib::SimpleActionClient<navigation_assistant::nav_assistantAction> NavAssistClient;

class GSLAlgorithm
{
    public:
        GSLAlgorithm(ros::NodeHandle *nh);
        ~GSLAlgorithm();
        bool get_inMotion();                //True if we have a movement target
        virtual int checkSourceFound();
        void restart_search();
        double max_search_time;

        bool isPointInsideMapBounds(const Utils::Vector2& point) const;
        
    protected:
        ros::NodeHandle *nh_;                                                 //! Node handler.
        bool inMotion;                                                      //! Determines if a goal has been set and we are moving towards it
        std::string enose_topic, anemometer_topic, robot_location_topic, map_topic, costmap_topic;

        int moving_average_size;

        bool verbose;
        bool inExecution;
        ros::Time start_time;
        double distance_found;
        std::vector<geometry_msgs::PoseWithCovarianceStamped> robot_poses_vector;
        double source_pose_x, source_pose_y;
        double robot_pose_x, robot_pose_y;
        std::string results_file;
        std::string errors_file;
        std::string path_file;
    
        nav_msgs::OccupancyGrid map_;                                       //! Map
        nav_msgs::OccupancyGrid costmap_;                                       //! Map
        geometry_msgs::PoseWithCovarianceStamped movingPose;        //! Robot pose on the global frame referential
        geometry_msgs::PoseWithCovarianceStamped current_robot_pose;        //! Robot pose on the global frame referential
        NavAssistClient mb_ac;                                               //! Move Base Action Server.


        //Subscriptions
        ros::Publisher localizationOffset;
        ros::Subscriber gas_sub_;                                           //! Gas readings subscriber
        ros::Subscriber wind_sub_;                                          //! Wind readings subscriber
        ros::Subscriber map_sub_;                                           //! Map subscriber.
        ros::Subscriber costmap_sub_;                                       //! CostMap subscriber.
        ros::Subscriber localization_sub_;
        ros::ServiceClient make_plan_client;
        tf::TransformListener tf_listener;
        //tf::MessageFilter<nav_msgs::Odometry> * tf_filter_;

        //CallBacks
        virtual void gasCallback(const olfaction_msgs::gas_sensorPtr& msg)=0;
        virtual void windCallback(const olfaction_msgs::anemometerPtr& msg)=0;
        void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
        virtual void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)=0;
        void localizationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
        virtual void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const navigation_assistant::nav_assistantResultConstPtr &result);
        virtual void goalActiveCallback();
        virtual void goalFeedbackCallback(const navigation_assistant::nav_assistantFeedbackConstPtr &feedback);

        bool checkGoal(navigation_assistant::nav_assistantGoal * goal);
        float get_average_vector(std::vector<float> const &v);
        virtual void save_results_to_file(int result);
};