#ifndef GSL_ALGORITHM
#define GSL_ALGORITHM
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h>

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
#include <boost/format.hpp>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
class GSLAlgorithm
{
    public:
        GSLAlgorithm(ros::NodeHandle *nh);
        ~GSLAlgorithm();
        bool get_inMotion();                //True if we have a movement target
        virtual int checkSourceFound();
        void restart_search();

    protected:
        ros::NodeHandle *nh_;                                                 //! Node handler.
        bool inMotion;                                                      //! Determines if a goal has been set and we are moving towards it
        std::string enose_topic, anemometer_topic, robot_location_topic, map_topic;

        int moving_average_size;

        bool verbose;
        bool inExecution;
        ros::Time start_time;
        double max_search_time;
        double distance_found;
        std::vector<geometry_msgs::PoseWithCovarianceStamped> robot_poses_vector;
        double source_pose_x, source_pose_y;
        double robot_pose_x, robot_pose_y;
        std::string results_file;
    
        nav_msgs::OccupancyGrid map_;                                       //! Map
        geometry_msgs::PoseWithCovarianceStamped movingPose;        //! Robot pose on the global frame referential
        geometry_msgs::PoseWithCovarianceStamped current_robot_pose;        //! Robot pose on the global frame referential
        MoveBaseClient mb_ac;                                               //! Move Base Action Server.


        //Subscriptions
        ros::Subscriber gas_sub_;                                           //! Gas readings subscriber
        ros::Subscriber wind_sub_;                                          //! Wind readings subscriber
        ros::Subscriber map_sub_;                                           //! Map subscriber.
        ros::Subscriber localization_sub_;
        ros::ServiceClient mb_client;
        tf::TransformListener tf_;
        //tf::MessageFilter<nav_msgs::Odometry> * tf_filter_;

        //Called Services
        ros::ServiceClient srv_GDM_client;

        //CallBacks
        virtual void gasCallback(const olfaction_msgs::gas_sensorPtr& msg)=0;
        virtual void windCallback(const olfaction_msgs::anemometerPtr& msg)=0;
        virtual void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)=0;
        void localizationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
        virtual void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result);
        void goalActiveCallback();
        void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);

        bool checkGoal(move_base_msgs::MoveBaseGoal * goal);
        float get_average_vector(std::vector<float> const &v);
        virtual void save_results_to_file(int result);
};

#endif