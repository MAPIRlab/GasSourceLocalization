#ifndef PLUME_TRACKING
#define PLUME_TRACKING
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetPlan.h>
#include <boost/format.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <olfaction_msgs/gas_sensor_array.h>
#include <olfaction_msgs/gas_sensor.h>
#include <olfaction_msgs/anemometer.h>

#include <vector>
#include <list>
#include <angles/angles.h>
#include <fstream>      // std::ofstream
#include <iostream>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
enum class PT_state {WAITING_FOR_MAP, EXPLORATION, STOP_AND_MEASURE, INSPECTION, UPWIND_SURGE, CROSSWIND_CAST};

class PlumeTracking
{

public:
    PlumeTracking(ros::NodeHandle *nh);
    ~PlumeTracking();

    PT_state get_state();              //Returns current Search-State
    bool get_inMotion();                //Ture if we have a movement target
    void getGasWindObservations();      //Get Wind and Gas observations while standing
    float get_gas_concentration();      //Returns the averaged gas concentration
    virtual void checkState() = 0;                  //Check gas/wind to see if a state transition is necessary

    void setExplorationGoal();          //Set target according to GDM
    void setRandomGoal();               //Set target random (within the map)
    virtual void setSurgeGoal() = 0;                //Set target upwind
    virtual void setCastGoal() = 0;                 //Set target crosswind
    void setInspectionGoal();           // Set target arround current location

    void cancel_navigation();
    virtual int checkSourceFound();
    void restart_search();
protected:
    ros::NodeHandle *nh_;                                                 //! Node handler.
    ros::Time movingTimestamp;
    ros::Time recoveryTimestamp;

    //GSL variables
    PT_state current_state, previous_state;
    bool inMotion;                                                      //! Determines if a goal has been set and we are moving towards it
    int moving_average_size;
    double th_gas_present;
    double th_wind_present;
    double step, current_step;                                          //!distance (m) to move upwind during Surge search
    unsigned int cast_movement;                                          //! initial movement direction for crossWind (cast phase)    
    int inspection_iter;
    double inspection_radius;
    std::string enose_topic, anemometer_topic, robot_location_topic, map_topic;
    //For gas hit detection
    std::vector<float> gasConcentration_v;
    std::vector<float> windSpeed_v;
    std::vector<float> windDirection_v;
    std::vector<float>::iterator gas_it;
    std::vector<float>::iterator windS_it;
    std::vector<float>::iterator windD_it;
    //Stop_and_measure
    ros::Time time_stopped;                                             //! Used to measure wind while robot is stopped
    float average_concentration;
    float average_wind_direction;
    float average_wind_spped;
    std::vector<float> stop_and_measure_gas_v;
    std::vector<float> stop_and_measure_windS_v;
    std::vector<float> stop_and_measure_windD_v;

    double timeout_cast;                                                //! (seconds) Max time to do Cast (crosswind search) before returning to the EXPLORATION state.
    double stop_and_measure_time;                                       //! (seconds) time the robot is stopped while measuring the wind direction
    bool gasHit;
    //metrics
    bool verbose;
    bool inExecution;
    ros::Time start_time;
    double max_search_time;
    double distance_found;
    std::vector<geometry_msgs::PoseWithCovarianceStamped> robot_poses_vector;
    double source_pose_x, source_pose_y;
    double robot_pose_x, robot_pose_y;
    std::string results_file;

    visualization_msgs::Marker wind_point;
    visualization_msgs::MarkerArray wind_pointcloud;
    int arrow_count;

    //Variables for navigation
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
    void gasCallback(const olfaction_msgs::gas_sensorPtr& msg);
    virtual void windCallback(const olfaction_msgs::anemometerPtr& msg);
    void localizationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    //Topics to publish
    ros::Publisher wind_measurements_pub;              // Visualization of the wind vectors measured along the Search

    //Methods
    bool checkGoal(move_base_msgs::MoveBaseGoal * goal);
    bool checkCell(int goal_cell_x, int goal_cell_y);
    float get_average_vector(std::vector<float> const &v);
    float get_average_wind_direction(std::vector<float> const &v);
    geometry_msgs::PoseStamped get_random_pose_environment();
    double fRand(double fMin, double fMax);
    virtual void save_results_to_file(int result);

    //Actionlib callbacks (move_base)
    void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result);
    void goalActiveCallback();
    void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);
};

#endif