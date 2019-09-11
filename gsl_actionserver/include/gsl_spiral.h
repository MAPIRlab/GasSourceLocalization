#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
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

#include <olfaction_msgs/gas_sensor_array.h>
#include <olfaction_msgs/gas_sensor.h>
#include <olfaction_msgs/anemometer.h>

#include <vector>
#include <list>
#include <angles/angles.h>
#include <fstream>      // std::ofstream
#include <iostream>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
enum class SPIRAL_state {WAITING_FOR_MAP, STOP_AND_MEASURE, SPIRAL};

class SpiralSearcher
{   
public:

    SpiralSearcher(ros::NodeHandle *nh);
    ~SpiralSearcher();

    //void checkState();
    void cancelNavigation();
    void getGasObservations();
    //useful function
    double get_average_vector(std::vector<std::vector<double> > const &v);
    double get_average_vector(std::vector<double> const &v);
    double fRand(double fMin, double fMax);
    double getSumOfLocalMaxima(std::vector<std::vector<double> > const &v);
    //movement functions
    geometry_msgs::PoseStamped get_random_pose_environment();
    void setRandomGoal();
    void resetSpiral();
    move_base_msgs::MoveBaseGoal nextGoalSpiral(geometry_msgs::Pose initial);
    bool checkGoal(move_base_msgs::MoveBaseGoal * goal);
    bool doSpiral();
    int checkSourceFound();
    bool isInMotion();
    SPIRAL_state getPreviousState();
    SPIRAL_state getCurrentState();
    double getPI();

    void save_results_to_file(int result);
    
private:
    ros::NodeHandle *nh_;
    MoveBaseClient mb_ac;
    SPIRAL_state current_state;
    SPIRAL_state previous_state;
    bool inMotion;    //! Determines if a goal has been set and we are moving towards it

    //Variables for controlling when to reset spiral
    double Kmu; //constants
    double Kp;

    double gas_present_thr;

    double intervalLength; //divides the stop-and-measure into smaller intervals (used to calculate the PI)
    int currentInterval; //index 
    ros::Time lastInterval; //when did the last interval end
    
    double previousPI;  //if new PI is better than this, we consider that a hit
    double minPI;   //used to reset the PI after a large number of misses
    int consecutive_misses; 
    int spiral_iter;
    
    //Stop_and_measure
    ros::Time time_stopped;                                             //! Used to measure while robot is stopped
    double average_concentration;
    std::vector<std::vector<double> > stop_and_measure_gas_v;

    double stop_and_measure_time;                                       //! (seconds) time the robot is stopped while measuring the gas
    double step;
    double step_increment;
    double initStep;
    bool verbose;
    bool inExecution;
    ros::Time start_time;
    double max_search_time;
    double distance_found;
    std::vector<geometry_msgs::PoseWithCovarianceStamped> robot_poses_vector;
    double source_pose_x, source_pose_y;
    double robot_pose_x, robot_pose_y;
    std::string results_file;
    int arrow_count;

    //Variables for navigation
    nav_msgs::OccupancyGrid map_;                                       //! Map
    geometry_msgs::PoseWithCovarianceStamped current_robot_pose;        //! Robot pose on the global frame referential


    //Subscriptions
    ros::Subscriber gas_sub_;                                           //! Gas readings subscriber
    ros::Subscriber map_sub_;                                           //! Map subscriber.
    ros::Subscriber localization_sub_;
    ros::ServiceClient mb_client;
    tf::TransformListener tf_;
    std::string enose_topic, robot_location_topic, map_topic;
    
    //Called Services
    ros::ServiceClient srv_GDM_client;
    
    //-------------------
    // CallBack functions
    //-------------------
    void gasCallback(const olfaction_msgs::gas_sensorPtr& msg);
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void localizationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result);
    void goalActiveCallback();
    void goalFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);
};