#pragma once
#include <vector>
#include <list>
#include <fstream>      // std::ofstream
#include <iostream>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <gsl_algorithm.h>

typedef actionlib::SimpleActionClient<navigation_assistant::nav_assistantAction> MoveBaseClient;
enum class SPIRAL_state {WAITING_FOR_MAP, STOP_AND_MEASURE, SPIRAL};

class SpiralSearcher:public GSLAlgorithm
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
    navigation_assistant::nav_assistantGoal nextGoalSpiral(geometry_msgs::Pose initial);
    bool doSpiral();

    SPIRAL_state getPreviousState();
    SPIRAL_state getCurrentState();
    double getPI();

    void save_results_to_file(int result);
    
private:
    SPIRAL_state current_state;
    SPIRAL_state previous_state;

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

    //-------------------
    // CallBack functions
    //-------------------
    void gasCallback(const olfaction_msgs::gas_sensorPtr& msg) override;
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) override;
    void windCallback(const olfaction_msgs::anemometerPtr& msg) override;
};