#include <gsl_algorithm.h>
#include <fstream>      // std::ofstream
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <bits/stdc++.h>
#include <unordered_set>

#include <gmrf_wind_mapping/WindEstimation.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
enum class Grid_state {WAITING_FOR_MAP, EXPLORATION, STOP_AND_MEASURE, MOVING};

class Cell{
    public:
        Cell(bool free, double x, double y, double weight);
        ~Cell();
        bool free;
        double x, y, weight, auxWeight;
        double distance;        
};

struct WindVector{
    int i;
    int j;
    double speed;
    double angle;
};

class GridGSL:public GSLAlgorithm
{
    public:
        GridGSL(ros::NodeHandle *nh);
        ~GridGSL();
        void getGasWindObservations();
        Grid_state getState();
        void setGoal();
        int checkSourceFound() override;

    protected:

        //CallBacks
        void gasCallback(const olfaction_msgs::gas_sensorPtr& msg) override;
        void windCallback(const olfaction_msgs::anemometerPtr& msg) override;
        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) override;
        void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResultConstPtr &result) override;
            
        Grid_state previous_state, current_state;

        //Measurements
        ros::Time time_stopped; 
        double stop_and_measure_time;                                       //! (seconds) time the robot is stopped while measuring the wind direction
        bool gasHit;
        double th_gas_present;
        double th_wind_present;

        std::vector<float> stop_and_measure_gas_v;
        std::vector<float> stop_and_measure_windS_v;
        std::vector<float> stop_and_measure_windD_v;
        double average_concentration, average_wind_direction, average_wind_speed;
        float get_average_wind_direction(std::vector<float> const &v);

        //Estimations
        double stdev_hit;
        double stdev_miss;
        void estimateProbabilities(std::vector<std::vector<Cell> >& map,
                                    bool hit,
                                    double wind_direction,
                                    Eigen::Vector2i robot_pos);
        void propagateProbabilities(std::vector<std::vector<Cell> >& map,
                                    std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& openSet,
                                    std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& closedSet,
                                    std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& activeSet);

        void calculateWeight(std::vector<std::vector<Cell> >& map,
                                int i, int j, std::pair<int,int> p,
                                std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& openSet,
                                std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& closedSet,
                                std::unordered_set<std::pair<int, int>, boost::hash< std::pair<int, int> > >& activeSet);
        void normalizeWeights();

        
        //Movement
        bool infoTaxis;
        void moveTo(int i, int j);
        void cancel_navigation(); 
        
        Eigen::Vector2d previous_robot_pose;

        std::unordered_set< 
            std::pair<int, int>, 
            boost::hash< std::pair<int, int> > 
        > openMoveSet;
        std::unordered_set< 
            std::pair<int, int>, 
            boost::hash< std::pair<int, int> > 
        > closedMoveSet;
        
        void updateSets();

        //Infotaxis
        double entropy(int i, int j, Eigen::Vector2d wind);
        ros::ServiceClient clientW;
        std::vector<WindVector> estimateWind();

        //Cells
        double scale;
        int numCells;
        std::vector<std::vector<Cell> > cells;
        ros::Publisher probability_markers;
        Eigen::Vector2i currentPosIndex;
        
        //Auxiliary functions
        visualization_msgs::Marker emptyMarker();
        void showWeights();
        Eigen::Vector3d valueToColor(double val, double low, double high);
        Eigen::Vector2i coordinatesToIndex(double x, double y);
        Eigen::Vector2d indexToCoordinates(double i, double j);
        double gaussian(double distance, double sigma);
        
        //Termination condition
        bool reached;
        double t1;
        double convergence_thr;
        double ground_truth_x, ground_truth_y;
        void save_results_to_file(int result, int i, int j);
};