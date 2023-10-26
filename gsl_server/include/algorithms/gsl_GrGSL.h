#include <algorithms/gsl_algorithm.h>
#include <Utils/Utils.h>
#include <fstream> // std::ofstream
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <bits/stdc++.h>
#include <unordered_set>
#include <std_msgs/msg/string.hpp>
#include <deque>

#include <gmrf_wind_mapping/srv/wind_estimation.hpp>

namespace GrGSL
{

    typedef Utils::Vector2Int Vector2Int;
    typedef std::unordered_set<Vector2Int, Vector2Int::Vec2IntHash, Vector2Int::Vec2IntCompare> hashSet;
    using WindEstimation = gmrf_wind_mapping::srv::WindEstimation;

    enum class State
    {
        WAITING_FOR_MAP,
        INITIALIZING,
        EXPLORATION,
        STOP_AND_MEASURE,
        MOVING
    };

    class Cell
    {
    public:
        Cell(bool free, double weight);
        ~Cell();
        bool free;
        double weight;
        double auxWeight, originalPropagatedWeight;
        double distance;
    };

    struct WindVector
    {
        int i;
        int j;
        double speed;
        double angle;
    };

    class GrGSL : public GSLAlgorithm
    {
    public:
        GrGSL(std::shared_ptr<rclcpp::Node> _node);
        ~GrGSL();
        void getGasWindObservations();
        State getState();
        int checkSourceFound() override;
        void showWeights();
        virtual void setGoal();
        virtual void initialize() override;
        void initializeMap();

    protected:
        void declareParameters() override;
        // CallBacks
        void gasCallback(const olfaction_msgs::msg::GasSensor::SharedPtr msg) override;
        void windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg) override;
        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) override;
        void goalDoneCallback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult&) override;

        State previous_state, current_state;

        // Measurements
        rclcpp::Time time_stopped;
        double stop_and_measure_time; //! (seconds) time the robot is stopped while measuring the wind direction
        bool gasHit;
        double th_gas_present;
        double th_wind_present;

        std::vector<float> stop_and_measure_gas_v;
        std::vector<float> stop_and_measure_windS_v;
        std::vector<float> stop_and_measure_windD_v;
        double average_concentration, average_wind_direction, average_wind_speed;
        float get_average_wind_direction(std::vector<float> const& v);

        // Estimations
        double stdev_hit;
        double stdev_miss;
        void estimateProbabilitiesfromGasAndWind(std::vector<std::vector<Cell>>& map, bool hit, bool advection, double wind_direction,
                                                 Vector2Int robot_pos);
        void propagateProbabilities(std::vector<std::vector<Cell>>& map, hashSet& openSet, hashSet& closedSet, hashSet& activeSet);

        void calculateWeight(std::vector<std::vector<Cell>>& map, int i, int j, Vector2Int p, hashSet& openPropagationSet,
                             hashSet& closedPropagationSet, hashSet& activePropagationSet);
        void normalizeWeights(std::vector<std::vector<Cell>>& map);

        // Movement
        bool infoTaxis;
        bool allowMovementRepetition;
        void moveTo(NavigateToPose::Goal goal);
        NavigateToPose::Goal indexToGoal(int i, int j);
        void cancel_navigation();

        Eigen::Vector2d previous_robot_pose;

        hashSet openMoveSet;
        hashSet closedMoveSet;

        void updateSets();

        // Infotaxis
        bool computingInfoGain = false;
        double informationGain(WindVector windVec);
        double KLD(std::vector<std::vector<Cell>>& a, std::vector<std::vector<Cell>>& b);
        rclcpp::Client<WindEstimation>::SharedPtr clientWind;
        std::vector<WindVector> estimateWind();

        // Cells
        int scale;
        int numCells;
        std::vector<std::vector<Cell>> cells;
        rclcpp::Publisher<Marker>::SharedPtr probability_markers;
        rclcpp::Publisher<Marker>::SharedPtr estimation_markers;
        Vector2Int currentPosIndex;
        virtual double probability(const Vector2Int& indices);

        // Auxiliary functions
        Marker emptyMarker();
        double markers_height;
        Eigen::Vector3d valueToColor(double val, double low, double high);
        Vector2Int coordinatesToIndex(double x, double y);
        Eigen::Vector2d indexToCoordinates(double i, double j);
        double gaussian(double distance, double sigma);

        // Termination condition
        int exploredCells;
        bool reached;
        double t1;
        int convergence_steps;
        double convergence_thr;
        double ground_truth_x, ground_truth_y;
        std::deque<Eigen::Vector2d> estimatedSourceLocations;
        std::vector<double> errorOverTimeAll;
        std::vector<double> errorOverTime;
        Eigen::Vector2d expectedValueSource(double proportionBest);
        double varianceSourcePosition();
        void save_results_to_file(int result, double i, double j, double allI, double allJ);
    };

    struct CellData
    {
        Vector2Int indices;
        double probability;
        CellData(Vector2Int ind, double prob)
        {
            indices = ind;
            probability = prob;
        }
        ~CellData()
        {
        }
    };
} // namespace GrGSL
