#pragma once
#include <gsl_server/algorithms/Algorithm.hpp>
#include <gsl_server/algorithms/Common/GridData.hpp>
#include <gmrf_wind_mapping/srv/wind_estimation.hpp>
#include <gsl_server/core/FunctionQueue.hpp>

namespace GSL
{

    class GrGSL : public Algorithm
    {
        friend class MovingStateGrGSL;
        using WindEstimation = gmrf_wind_mapping::srv::WindEstimation;
        using hashSet = std::unordered_set<Vector2Int>;

    public:
        GrGSL(std::shared_ptr<rclcpp::Node> _node) : Algorithm(_node)
        {}

        void OnUpdate() override;

    protected:
        struct WindVector
        {
            int i;
            int j;
            double speed;
            double angle;
        };
        struct Cell
        {
            Cell(bool free, double weight);
            bool free;
            double weight;
            double auxWeight, originalPropagatedWeight;
            double distance;
        };
        std::vector<std::vector<Cell>> grid;
        GridData gridData;

        struct Settings
        {
            float stdev_hit;
            float stdev_miss;
            float convergence_thr;

            bool infoTaxis;
            bool allowMovementRepetition;
			bool useDiffusionTerm;
        } settings;

        struct Markers
        {
            rclcpp::Publisher<Marker>::SharedPtr probability_markers;
            rclcpp::Publisher<Marker>::SharedPtr estimation_markers;
            float markersHeight;
        } markers;

        int exploredCells = 0;
        Vector2 positionOfLastHit;
        FunctionQueue functionQueue;

        void initialize() override;
        void declareParameters() override;
        void onGetMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) override;
        void initializeMap();
        void processGasAndWindMeasurements(double concentration, double wind_speed, double wind_direction) override;
        virtual double probability(const Vector2Int& indices) const;

        GSLResult checkSourceFound() override;
        void saveResultsToFile(GSLResult result) override;

        // core
        //-------------
        void estimateProbabilitiesfromGasAndWind(std::vector<std::vector<Cell>>& map, bool hit, bool advection, double wind_direction,
                                                 Vector2Int robot_pos);
        void propagateProbabilities(std::vector<std::vector<Cell>>& map, hashSet& openSet, hashSet& closedSet, hashSet& activeSet);

        void calculateWeight(std::vector<std::vector<Cell>>& map, int i, int j, Vector2Int p, hashSet& openPropagationSet,
                             hashSet& closedPropagationSet, hashSet& activePropagationSet);
        void normalizeWeights(std::vector<std::vector<Cell>>& map);
        double informationGain(const WindVector& windVec);

        Vector2Int currentPosIndex()
        {
            return gridData.coordinatesToIndex(currentRobotPose.pose.pose.position.x, currentRobotPose.pose.pose.position.y);
        }
        Vector2 expectedValueSource(double proportionBest);
        double varianceSourcePosition();
        void showWeights();

		enum MapFunctionMode {Sequential, Parallel};
		void mapFunctionToCells(std::vector<std::vector<Cell>>& cells, std::function<void(Cell&, size_t, size_t)> function, MapFunctionMode mode = MapFunctionMode::Sequential);
    };
} // namespace GSL