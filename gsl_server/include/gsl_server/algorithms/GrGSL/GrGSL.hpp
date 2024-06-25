#pragma once
#include <gsl_server/algorithms/Algorithm.hpp>
#include <gsl_server/algorithms/Common/Grid.hpp>
#include <gmrf_wind_mapping/srv/wind_estimation.hpp>
#include <gsl_server/core/FunctionQueue.hpp>

namespace GSL
{

    class GrGSL : public Algorithm
    {
        friend class MovingStateGrGSL;
        using WindEstimation = gmrf_wind_mapping::srv::WindEstimation;
        using HashSet = std::unordered_set<Vector2Int>;

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
        GridMetadata gridMetadata;

        struct Settings
        {
            float stdevHit;
            float stdevMiss;
            float convergence_thr;

            bool infoTaxis;
            bool allowMovementRepetition;
			bool useDiffusionTerm;
        } settings;

        struct Markers
        {
            rclcpp::Publisher<Marker>::SharedPtr probabilityMarkers;
            rclcpp::Publisher<Marker>::SharedPtr estimationMarkers;
            float markersHeight;
        } markers;

        int exploredCells = 0;
        Vector2 positionOfLastHit;
        FunctionQueue functionQueue;

        void Initialize() override;
        void declareParameters() override;
        void onGetMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) override;
        void initializeMap();
        void processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection) override;
        virtual double probability(const Vector2Int& indices) const;

        GSLResult checkSourceFound() override;
        void saveResultsToFile(GSLResult result) override;

        // core
        //-------------
        void estimateProbabilitiesfromGasAndWind(std::vector<std::vector<Cell>>& map, bool hit, bool advection, double windDirection,
                                                 Vector2Int robotPosition);
        void propagateProbabilities(std::vector<std::vector<Cell>>& map, HashSet& openSet, HashSet& closedSet, HashSet& activeSet);

        void calculateWeight(std::vector<std::vector<Cell>>& map, int i, int j, Vector2Int p, HashSet& openPropagationSet,
                             HashSet& closedPropagationSet, HashSet& activePropagationSet);
        void normalizeWeights(std::vector<std::vector<Cell>>& map);
        double informationGain(const WindVector& windVec);

        Vector2 expectedValueSource(double proportionBest);
        double varianceSourcePosition();
        void showWeights();

		enum MapFunctionMode {Sequential, Parallel};
		void mapFunctionToCells(std::vector<std::vector<Cell>>& cells, std::function<void(Cell&, size_t, size_t)> function, MapFunctionMode mode = MapFunctionMode::Sequential);
    };
} // namespace GSL