#pragma once
#include <gsl_server/algorithms/Common/Algorithm.hpp>
#include <gsl_server/algorithms/Common/Grid.hpp>
#include <gmrf_wind_mapping/srv/wind_estimation.hpp>
#include <gsl_server/core/FunctionQueue.hpp>
#include "GrGSL_internal.hpp"

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
        
        std::vector<std::vector<GrGSL_internal::Cell>> grid;
        GridMetadata gridMetadata;

        GrGSL_internal::Settings settings;

        GrGSL_internal::Markers markers;

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
        void estimateProbabilitiesfromGasAndWind(std::vector<std::vector<GrGSL_internal::Cell>>& map, bool hit, bool advection, double windDirection,
                Vector2Int robotPosition);
        void propagateProbabilities(std::vector<std::vector<GrGSL_internal::Cell>>& map, HashSet& openSet, HashSet& closedSet, HashSet& activeSet);

        void calculateWeight(std::vector<std::vector<GrGSL_internal::Cell>>& map, int i, int j, Vector2Int p, HashSet& openPropagationSet,
                             HashSet& closedPropagationSet, HashSet& activePropagationSet);
        void normalizeWeights(std::vector<std::vector<GrGSL_internal::Cell>>& map);
        double informationGain(const GrGSL_internal::WindVector& windVec);

        Vector2 expectedValueSource(double proportionBest);
        double varianceSourcePosition();
        void showWeights();

        enum MapFunctionMode {Sequential, Parallel};
        void mapFunctionToCells(std::vector<std::vector<GrGSL_internal::Cell>>& cells, std::function<void(GrGSL_internal::Cell&, size_t, size_t)> function, MapFunctionMode mode = MapFunctionMode::Sequential);
    };
} // namespace GSL