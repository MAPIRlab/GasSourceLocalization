#pragma once
#include <gsl_server/algorithms/Common/Algorithm.hpp>
#include <gsl_server/algorithms/Common/Grid2D.hpp>
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

        std::vector<GrGSL_internal::Cell> cells;
        std::vector<Occupancy> occupancy;
        Grid2DMetadata gridMetadata;

        GrGSL_internal::Settings settings;
        GrGSL_internal::Markers markers;

        int exploredCells = 0;
        Vector2 positionOfLastHit;
        FunctionQueue functionQueue;

        void Initialize() override;
        void declareParameters() override;
        void onGetMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) override;
        void processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection) override;
        double probability(const Vector2Int& indices) const;

        GSLResult checkSourceFound() override;
        void saveResultsToFile(GSLResult result) override;
    };
} // namespace GSL