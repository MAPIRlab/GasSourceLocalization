#pragma once
#include "GrGSL_internal.hpp"
#include "gsl_server/core/ConditionalMacros.hpp"
#include <gmrf_msgs/srv/wind_estimation.hpp>
#include <gsl_server/algorithms/Common/Algorithm.hpp>
#include <gsl_server/algorithms/Common/Grid2D.hpp>
#include <gsl_server/core/FunctionQueue.hpp>

#if USE_GUI
#include "internal/UI.hpp"
#endif

namespace GSL
{

    class GrGSL : public Algorithm
    {
        friend class MovingStateGrGSL;
        IF_GUI(friend class GrGSL_internal::UI);
        using WindEstimation = gmrf_msgs::srv::WindEstimation;
        using HashSet = std::unordered_set<Vector2Int>;

    public:
        GrGSL(std::shared_ptr<rclcpp::Node> _node);

        void OnUpdate() override;

    private:
        void Initialize() override;
        void declareParameters() override;
        void onGetMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) override;
        void processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection) override;
        double probability(const Vector2Int& indices) const;
        GSLResult checkSourceFound() override;
        void saveResultsToFile(GSLResult result) override;

    private:
        std::vector<GrGSL_internal::Cell> cells;
        std::vector<Occupancy> occupancy;
        Grid2DMetadata gridMetadata;

        GrGSL_internal::Settings settings;
        GrGSL_internal::Markers markers;

        int exploredCells = 0;
        Vector2 positionOfLastHit;
        FunctionQueue functionQueue;
        
        bool paused = false;
        IF_GUI(GrGSL_internal::UI ui;)
    };
} // namespace GSL