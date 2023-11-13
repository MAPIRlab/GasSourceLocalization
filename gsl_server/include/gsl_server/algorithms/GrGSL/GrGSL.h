#pragma once
#include <gsl_server/algorithms/Algorithm.h>
#include <gmrf_wind_mapping/srv/wind_estimation.hpp>
#include <gsl_server/algorithms/Common/Grid.h>
#include <gsl_server/core/FunctionQueue.h>

namespace GSL
{
    using WindEstimation = gmrf_wind_mapping::srv::WindEstimation;

    class GrGSL : public Algorithm
    {
        GrGSL(std::shared_ptr<rclcpp::Node> _node) : Algorithm(_node)
        {}

        void OnUpdate() override;

    protected:
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

        struct WindVector
        {
            int i;
            int j;
            double speed;
            double angle;
        };

        void initialize() override;
        void declareParameters() override;

        void processGasAndWindMeasurements(double concentration, double wind_speed, double wind_direction) override;
        void onGetMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) override;
        void OnCompleteNavigation(GSLResult result) override;

        virtual GSLResult checkSourceFound();

        GridData grid;
        rclcpp::Client<WindEstimation>::SharedPtr clientWind;

        FunctionQueue functionQueue;
    };
} // namespace GSL