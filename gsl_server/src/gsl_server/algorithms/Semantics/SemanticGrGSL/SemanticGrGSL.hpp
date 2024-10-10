#pragma once

#include "gsl_server/algorithms/Common/Algorithm.hpp"
#include "gsl_server/algorithms/Common/Grid2D.hpp"
#include "gsl_server/algorithms/GrGSL/GrGSL_internal.hpp"
#include "gsl_server/core/FunctionQueue.hpp"
#include "gsl_server/algorithms/Semantics/Semantics/Common/ISemantics.hpp"

namespace GSL
{
    class SemanticGrGSL : public Algorithm
    {
    public:
        SemanticGrGSL(std::shared_ptr<rclcpp::Node> _node)
            : Algorithm(_node)
        {}

        void OnUpdate() override;

    protected:
        std::vector<GrGSL_internal::Cell> cells;       // only considers olfaction data
        std::vector<double> combinedSourceProbability; // olfaction + semantics
        std::vector<Occupancy> navigationOccupancy;
        std::vector<Occupancy> simulationOccupancy;
        Grid2DMetadata gridMetadata;
        std::unique_ptr<ISemantics> semantics;

        GrGSL_internal::Settings settings;
        GrGSL_internal::Markers markers;

        int exploredCells = 0;
        Vector2 positionOfLastHit;
        FunctionQueue functionQueue;

        void Initialize() override;
        void declareParameters() override;
        void onGetMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) override;
        void processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection) override;
        void updateSourceFromSemantics();

        void createClassMap2D();
        void createClassMapVoxeland();
    };
} // namespace GSL