#pragma once

#include "gsl_server/algorithms/Common/Algorithm.hpp"
#include "gsl_server/algorithms/Common/Grid2D.hpp"
#include "gsl_server/algorithms/GrGSL/GrGSL_internal.hpp"
#include "gsl_server/algorithms/Semantics/Semantics/Common/ISemantics.hpp"
#include "internal/UI.hpp"

namespace GSL
{
    class SemanticGrGSL : public Algorithm
    {
        friend class MovingStateSemanticGrGSL;
        IF_GUI(friend class SemanticGrGSL_internal::UI;)
    public:
        SemanticGrGSL(std::shared_ptr<rclcpp::Node> _node)
            : Algorithm(_node)
            IF_GUI(,ui(this))
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

        IF_GUI(SemanticGrGSL_internal::UI ui;)

        void Initialize() override;
        void declareParameters() override;
        void onGetMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) override;
        void processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection) override;
        void updateSourceFromSemantics();
        void logProgressionAndVisualize();

        void createClassMap2D();
        void createClassMapVoxeland();
        GSLResult checkSourceFound() override;
        void saveResultsToFile(GSLResult result) override;
    
        template <typename T>
        Grid2D<T> AsGrid(std::vector<T>& vec, std::vector<Occupancy>& occupancy)
        {
            return Grid2D<T>(vec, occupancy, gridMetadata);
        }
    };
} // namespace GSL