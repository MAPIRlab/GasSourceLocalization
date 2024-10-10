#pragma once

#include <gsl_server/algorithms/Common/Algorithm.hpp>
#include <gsl_server/algorithms/PMFS/internal/HitProbKernel.hpp>
#include <gsl_server/algorithms/PMFS/internal/HitProbability.hpp>
#include <gsl_server/algorithms/PMFS/internal/Simulations.hpp>
#include <gsl_server/algorithms/Semantics/SemanticPMFS/MovingStateSemanticPMFS.hpp>
#include <gsl_server/algorithms/Semantics/SemanticPMFS/SemanticPMFSPubs.hpp>
#include <gsl_server/algorithms/Semantics/SemanticPMFS/SemanticPMFSSettings.hpp>
#include <gsl_server/algorithms/Semantics/Semantics/Common/ISemantics.hpp>
#include <gsl_server/core/FunctionQueue.hpp>

namespace GSL
{
    class SemanticPMFS : public Algorithm
    {

        template <typename T>
        using Grid = Grid2D<T>;
        using HitProbability = PMFS_internal::HitProbability;
        using HitProbKernel = PMFS_internal::HitProbKernel;
        using HashSet = std::unordered_set<Vector2Int>;

        friend class MovingStateSemanticPMFS;

    public:
        SemanticPMFS(std::shared_ptr<rclcpp::Node> _node);
        void Initialize() override;
        void OnUpdate() override;

    private:
        void declareParameters() override;
        void onGetMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) override;
        void processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection) override;
        void updateSourceFromSemantics();

    private:
        std::unique_ptr<ISemantics> semantics;
        std::vector<double> combinedSourceProbability;

        //-------------PMFS-------------
        Grid2DMetadata gridMetadata;
        std::vector<double> sourceProbabilityPMFS;
        std::vector<HitProbability> hitProbability;
        std::vector<Occupancy> navigationOccupancy;
        std::vector<Occupancy> simulationOccupancy;
        std::vector<Vector2> estimatedWindVectors;

        PMFS_internal::Simulations simulations;

        //-------------Data-------------
        SemanticPMFS_internal::Settings settings;
        SemanticPMFS_internal::PublishersAndSubscribers pubs;

        //-------------Utils-------------
        std::optional<VisibilityMap> visibilityMap;
        FunctionQueue functionQueue;
        uint iterationsCounter = 0;
        bool paused = false;

        void createClassMap2D();
        void createClassMapVoxeland();
    };
} // namespace GSL