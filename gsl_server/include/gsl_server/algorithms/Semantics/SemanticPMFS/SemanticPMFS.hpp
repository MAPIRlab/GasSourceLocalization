#pragma once

#include <gsl_server/algorithms/Algorithm.hpp>
#include <gsl_server/algorithms/PMFS/internal/HitProbKernel.hpp>
#include <gsl_server/algorithms/PMFS/internal/HitProbability.hpp>
#include <gsl_server/algorithms/PMFS/internal/Simulations.hpp>
#include <gsl_server/algorithms/Semantics/SemanticPMFS/SemanticPMFSSettings.hpp>
#include <gsl_server/algorithms/Semantics/SemanticPMFS/SemanticPMFSPubs.hpp>
#include <gsl_server/algorithms/Semantics/ISemantics.hpp>
#include <gsl_server/core/FunctionQueue.hpp>

namespace GSL
{
    class SemanticPMFS : public Algorithm
    {
        template <typename T>
        using Grid = Grid<T>;
        using HitProbability = PMFS_internal::HitProbability;
        using HitProbKernel = PMFS_internal::HitProbKernel;
        using HashSet = std::unordered_set<Vector2Int>;

    public:
        SemanticPMFS(std::shared_ptr<rclcpp::Node> _node);
        void Initialize() override;
        void OnUpdate() override;

    protected:
        void declareParameters() override;
        void onGetMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) override;
        void processGasAndWindMeasurements(double concentration, double wind_speed, double wind_direction) override;
    
    private:
        std::unique_ptr<ISemantics> semantics;

        //-------------PMFS-------------
        GridMetadata gridMetadata;
        std::vector<double> sourceProbability;
        std::vector<HitProbability> hitProbability;
        std::vector<Occupancy> navigationOccupancy;
        std::vector<Occupancy> simulationOccupancy;
        std::vector<Vector2> estimatedWindVectors;

        PMFS_internal::Simulations simulations;

        //-------------Data-------------
        SemanticPMFS_internal::Settings settings;
        SemanticPMFS_internal::PublishersAndSubscribers pubs;
    

        //-------------Utils-------------
        VisibilityMap visibilityMap;
        FunctionQueue functionQueue;
    };
}