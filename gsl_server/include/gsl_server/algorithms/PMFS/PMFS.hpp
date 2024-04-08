#pragma once
#include <gsl_server/algorithms/Algorithm.hpp>

#include <gsl_server/algorithms/PMFS/internal/Cell.hpp>
#include <gsl_server/algorithms/PMFS/internal/Settings.hpp>
#include <gsl_server/algorithms/PMFS/internal/PublishersAndSubscribers.hpp>
#include <gsl_server/algorithms/PMFS/internal/HitProbKernel.hpp>
#include <gsl_server/algorithms/PMFS/internal/Simulations.hpp>
#include <gsl_server/algorithms/PMFS/internal/UI.hpp>
#include <gsl_server/algorithms/PMFS/MovingStatePMFS.hpp>

#include <gsl_server/core/FunctionQueue.hpp>

namespace GSL
{
    class PMFS : public Algorithm
    {
        friend class MovingStatePMFS;
        friend class PMFS_internal::Simulations;
        friend class PMFS_internal::SimulationSource;
#ifdef USE_GUI
        friend class PMFS_internal::UI;
#endif
        using HashSet = std::unordered_set<Vector2Int>;

        template <typename T>
        using Grid = Grid<T>;
        using HitProbability = PMFS_internal::HitProbability;
        using HitProbKernel = PMFS_internal::HitProbKernel;

    public:
        PMFS(std::shared_ptr<rclcpp::Node> _node);
        void OnUpdate() override;

    protected:
        void initialize() override;
        void declareParameters() override;
        void onGetMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) override;
        void processGasAndWindMeasurements(double concentration, double wind_speed, double wind_direction) override;
        GSLResult checkSourceFound() override;
        void saveResultsToFile(GSLResult result) override;
        void OnCompleteNavigation(GSLResult result, State* previousState) override;
        float gasCallback(olfaction_msgs::msg::GasSensor::SharedPtr msg) override;


        //-------------Core-------------
        std::vector<double> sourceProbability;
        std::vector<HitProbability> hitProbability;
        std::vector<Occupancy> navigationOccupancy;
        std::vector<Occupancy> simulationOccupancy;

        GridMetadata gridMetadata;
        

        std::vector<Vector2> estimatedWindVectors;

        //-------------Data-------------
        PMFS_internal::Simulations simulations;
        PMFS_internal::Settings settings;
        PMFS_internal::PublishersAndSubscribers pubs;

        //-------------Utils-------------
        bool paused = false;
        std::unordered_map<Vector2Int, HashSet> visibilityMap;
        FunctionQueue functionQueue;
        uint iterationsCounter;

        Vector2Int currentPosIndex()
        {
            return gridMetadata.coordinatesToIndex(currentRobotPose.pose.pose.position.x, currentRobotPose.pose.pose.position.y);
        }
        bool pathFree(const Vector2Int& origin, const Vector2Int& end);
        bool indicesInBounds(const Vector2Int indices) const;
        void normalizeSourceProb(Grid<double>& variable);
        Vector2 expectedValueSource(double proportionBest);
        double varianceSourcePosition();

        // Visualization
        void showWeights();
        void debugMapSegmentation();
        void plotWindVectors();
#ifdef USE_GUI
        PMFS_internal::UI ui;
#endif
    };
} // namespace GSL