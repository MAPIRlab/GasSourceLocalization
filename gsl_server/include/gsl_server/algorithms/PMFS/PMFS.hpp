#pragma once
#include <gsl_server/algorithms/Algorithm.hpp>

#include <gsl_server/algorithms/PMFS/internal/Cell.hpp>
#include <gsl_server/algorithms/PMFS/internal/Settings.hpp>
#include <gsl_server/algorithms/PMFS/internal/PublishersAndSubscribers.hpp>
#include <gsl_server/algorithms/PMFS/internal/HitProbKernel.hpp>
#include <gsl_server/algorithms/PMFS/internal/Simulations.hpp>
#include <gsl_server/algorithms/PMFS/internal/UI.hpp>
#include <gsl_server/algorithms/PMFS/MovingStatePMFS.hpp>

#include <gsl_server/algorithms/Common/GridData.hpp>
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
        using hashSet = std::unordered_set<Vector2Int>;
        using Cell = PMFS_internal::Cell;
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
        void OnCompleteNavigation(GSLResult result) override;
        float gasCallback(olfaction_msgs::msg::GasSensor::SharedPtr msg) override;

        void initializeMap();

        //-------------Core-------------
        std::vector<std::vector<Cell>> grid;
        GridData gridData;
        void estimateHitProbabilities(std::vector<std::vector<Cell>>& hitLocalVariable, bool hit, double wind_direction, double wind_speed,
                                      Vector2Int robot_pos, bool infotaxis_sim = false);

        // returns the sum of all auxWeights, for normalization purposes
        double propagateProbabilities(std::vector<std::vector<Cell>>& var, hashSet& openSet, hashSet& closedSet, hashSet& activeSet,
                                      const HitProbKernel& kernel);
        double applyFalloffLogOdds(Vector2 originalVectorScaled, const HitProbKernel& kernel);

        std::vector<std::vector<Vector2>> estimatedWindVectors;
        void estimateWind(bool groundTruth);

        //-------------Data-------------
        PMFS_internal::Simulations simulations;
        PMFS_internal::Settings settings;
        PMFS_internal::PublishersAndSubscribers pubs;

        //-------------Utils-------------
        bool paused = false;
        std::unordered_map<Vector2Int, hashSet> visibilityMap;
        FunctionQueue functionQueue;
        uint iterationsCounter;

        Vector2Int currentPosIndex()
        {
            return gridData.coordinatesToIndex(currentRobotPose.pose.pose.position.x, currentRobotPose.pose.pose.position.y);
        }
        bool pathFree(const Vector2Int& origin, const Vector2Int& end);
        bool indicesInBounds(const Vector2Int indices) const;
        void normalizeSourceProb(std::vector<std::vector<Cell>>& variable);
        Vector2 expectedValueSource(double proportionBest);
        double varianceSourcePosition();
        double sourceProbability(int i, int j);

        // Visualization
        void showWeights();
        void debugMapSegmentation();
        void plotWindVectors();
#ifdef USE_GUI
        PMFS_internal::UI ui;
#endif
    };
} // namespace GSL