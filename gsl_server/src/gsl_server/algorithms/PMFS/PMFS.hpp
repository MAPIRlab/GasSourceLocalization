#pragma once
#include <gsl_server/algorithms/Common/Algorithm.hpp>

#include <gsl_server/algorithms/PMFS/internal/HitProbability.hpp>
#include <gsl_server/algorithms/PMFS/internal/Settings.hpp>
#include <gsl_server/algorithms/PMFS/internal/PublishersAndSubscribers.hpp>
#include <gsl_server/algorithms/PMFS/internal/HitProbKernel.hpp>
#include <gsl_server/algorithms/PMFS/internal/Simulations.hpp>
#include <gsl_server/algorithms/PMFS/internal/UI.hpp>
#include <gsl_server/algorithms/PMFS/internal/VisibilityMap.hpp>
#include <gsl_server/algorithms/PMFS/MovingStatePMFS.hpp>

#include <gsl_server/core/FunctionQueue.hpp>
#include <gsl_server/core/ConditionalMacros.hpp>

#include <tf2_ros/transform_broadcaster.h>

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

        using HitProbability = PMFS_internal::HitProbability;
        using HitProbKernel = PMFS_internal::HitProbKernel;

    public:
        PMFS(std::shared_ptr<rclcpp::Node> _node);
        void Initialize() override;
        void OnUpdate() override;
        Vector2 expectedValueSource(double proportionBest);
        double varianceSourcePosition();

    protected:
        void declareParameters() override;
        void onGetMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) override;
        void processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection) override;
        void processGasAndWindMeasurements(double x, double y, double concentration, double windSpeed, double windDirection) override;
        void updateSourceProbability() override;
        void publishAnemometer(double x, double y, double windSpeed, double windDirection) override;
        Vector2 getExpectedValueSourcePosition() override;
        CovarianceMatrix getVarianceSourcePosition() override;
        void resetMaps() override;

        bool isPaused() override {return paused;}
        void handleUI() override {functionQueue.run();}

        GSLResult checkSourceFound() override;
        void saveResultsToFile(GSLResult result) override;
        void OnCompleteNavigation(GSLResult result, State* previousState) override;
        float gasCallback(olfaction_msgs::msg::GasSensor::SharedPtr msg) override;


        //-------------Core-------------
        Grid2DMetadata gridMetadata;
        std::vector<double> sourceProbability;
        std::vector<HitProbability> hitProbability;
        std::vector<Occupancy> occupancy;
        std::vector<Vector2> estimatedWindVectors;

        PMFS_internal::Simulations simulations;

        //-------------Data-------------
        PMFS_internal::Settings settings;
        PMFS_internal::PublishersAndSubscribers pubs;
        rclcpp::Publisher<olfaction_msgs::msg::Anemometer>::SharedPtr anemometer_pub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        //-------------Utils-------------
        bool paused = false;
        std::optional<VisibilityMap> visibilityMap;
        FunctionQueue functionQueue;
        uint iterationsCounter;


        IF_GUI(PMFS_internal::UI ui);
    };
} // namespace GSL