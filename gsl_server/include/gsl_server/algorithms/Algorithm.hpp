#pragma once
#include <rclcpp/rclcpp.hpp>
#include <gsl_server/core/ros_typedefs.hpp>
#include <gsl_server/core/Navigation.hpp>
#include <gsl_server/Utils/BufferWrapper.hpp>
#include <olfaction_msgs/msg/anemometer.hpp>
#include <olfaction_msgs/msg/gas_sensor.hpp>
#include <gsl_server/algorithms/Common/GSLState.hpp>
#include <gsl_server/core/GSLResult.hpp>
#include <gsl_server/core/logging.hpp>

#include <gsl_server/algorithms/Common/WaitForMapState.hpp>
#include <gsl_server/algorithms/Common/WaitForGasState.hpp>
#include <gsl_server/algorithms/Common/StopAndMeasureState.hpp>
#include <gsl_server/algorithms/Common/MovingState.hpp>
#include <gsl_server/core/Vectors.hpp>

namespace GSL
{

    class Algorithm
    {
        friend class WaitForMapState;
        friend class WaitForGasState;
        friend class StopAndMeasureState;
        friend class MovingState;
        friend class PMFSLib;

    public:
        Algorithm() = delete;
        Algorithm(std::shared_ptr<rclcpp::Node> _node);
        ~Algorithm();
        virtual void initialize();
        virtual void OnUpdate();

        bool hasEnded();
        GSLResult getResult();
        
        template <typename T> T getParam(const std::string& name, T defaultValue)
        {
            if (node->has_parameter(name))
                return node->get_parameter_or<T>(name, defaultValue);
            else
                return node->declare_parameter<T>(name, defaultValue);
        }
    protected:
        virtual void declareParameters();

        double thresholdGas, thresholdWind;

        std::shared_ptr<rclcpp::Node> node;

        BufferWrapper tf_buffer;

        StateMachines::StateMachine<GSL::State> stateMachine;
        std::unique_ptr<WaitForMapState> waitForMapState;
        std::unique_ptr<WaitForGasState> waitForGasState;
        std::unique_ptr<StopAndMeasureState> stopAndMeasureState;
        std::unique_ptr<MovingState> movingState;

        rclcpp::Time start_time;
        PoseWithCovarianceStamped currentRobotPose;
        OccupancyGrid map;
        OccupancyGrid costmap;

        struct ResultLogging
        {
            std::vector<PoseWithCovarianceStamped> robot_poses_vector;
            Vector2 source_pose;
            double max_search_time;
            double distance_found;

            double navigationTime = -1;
            std::string results_file;
            std::string path_file;
            
            struct ProximityResult
            {
                double time = 0;
                double distance = 100;
            };
            std::vector<ProximityResult> proximityResult;
        };
        ResultLogging resultLogging;
        GSLResult currentResult = GSLResult::Running;
        virtual GSLResult checkSourceFound();
        virtual void saveResultsToFile(GSLResult result);

        virtual void processGasAndWindMeasurements(double concentration, double wind_speed, double wind_direction) = 0;
        virtual float gasCallback(const olfaction_msgs::msg::GasSensor::SharedPtr msg);
        virtual PoseStamped windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg);

        virtual void onGetMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        virtual void OnCompleteNavigation(GSLResult result, State* previousState);

        float ppmFromGasMsg(const olfaction_msgs::msg::GasSensor::SharedPtr msg);
        geometry_msgs::msg::PoseStamped getRandomPoseInMap();
        bool isPointInsideMapBounds(const Vector2& point) const;
        bool isPointFree(const Vector2& point);
        int8_t sampleCostmap(const Vector2& point);
        void updateProximityResults(bool forceUpdate=false);


        // Subscriptions
        rclcpp::Subscription<olfaction_msgs::msg::GasSensor>::SharedPtr gas_sub;
        rclcpp::Subscription<olfaction_msgs::msg::Anemometer>::SharedPtr wind_sub;
        rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr localization_sub;

        void onGetCostMap(const OccupancyGrid::SharedPtr msg);
        void localizationCallback(const PoseWithCovarianceStamped::SharedPtr msg);
    };
} // namespace GSL