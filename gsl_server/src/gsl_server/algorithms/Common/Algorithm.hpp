#pragma once
#include <rclcpp/rclcpp.hpp>
#include <gsl_server/core/ros_typedefs.hpp>
#include <gsl_server/core/Navigation.hpp>
#include <gsl_server/algorithms/Common/Utils/BufferWrapper.hpp>
#include <olfaction_msgs/msg/anemometer.hpp>
#include <olfaction_msgs/msg/gas_sensor.hpp>
#include <gsl_server/algorithms/Common/GSLState.hpp>
#include <gsl_server/core/GSLResult.hpp>
#include <gsl_server/core/Macros.hpp>

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
        friend class GrGSLLib;

    public:
        Algorithm() = delete;
        Algorithm(std::shared_ptr<rclcpp::Node> _node);
        ~Algorithm();
        virtual void Initialize();
        virtual void OnUpdate();

        bool HasEnded();
        GSLResult GetResult();

        template <typename T> T getParam(const std::string& name, T defaultValue)
        {
            if (node->has_parameter(name))
                return node->get_parameter_or<T>(name, defaultValue);
            else
                return node->declare_parameter<T>(name, defaultValue);
        }

        Vector2 currentCoordinates() {return Vector2(currentRobotPose.pose.pose.position.x, currentRobotPose.pose.pose.position.y);}
    protected:        
        virtual void declareParameters();
        virtual GSLResult checkSourceFound();
        virtual void saveResultsToFile(GSLResult result);

        virtual void processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection) = 0; //called from StopAndMeasure once we have enough data for this position

        virtual float gasCallback(const olfaction_msgs::msg::GasSensor::SharedPtr msg);
        virtual PoseStamped windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg);

        virtual void onGetMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        virtual void OnCompleteNavigation(GSLResult result, State* previousState);
        
        
        
        void onGetCostMap(const OccupancyGrid::SharedPtr msg);
        void localizationCallback(const PoseWithCovarianceStamped::SharedPtr msg);


        //Utils
        //-------
        float ppmFromGasMsg(const olfaction_msgs::msg::GasSensor::SharedPtr msg);
        geometry_msgs::msg::PoseStamped getRandomPoseInMap();
        bool isPointInsideMapBounds(const Vector2& point) const;
        bool isPointFree(const Vector2& point);
        int8_t sampleCostmap(const Vector2& point);
        void updateProximityResults(bool forceUpdate = false); //record how close we are to the source for later logging. By default it only adds a new point if the distance has changed a bit since the last write, but you can force it to always write the current


        // Subscriptions
        //-------
        rclcpp::Subscription<olfaction_msgs::msg::GasSensor>::SharedPtr gasSub;
        rclcpp::Subscription<olfaction_msgs::msg::Anemometer>::SharedPtr windSub;
        rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr localizationSub;


        double thresholdGas, thresholdWind;

        std::shared_ptr<rclcpp::Node> node;

        BufferWrapper tfBuffer;

        StateMachines::StateMachine<GSL::State> stateMachine;
        std::unique_ptr<WaitForMapState> waitForMapState;
        std::unique_ptr<WaitForGasState> waitForGasState;
        std::unique_ptr<StopAndMeasureState> stopAndMeasureState;
        std::unique_ptr<MovingState> movingState;

        rclcpp::Time startTime;
        PoseWithCovarianceStamped currentRobotPose;
        OccupancyGrid map;
        OccupancyGrid costmap;

        struct ResultLogging
        {
            std::vector<PoseWithCovarianceStamped> robotPosesVector;
            Vector2 sourcePositionGT;
            double maxSearchTime;
            double distanceThreshold;

            double navigationTime = -1;
            std::string resultsFile;
            std::string navigationPathFile;

            struct ProximityResult
            {
                double time = 0;
                double distance = 100;
            };
            std::vector<ProximityResult> proximityResult;
        };
        ResultLogging resultLogging;
        GSLResult currentResult = GSLResult::Running;
    };
} // namespace GSL