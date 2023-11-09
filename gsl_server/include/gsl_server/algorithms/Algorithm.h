#pragma once
#include <rclcpp.hpp>
#include <gsl_server/core/ros_typedefs.h>
#include <gsl_server/core/Navigation.h>
#include <gsl_server/Utils/BufferWrapper.h>
#include <olfaction_msgs/msg/anemometer.hpp>
#include <olfaction_msgs/msg/gas_sensor.hpp>
#include <gsl_server/algorithms/Common/GSLState.h>
#include <gsl_server/core/GSLResult.h>
#include <gsl_server/core/logging.h>

#include <gsl_server/algorithms/Common/WaitForMapState.h>
#include <gsl_server/algorithms/Common/StopAndMeasureState.h>

namespace GSL
{
    class Algorithm
    {
        friend class WaitForMapState;
        friend class StopAndMeasureState;

    public:
        Algorithm() = delete;
        Algorithm(std::shared_ptr<rclcpp::Node> _node);
        ~Algorithm() = default;
        virtual void initialize();
        virtual void OnUpdate();

        bool hasEnded();
        GSLResult getResult();

    protected:
        virtual void declareParameters();
        bool checkGoal(const NavigateToPose::Goal& goal);
        void sendGoal(const NavigateToPose::Goal& goal);

        std::shared_ptr<rclcpp::Node> node;
        NavigationClient nav_client;
#ifdef USE_NAV_ASSISTANT
        rclcpp::Client<MakePlan>::SharedPtr make_plan_client;
#else
        rclcpp_action::Client<MakePlan>::SharedPtr make_plan_client;
#endif
        BufferWrapper tf_buffer;

        StateMachines::StateMachine<GSL::State> stateMachine;
        WaitForMapState waitForMapState;
        StopAndMeasureState stopAndMeasureState;

        rclcpp::Time start_time;
        PoseWithCovarianceStamped current_robot_pose;
        OccupancyGrid map;
        OccupancyGrid costmap;

        struct ResultLogging
        {
            std::vector<PoseWithCovarianceStamped> robot_poses_vector;
            double source_pose_x, source_pose_y;
            double max_search_time;
            double distance_found;

            std::string results_file;
            std::string errors_file;
            std::string path_file;
        };
        ResultLogging resultLogging;
        GSLResult currentResult = GSLResult::Running;
        virtual GSLResult checkSourceFound();
        virtual void save_results_to_file(GSLResult result);

        template <typename T> T getParam(const std::string& name, T defaultValue)
        {
            if (node->has_parameter(name))
                return node->get_parameter_or<T>(name, defaultValue);
            else
                return node->declare_parameter<T>(name, defaultValue);
        }

        virtual void gasCallback(const olfaction_msgs::msg::GasSensor::SharedPtr msg) = 0;
        virtual void windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg) = 0;
        virtual void onGetMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) = 0;
        virtual void goalDoneCallback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult& result);

    private:
        // Subscriptions
        rclcpp::Subscription<olfaction_msgs::msg::GasSensor>::SharedPtr gas_sub;   //! Gas readings subscriber
        rclcpp::Subscription<olfaction_msgs::msg::Anemometer>::SharedPtr wind_sub; //! Wind readings subscriber
        rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr localization_sub;

        void onGetCostMap(const OccupancyGrid::SharedPtr msg);
        void localizationCallback(const PoseWithCovarianceStamped::SharedPtr msg);
    };
} // namespace GSL