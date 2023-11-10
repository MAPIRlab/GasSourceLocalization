#pragma once

#include <gsl_server/algorithms/Common/GSLState.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <gsl_server/core/Navigation.h>
#include <gsl_server/core/ros_typedefs.h>

namespace GSL
{
    class Algorithm;
    class MovingState : public State
    {
    public:
        MovingState(Algorithm* _algorithm);
        std::optional<nav_msgs::msg::Path> GetPlan(const PoseStamped& start, const PoseStamped& target);
        bool checkGoal(const NavigateToPose::Goal& goal);
        void sendGoal(const NavigateToPose::Goal& goal);
        void OnEnterState(State* previous) override;
        void OnUpdate() override;

    protected:
        rclcpp::Time startTime;
        NavigationClient nav_client;
        void goalDoneCallback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult& result);

#ifdef USE_NAV_ASSISTANT
        rclcpp::Client<MakePlan>::SharedPtr make_plan_client;
#else
        rclcpp_action::Client<MakePlan>::SharedPtr make_plan_client;
#endif
    };
} // namespace GSL