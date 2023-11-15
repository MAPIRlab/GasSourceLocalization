#pragma once

#include <gsl_server/algorithms/Common/GSLState.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <gsl_server/core/Navigation.hpp>
#include <gsl_server/core/ros_typedefs.hpp>
#include <optional>

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
        void OnExitState(State* next) override;
        void OnUpdate() override;

    protected:
        rclcpp::Time startTime;
        NavigationClient nav_client;
        std::optional<NavigateToPose::Goal> currentGoal = std::nullopt;

        void goalDoneCallback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult& result);

        virtual void Fail();

#ifdef USE_NAV_ASSISTANT
        rclcpp::Client<MakePlan>::SharedPtr make_plan_client;
#else
        rclcpp_action::Client<MakePlan>::SharedPtr make_plan_client;
#endif
    };
} // namespace GSL