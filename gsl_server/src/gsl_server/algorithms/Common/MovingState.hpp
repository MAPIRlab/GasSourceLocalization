#pragma once

#include <gsl_server/algorithms/Common/GSLState.hpp>
#include <gsl_server/core/Navigation.hpp>
#include <gsl_server/core/ros_typedefs.hpp>
#include <optional>
#include <rclcpp_action/rclcpp_action.hpp>

namespace GSL
{
    class Algorithm;
    class MovingState : public State
    {
    public:
        MovingState(Algorithm* _algorithm);
        std::optional<nav_msgs::msg::Path> GetPlan(const PoseStamped& start, const PoseStamped& target); //get a valid path from start to target
        bool checkGoal(const NavigateToPose::Goal& goal); //returns true if we can reach the goal
        void sendGoal(const NavigateToPose::Goal& goal);
        void OnEnterState(State* previous) override;
        void OnExitState(State* next) override;
        void OnUpdate() override;

    protected:
        rclcpp::Time startTime;
        State* previousState;
        NavigationClient nav_client;
        std::optional<NavigateToPose::Goal> currentGoal = std::nullopt;

        // called by the actionServer automatically
        void goalDoneCallback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult& result);

        // Cancel the navigation and call OnCompleteNavigaton
        virtual void Fail();

#ifdef USE_NAV_ASSISTANT
        rclcpp::Client<MakePlan>::SharedPtr make_plan_client;
#else
        rclcpp_action::Client<MakePlan>::SharedPtr make_plan_client;
#endif
    };
} // namespace GSL