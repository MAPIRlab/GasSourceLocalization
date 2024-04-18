#include <gsl_server/algorithms/Common/MovingState.hpp>
#include <gsl_server/core/Logging.hpp>
#include <gsl_server/core/GSLResult.hpp>
#include <gsl_server/algorithms/Algorithm.hpp>

#define NAVIGATION_FIXES 0 // enables some navigation checks that should be handled by nav2 directly, but can cause problems if it is not correcly configured
                           // honestly, don't use this, just configure nav2 properly

namespace GSL
{
#if NAVIGATION_FIXES
    static constexpr int max_navigation_time = 20;
    static constexpr int8_t lethal_cost = 70;
#endif 

    MovingState::MovingState(Algorithm* _algorithm) : State(_algorithm)
    {
#ifdef USE_NAV_ASSISTANT
        make_plan_client = algorithm->node->create_client<MakePlan>("navigation_assistant/make_plan");
        nav_client = rclcpp_action::create_client<NavigateToPose>(algorithm->node, "nav_assistant");
#else
        make_plan_client = rclcpp_action::create_client<MakePlan>(algorithm->node, "compute_path_to_pose");
        nav_client = rclcpp_action::create_client<NavigateToPose>(algorithm->node, "navigate_to_pose");
#endif

        GSL_INFO("Waiting for the move_base action server to come online...");

        for (int i = 0; i < 100; i++)
        {
            if (nav_client->wait_for_action_server(std::chrono::seconds(1)))
            {
                GSL_INFO("Found MoveBase!");
                return;
            }
            GSL_INFO("Unable to find the move_base action server, retrying...");
        }

        GSL_ERROR("Timed out waiting for navigation action_server to become available. Closing node.");
        rclcpp::shutdown();
    }

    void MovingState::OnEnterState(State* previous)
    {
        GSL_TRACE("Entering Moving");
        GSL_ASSERT_MSG(
            currentGoal.has_value(),
            "Entering moving state without a goal set! You should never set this state directly, only through 'sendGoal' or 'chooseGoalAndMove'");
        startTime = algorithm->node->now();
		previousState = previous;
    }

    void MovingState::OnUpdate()
    {  
#if NAVIGATION_FIXES   
        if ((algorithm->node->now() - startTime).seconds() > max_navigation_time)
        {
            GSL_ERROR("Timed out trying to reach target. Cancelling navigation");
            nav_client->async_cancel_all_goals();
            Fail();
        }
#endif
    }

    void MovingState::OnExitState(State* next)
    {
        if (next != this)
            currentGoal = std::nullopt;
    }

    void MovingState::goalDoneCallback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult& result)
    {
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
        {
            GSL_ERROR("Couldn't reach the target. Navigation goal ReturnCode: {}", (int)result.code);
            Fail();
        }
        else
        {
            algorithm->OnCompleteNavigation(GSLResult::Success, previousState);
        }
    }

    void MovingState::sendGoal(const NavigateToPose::Goal& goal)
    {
        GSL_ASSERT_MSG(currentGoal == std::nullopt, "Sending new goal while there is one active");
        GSL_INFO("Sending goal ({:.2f}, {:.2f})", goal.pose.pose.position.x, goal.pose.pose.position.y);
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
        options.result_callback = std::bind(&MovingState::goalDoneCallback, this, std::placeholders::_1);
        auto future = nav_client->async_send_goal(goal, options);
        rclcpp::FutureReturnCode code = rclcpp::spin_until_future_complete(algorithm->node, future, std::chrono::seconds(1));

        currentGoal = goal;
        if (code != rclcpp::FutureReturnCode::SUCCESS)
        {
            GSL_ERROR("Error sending goal to navigation server! Received code {}", (int)code);
            rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult result;
            result.code = rclcpp_action::ResultCode::ABORTED;
            Fail();
        }
        else
        {
            algorithm->stateMachine.forceResetState(this);
        }
    }

    std::optional<nav_msgs::msg::Path> MovingState::GetPlan(const PoseStamped& start, const PoseStamped& target)
    {
        std::optional<nav_msgs::msg::Path> currentPlan = std::nullopt;

#ifdef USE_NAV_ASSISTANT

        auto request = std::make_shared<MakePlan::Request>();
        request->start.header.frame_id = "map";
        request->start.header.stamp = algorithm->node->now();
        request->start.pose = start;

        request->goal.header.frame_id = "map";
        request->goal.header.stamp = algorithm->node->now();
        request->goal.pose = target;

        auto future = make_plan_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(algorithm->node, future, std::chrono::seconds(1)) != rclcpp::FutureReturnCode::SUCCESS)
        {
            GSL_WARN("Unable to call make_plan!");
            return std::nullopt;
        }

        auto response = future.get();
        currentPlan = response->plan;
#else
        MakePlan::Goal plan_request;

        plan_request.use_start = true;
        plan_request.start = start;
        plan_request.goal = target;

        // send the "make plan" goal to nav2 and wait until the response comes back

        auto callback = [&currentPlan, this](const rclcpp_action::ClientGoalHandle<MakePlan>::WrappedResult& w_result) {
            currentPlan = w_result.result->path;
        };
        auto goal_options = rclcpp_action::Client<MakePlan>::SendGoalOptions();
        goal_options.result_callback = callback;

        auto future = make_plan_client->async_send_goal(plan_request, goal_options);
        auto result = rclcpp::spin_until_future_complete(algorithm->node, future, std::chrono::seconds(1));

        // Check if valid goal with move_base srv
        if (result != rclcpp::FutureReturnCode::SUCCESS)
        {
            // SRV is not available!! Report Error
            GSL_INFO("Unable to call MAKE_PLAN service from MoveBase");
            return std::nullopt;
        }

        // wait until path is received. The spin-until-future-complete line only blocks until the request is accepted!
        rclcpp::Rate wait_rate(200);
        while (!currentPlan.has_value())
        {
            wait_rate.sleep();
            rclcpp::spin_some(algorithm->node);
        }

#endif
        return currentPlan;
    }

    bool MovingState::checkGoal(const NavigateToPose::Goal& goal)
    {
        Vector2 goalPosition = {goal.pose.pose.position.x, goal.pose.pose.position.y};
        if(!algorithm->isPointInsideMapBounds(goalPosition)       
#if NAVIGATION_FIXES
            || !algorithm->sampleCostmap(goalPosition) > lethal_cost
#endif
            )
            return false;

        PoseStamped start;
        start.pose = algorithm->currentRobotPose.pose.pose;
        start.header = algorithm->currentRobotPose.header;

        auto plan = GetPlan(start, goal.pose);

        if (!plan.has_value() || plan.value().poses.empty())
        {
            GSL_WARN("Unable to get plan");
            return false;
        }
        return true;
    }

    void MovingState::Fail()
    {
        if (!currentGoal.has_value())
            return;
        currentGoal = std::nullopt;
        algorithm->OnCompleteNavigation(GSLResult::Failure, previousState);
    }
} // namespace GSL