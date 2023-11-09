#include <gsl_server/algorithms/Algorithm.h>

namespace GSL
{

    Algorithm::Algorithm(std::shared_ptr<rclcpp::Node> _node) : node(_node), tf_buffer(node->get_clock())
    {
    }

    void Algorithm::initialize()
    {
        // Navigation
        //-----------

#ifdef USE_NAV_ASSISTANT
        make_plan_client = node->create_client<MakePlan>("navigation_assistant/make_plan");
        nav_client = rclcpp_action::create_client<NavigateToPose>(node, "nav_assistant");
#else
        make_plan_client = rclcpp_action::create_client<MakePlan>(node, "compute_path_to_pose");
        nav_client = rclcpp_action::create_client<NavigateToPose>(node, "navigate_to_pose");
#endif

        GSL_INFO("Waiting for the move_base action server to come online...");
        bool mb_aconline = false;

        for (int i = 0; i < 100; i++)
        {
            if (nav_client->wait_for_action_server(std::chrono::seconds(1)))
            {
                mb_aconline = true;
                break;
            }
            GSL_INFO("Unable to find the move_base action server, retrying...");
        }

        if (!mb_aconline)
        {
            GSL_ERROR("No move_base node found. Please ensure the move_base node is active.");
            return;
        }
        GSL_INFO("Found MoveBase! Initializing module...");

        //-----------
        declareParameters();

        // Subscribers
        //------------
        using namespace std::placeholders;
        gas_sub = node->create_subscription<olfaction_msgs::msg::GasSensor>(getParam<std::string>("enose_topic", "PID/Sensor_reading"), 1,
                                                                            std::bind(&Algorithm::gasCallback, this, _1));

        wind_sub = node->create_subscription<olfaction_msgs::msg::Anemometer>(
            getParam<std::string>("anemometer_topic", "Anemometer/WindSensor_reading"), 1, std::bind(&Algorithm::windCallback, this, _1));

        localization_sub = node->create_subscription<PoseWithCovarianceStamped>(getParam<std::string>("robot_location_topic", "amcl_pose"), 100,
                                                                                std::bind(&Algorithm::localizationCallback, this, _1));

        GSL_INFO("INITIALIZATON COMPLETED");
        stateMachine.forceSetState(&waitForMapState);
    }

    void Algorithm::declareParameters()
    {
        resultLogging.max_search_time = getParam<double>("max_search_time", 1000.0);
        resultLogging.distance_found = getParam<double>("distance_found", 0.5);
        resultLogging.source_pose_x = getParam<double>("ground_truth_x", 0.0);
        resultLogging.source_pose_y = getParam<double>("ground_truth_y", 0.0);
        resultLogging.results_file = getParam<std::string>("results_file", "");
        resultLogging.errors_file = getParam<std::string>("errors_file", "");
        resultLogging.path_file = getParam<std::string>("path_file", "");
    }

    void Algorithm::OnUpdate()
    {
        rclcpp::spin_some(node);
        stateMachine.getCurrentState()->OnUpdate();
    }

    bool Algorithm::hasEnded()
    {
        getResult() != GSLResult::Running;
    }

    GSLResult Algorithm::getResult()
    {
        return currentResult;
    }

    void Algorithm::localizationCallback(const PoseWithCovarianceStamped::SharedPtr msg)
    {
        // keep the most recent robot pose
        current_robot_pose = *msg;

        // Keep all poses for later distance estimation
        resultLogging.robot_poses_vector.push_back(current_robot_pose);
    }

    void Algorithm::onGetMap(const OccupancyGrid::SharedPtr msg)
    {
        map = *msg;

        {
            GSL_INFO("--------------GSL---------------");
            GSL_INFO("Occupancy Map dimensions:");
            GSL_INFO("x_min:{:.2} x_max:{:.2}   -   y_min:{:.2} y_max:{:.2}", map.info.origin.position.x,
                     map.info.origin.position.x + map.info.width * map.info.resolution, map.info.origin.position.y,
                     map.info.origin.position.y + map.info.height * map.info.resolution);
            GSL_INFO("--------------------------------");
        }
    }

    void Algorithm::onGetCostMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        costmap = *msg;
    }

    void Algorithm::goalDoneCallback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult& result)
    {
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
            GSL_ERROR("Couldn't reach the target. Navigation goal ReturnCode: {}", (int)result.code);
    }

    bool Algorithm::checkGoal(const NavigateToPose::Goal& goal)
    {
#ifdef USE_NAV_ASSISTANT
        float pos_x = goal.pose.pose.position.x;
        float pos_y = goal.pose.pose.position.y;

        auto request = std::make_shared<MakePlan::Request>();
        request->start.header.frame_id = "map";
        request->start.header.stamp = node->now();
        request->start.pose = current_robot_pose.pose.pose;

        request->goal = goal.pose;

        auto future = make_plan_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(1)) != rclcpp::FutureReturnCode::SUCCESS)
        {
            GSL_WARN("Unable to call make_plan!");
            return false;
        }

        auto response = future.get();
        if (response->plan.poses.empty())
            return false;
        else
            return true;
#else
        MakePlan::Goal plan_request;

        plan_request.use_start = true;
        plan_request.start.pose = current_robot_pose.pose.pose;
        plan_request.start.header = current_robot_pose.header;
        plan_request.goal = goal.pose;

        // send the "make plan" goal to nav2 and wait until the response comes back
        std::optional<nav_msgs::msg::Path> currentPlan = std::nullopt;

        auto callback = [&currentPlan, this](const rclcpp_action::ClientGoalHandle<MakePlan>::WrappedResult& w_result)
        { currentPlan = w_result.result->path; };
        auto goal_options = rclcpp_action::Client<MakePlan>::SendGoalOptions();
        goal_options.result_callback = callback;

        auto future = make_plan_client->async_send_goal(plan_request, goal_options);
        auto result = rclcpp::spin_until_future_complete(node, future);

        // Check if valid goal with move_base srv
        if (result != rclcpp::FutureReturnCode::SUCCESS)
        {
            // SRV is not available!! Report Error
            GSL_INFO("Unable to call MAKE_PLAN service from MoveBase");
            return false;
        }

        // wait until path is received. The spin-until-future-complete line only blocks until the request is accepted!
        rclcpp::Rate wait_rate(200);
        while (!currentPlan.has_value())
        {
            wait_rate.sleep();
            rclcpp::spin_some(node);
        }

        if (currentPlan.value().poses.empty())
        {
            GSL_WARN("Unable to get plan");
            return true;
        }
        return true;
#endif
    }

    void Algorithm::sendGoal(const NavigateToPose::Goal& goal)
    {
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
        options.result_callback = std::bind(&Algorithm::goalDoneCallback, this, std::placeholders::_1);
        auto future = nav_client->async_send_goal(goal, options);
        rclcpp::FutureReturnCode code = rclcpp::spin_until_future_complete(node, future);
        if (code != rclcpp::FutureReturnCode::SUCCESS)
        {
            GSL_ERROR("Error sending goal to navigation server! Received code {}", code);
            rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult result;
            result.code = rclcpp_action::ResultCode::ABORTED;
            goalDoneCallback(result);
        }
    }

    GSLResult Algorithm::checkSourceFound()
    {
        // 1. Check that working time < max allowed time for search
        rclcpp::Duration time_spent = node->now() - start_time;
        if (time_spent.seconds() > resultLogging.max_search_time)
        {
            // Report failure, we were too slow
            GSL_INFO("FAILURE-> Time spent ({} s) > max_search_time = {}", time_spent.seconds(), resultLogging.max_search_time);
            save_results_to_file(GSLResult::Failure);
            return GSLResult::Failure;
        }

        // 2. Distance from robot to source
        double Ax = current_robot_pose.pose.pose.position.x - resultLogging.source_pose_x;
        double Ay = current_robot_pose.pose.pose.position.y - resultLogging.source_pose_y;
        double dist = sqrt(pow(Ax, 2) + pow(Ay, 2));
        if (dist < resultLogging.distance_found)
        {
            // GSL has finished with success!
            GSL_INFO("SUCCESS -> Time spent ({} s)", time_spent.seconds());
            save_results_to_file(GSLResult::Success);
            return GSLResult::Success;
        }

        // In other case, we are still searching (keep going)
        return GSLResult::Running;
    }
} // namespace GSL