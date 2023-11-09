#include <gsl_server/gsl_server.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto gsl_node = std::make_shared<GSLServer>("gsl_server");
    GSL_INFO("GSL action server is ready for action!");

    while (rclcpp::ok())
    {
        rclcpp::spin_some(gsl_node);
        if (gsl_node->m_activeGoal.get() != nullptr)
        {
            gsl_node->execute(gsl_node->m_activeGoal);
        }
    }

    return 0;
}

rclcpp_action::GoalResponse GSLServer::handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const DoGSL::Goal> goal)
{
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GSLServer::handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<DoGSL>> goal_handle)
{
    rclcpp::shutdown();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void GSLServer::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<DoGSL>> goal_handle)
{
    m_activeGoal = goal_handle;
}

void GSLServer::execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<DoGSL>> goal_handle)
{
    using namespace GSL;
    GSL_INFO("Request received: {}", goal_handle->get_goal()->gsl_method);
    auto actionResult = std::make_shared<DoGSL::Result>();

    // 1. Start the localization of the gas source
    std::shared_ptr<Algorithm> algorithm{nullptr};
    if (goal_handle->get_goal()->gsl_method == "surge_cast")
    {
        algorithm = std::make_shared<SurgeCast>(shared_from_this());
    }
    else if (goal_handle->get_goal()->gsl_method == "spiral")
    {
        algorithm = std::make_shared<Spiral>(shared_from_this());
    }
    else if (goal_handle->get_goal()->gsl_method == "surge_spiral")
    {
        algorithm = std::make_shared<SurgeSpiral>(shared_from_this());
    }
    else if (goal_handle->get_goal()->gsl_method == "particle_filter")
    {
        algorithm = std::make_shared<ParticleFilter>(shared_from_this());
    }
    else if (goal_handle->get_goal()->gsl_method == "GrGSL")
    {
        algorithm = std::make_shared<GrGSL>(shared_from_this());
    }
    else if (goal_handle->get_goal()->gsl_method == "PMFS")
    {
        algorithm = std::make_shared<PMFS>(shared_from_this());
    }
    else
    {
        GSL_ERROR(
            "[GSL_server] Invalid GSL method: \"{}\", candidates are:\n 'surge_cast', 'surge_spiral, 'spiral', 'particle_filter', 'grid', 'PMFS'",
            goal_handle->get_goal()->gsl_method.c_str());
        actionResult->success = false;
        goal_handle->abort(actionResult);
        return;
    }

    GSLResult gslResult = runMethod(algorithm);
    if (gslResult == GSLResult::Success)
    {
        actionResult->success = true;
        GSL_INFO("Found the emission source!");
    }
    else
    {
        actionResult->success = false;
        GSL_INFO("Could not find the source. Sorry!");
    }
    goal_handle->succeed(actionResult);
    rclcpp::shutdown();
}

GSLResult GSLServer::runMethod(std::shared_ptr<GSL::Algorithm> algorithm)
{
    algorithm->initialize();

    rclcpp::Rate rate(20);
    while (rclcpp::ok() && !algorithm->hasEnded())
    {
        algorithm->OnUpdate();
        rate.sleep();
    }
    return algorithm->getResult();
}