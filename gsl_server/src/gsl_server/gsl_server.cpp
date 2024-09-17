/*
    Waits until the actionserver receives a request. When one is received, instantiates the corresponding algorithm and starts the search.
    Each of the methods is guarded by preprocessor definitions so you can omit compiling them if you dont want to install their dependencies (see the CMakeLists)
*/

#include <gsl_server/gsl_server.hpp>

#if ENABLE_PLUME_TRACKING
    #include <gsl_server/algorithms/PlumeTracking/SurgeCast/SurgeCast.hpp>
    #include <gsl_server/algorithms/PlumeTracking/SurgeSpiral/SurgeSpiral.hpp>
#endif
#if ENABLE_SPIRAL
    #include <gsl_server/algorithms/Spiral/Spiral.hpp>
#endif
#if ENABLE_PARTICLE_FILTER
    #include <gsl_server/algorithms/ParticleFilter/ParticleFilter.hpp>
#endif
#if ENABLE_GrGSL
    #include <gsl_server/algorithms/GrGSL/GrGSL.hpp>
#endif
#if ENABLE_PMFS
    #include <gsl_server/algorithms/PMFS/PMFS.hpp>
#endif
#if ENABLE_SEMANTICS
    #include <gsl_server/algorithms/Semantics/SemanticPMFS/SemanticPMFS.hpp>
#endif

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // arbitrary wait to let the simulation start. Not strictly required, but avoids some annoying problems
    rclcpp::sleep_for(std::chrono::seconds(5));

    auto gsl_node = std::make_shared<GSLServer>("gsl_server");
    GSL_INFO("GSL action server is ready for action!");

    // Wait for requests
    rclcpp::Rate rate(1);
    while (rclcpp::ok())
    {
        rate.sleep();
        rclcpp::spin_some(gsl_node);
        if (gsl_node->m_activeGoal.get() != nullptr)
        {
            //in debug mode, don't catch the exception! It prevents the debugger from automatically stopping in the offending line
#if GSL_DEBUG
            gsl_node->execute(gsl_node->m_activeGoal);
#else
            try
            {
                gsl_node->execute(gsl_node->m_activeGoal);
            }
            catch (std::exception& e)
            {
                GSL_ERROR("Exception while running GSL: {}", e.what());
            }
#endif

            rclcpp::sleep_for(std::chrono::seconds(1));
            rclcpp::shutdown();
        }
    }

    return 0;
}

//Main algorithm loop
//-------------------
GSLResult GSLServer::runMethod(std::shared_ptr<GSL::Algorithm> algorithm)
{
    algorithm->Initialize();

    rclcpp::Rate rate(20);
    while (rclcpp::ok() && !algorithm->HasEnded())
    {
        algorithm->OnUpdate();
        rate.sleep();
    }
    return algorithm->GetResult();
}


//ActionServer Setup
//-----------------
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


//Request processing
//---------------------
void GSLServer::execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<DoGSL>> goal_handle)
{
    using namespace GSL;
    GSL_INFO_COLOR(fmt::terminal_color::yellow, "Request received: {}", goal_handle->get_goal()->gsl_method);
    auto actionResult = std::make_shared<DoGSL::Result>();

    // 1. Start the localization of the gas source
    std::shared_ptr<Algorithm> algorithm = createAlgorithm(goal_handle->get_goal()->gsl_method);
    if (!algorithm)
    {
        GSL_ERROR("Invalid GSL method: \"{}\", candidates are:\n 'surge_cast', 'surge_spiral, 'spiral', 'particle_filter', 'GrGSL', 'PMFS'",
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
}

std::shared_ptr<GSL::Algorithm> GSLServer::createAlgorithm(const std::string name)
{
    if (false) // just so we can disable the first one with a preprocessor define without breaking the if
    {}
#if ENABLE_PLUME_TRACKING
    else if (name == "surge_cast")
    {
        return std::make_shared<GSL::SurgeCast>(shared_from_this());
    }
    else if (name == "surge_spiral")
    {
        return std::make_shared<GSL::SurgeSpiral>(shared_from_this());
    }
#endif
#if ENABLE_SPIRAL
    else if (name == "spiral")
    {
        return std::make_shared<GSL::Spiral>(shared_from_this());
    }
#endif
#if ENABLE_PARTICLE_FILTER
    else if (name == "particle_filter")
    {
        return std::make_shared<GSL::ParticleFilter>(shared_from_this());
    }
#endif
#if ENABLE_GrGSL
    else if (name == "GrGSL")
    {
        return std::make_shared<GSL::GrGSL>(shared_from_this());
    }
#endif
#if ENABLE_PMFS
    else if (name == "PMFS")
    {
        return std::make_shared<GSL::PMFS>(shared_from_this());
    }
#endif
#if ENABLE_SEMANTICS
    else if (name == "SemanticPMFS")
    {
        return std::make_shared<GSL::SemanticPMFS>(shared_from_this());
    }
#endif
    return nullptr;
}
