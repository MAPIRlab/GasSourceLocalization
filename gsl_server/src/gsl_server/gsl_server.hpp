#pragma once

#include <gsl_server/core/Logging.hpp>
#include <gsl_server/core/GSLResult.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <gsl_actions/action/do_gsl.hpp>
#include <gsl_server/algorithms/Common/Algorithm.hpp>

using DoGSL = gsl_actions::action::DoGSL;

using namespace std::placeholders;
class GSLServer : public rclcpp::Node
{
protected:
    rclcpp_action::Server<DoGSL>::SharedPtr actionServer;

public:
    GSLServer(std::string name) : Node(name)
    {
        actionServer =
            rclcpp_action::create_server<DoGSL>(this, name, std::bind(&GSLServer::handle_goal, this, _1, _2),
                                                std::bind(&GSLServer::handle_cancel, this, _1), std::bind(&GSLServer::handle_accepted, this, _1));
    }

    ~GSLServer()
    {}
    std::shared_ptr<rclcpp_action::ServerGoalHandle<DoGSL>> m_activeGoal{nullptr};

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const DoGSL::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<DoGSL>> goal_handle);
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<DoGSL>> goal_handle);

    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<DoGSL>> goal_handle);

private:
    // Implement here all the GSL methods: CFD, plume_tracking, etc.
    GSLResult runMethod(std::shared_ptr<GSL::Algorithm> algorithm);
    std::shared_ptr<GSL::Algorithm> createAlgorithm(const std::string name);
};
