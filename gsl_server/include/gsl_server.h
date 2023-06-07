#pragma once
#include <rclcpp/rclcpp.hpp>
#include <fstream>      // std::ofstream
#include <rclcpp_action/rclcpp_action.hpp>
#include <gsl_actions/action/do_gsl.hpp>
#include <numeric>      // std::inner_product
#include <eigen3/Eigen/Dense>

using DoGSL=gsl_actions::action::DoGSL;
using namespace std::placeholders;

class CGSLServer : public rclcpp::Node
{
protected:
    rclcpp_action::Server<DoGSL>::SharedPtr actionServer;
    std::string action_name_;

public:
    CGSLServer(std::string name) :
        action_name_(name), Node("gsl_server")
    {
        actionServer = rclcpp_action::create_server<DoGSL>(this, name, 
            std::bind(&handle_goal, this, _1, _2), 
            std::bind(&handle_cancel, this, _1), 
            std::bind(&handle_accepted, this, _1)
        );
    }

    ~CGSLServer(){}

    rclcpp_action::GoalResponse  handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DoGSL::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<DoGSL>> goal_handle);
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<DoGSL>> goal_handle);

    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<DoGSL>> goal_handle);
private:

    //Implement here all the GSL methods: CFD, plume_tracking, etc.
    int doSurgeCast();
    int doSpiral();
    int doSurgeSpiral();
    int doParticleFilter();
    int doPMFS();
    int doGrGSL();
    int gsl_approach;
    std::string resultsFile;
};
