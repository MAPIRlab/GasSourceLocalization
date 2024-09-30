/*
    Send a call to the actionserver with the name of the GSL method you want to use
    You can choose which one in the launchfile or commandline with the parameter "method" of this node
*/

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <gsl_actions/action/do_gsl.hpp>
#include <std_msgs/msg/string.hpp>

using DoGSL = gsl_actions::action::DoGSL;

template <typename T> 
static T getParam(rclcpp::Node::SharedPtr node, const std::string& name, T defaultValue)
{
    if (node->has_parameter(name))
        return node->get_parameter_or<T>(name, defaultValue);
    else
        return node->declare_parameter<T>(name, defaultValue);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);


    rclcpp::Node::SharedPtr call_node = std::make_shared<rclcpp::Node>("gsl_call");
    auto resetSimulatorPub = call_node->create_publisher<std_msgs::msg::String>("/basic_sim/reset", rclcpp::QoS(1).transient_local());
    {
        std_msgs::msg::String msg;
        msg.data = "all";
        resetSimulatorPub->publish(msg);
    }

    std::string method = getParam<std::string>(call_node, "method", "surge_cast");

    auto action_client = rclcpp_action::create_client<DoGSL>(call_node, "gsl_server");

    using namespace std::chrono_literals;
    while (rclcpp::ok() && !action_client->wait_for_action_server(10s))
    {
        RCLCPP_INFO(call_node->get_logger(), "Waiting for action server to start.");
    }

    RCLCPP_INFO(call_node->get_logger(), "Action server started, sending goal.");
    DoGSL::Goal goal;
    goal.gsl_method = method;
    rclcpp_action::Client<DoGSL>::SendGoalOptions options;

    bool done = false;
    options.result_callback = [&done](const rclcpp_action::ClientGoalHandle<DoGSL>::WrappedResult & result)
    {
        done = true;
    };
    action_client->async_send_goal(goal);

    rclcpp::Rate rate(1);
    while (!done)
    {
        rclcpp::spin_some(call_node);
        rate.sleep();
    }

    RCLCPP_INFO(call_node->get_logger(), "GSL finished");

    return 0;
}
