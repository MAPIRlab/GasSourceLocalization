#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <gsl_actions/action/do_gsl.hpp>

using DoGSL = gsl_actions::action::DoGSL;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr call_node = std::make_shared<rclcpp::Node>("gsl_call");
    call_node->declare_parameter<std::string>("method", "surge_cast");
    std::string method = call_node->get_parameter("method").as_string();

    auto action_client = rclcpp_action::create_client<DoGSL>(call_node, "gsl");

    RCLCPP_INFO(call_node->get_logger(), "Waiting for action server to start.");

    using namespace std::chrono_literals;
    while (rclcpp::ok() && !action_client->wait_for_action_server(10s))
        ;

    RCLCPP_INFO(call_node->get_logger(), "Action server started, sending goal.");
    DoGSL::Goal goal;
    goal.gsl_method = method;
    rclcpp_action::Client<DoGSL>::SendGoalOptions options;

    bool done = false;
    options.result_callback = [&done](const rclcpp_action::ClientGoalHandle<DoGSL>::WrappedResult& result)
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
