#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("SendPose");

    double x = node->declare_parameter<double>("x", 0.0);
    double y = node->declare_parameter<double>("y", 0.0);
    double z = node->declare_parameter<double>("z", 0.0);

    std::string topic = node->declare_parameter<std::string>("topic", "/giraff/resetPose");
    auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic, rclcpp::QoS(1).reliable().transient_local());

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id ="map";
    pose.header.stamp = node->now();
    pose.pose.position.x=x;
    pose.pose.position.y=y;
    pose.pose.position.z=z;
    //for some reason the message is lost unless we wait a bit before sendind it... seems like a bug of some king on ROS's end
    rclcpp::sleep_for(std::chrono::seconds(1));
    pub->publish(pose);
    
    RCLCPP_INFO(node->get_logger(), "Sent position (%.2f, %.2f, %.2f) to %s", x, y, z, topic.c_str());

    return 0;
}