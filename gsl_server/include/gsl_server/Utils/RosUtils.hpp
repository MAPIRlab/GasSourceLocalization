#pragma once

#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <gsl_server/core/Vectors.hpp>
#include <string>

namespace GSL
{

    namespace Utils
    {
        visualization_msgs::msg::Marker emptyMarker(Vector2 scale, rclcpp::Clock::SharedPtr clock);

        enum valueColorMode
        {
            Linear,
            Logarithmic
        };
        std_msgs::msg::ColorRGBA valueToColor(double val, double low, double high, valueColorMode mode);
        std_msgs::msg::ColorRGBA create_color(float r, float g, float b, float a);

        geometry_msgs::msg::Pose compose(const geometry_msgs::msg::Pose& referenceSystem, const geometry_msgs::msg::Pose& pose);

        double getYaw(const geometry_msgs::msg::Quaternion& quat);

        geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);

        template <typename T> T getParam(rclcpp::Node::SharedPtr node, const std::string& name, T defaultValue)
        {
            if (node->has_parameter(name))
                return node->get_parameter_or<T>(name, defaultValue);
            else
                return node->declare_parameter<T>(name, defaultValue);
        }

        Vector3 fromMsg(const geometry_msgs::msg::Vector3& v);
        Vector3 fromMsg(const geometry_msgs::msg::Point& v);
    }; // namespace Utils
};     // namespace GSL