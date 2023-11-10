#pragma once

#include <rclcpp/clock.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <gsl_server/core/Vectors.h>

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
    }; // namespace Utils
};     // namespace GSL