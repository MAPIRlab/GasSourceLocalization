#include <gsl_server/Utils/RosUtils.h>
#include <gsl_server/Utils/Math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace GSL::Utils
{

    geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
    {
        return tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw));
    }

    visualization_msgs::msg::Marker emptyMarker(Vector2 scale, rclcpp::Clock::SharedPtr clock)
    {
        visualization_msgs::msg::Marker points;
        points.header.frame_id = "map";
        points.header.stamp = clock->now();
        points.ns = "cells";
        points.id = 0;
        points.type = visualization_msgs::msg::Marker::POINTS;
        points.action = visualization_msgs::msg::Marker::ADD;

        points.color = valueToColor(0.5, 0, 1, valueColorMode::Linear);
        points.scale.x = scale.x;
        points.scale.y = scale.y;
        return points;
    }

    std_msgs::msg::ColorRGBA valueToColor(double val, double lowLimit, double highLimit, valueColorMode mode)
    {
        double r, g, b;
        double range;
        if (mode == valueColorMode::Logarithmic)
        {
            val = std::log10(val);
            range = (std::log10(highLimit) - std::log10(lowLimit)) / 4;
            lowLimit = std::log10(lowLimit);
        }
        else
        {
            range = (highLimit - lowLimit) / 4;
        }

        if (val < lowLimit + range)
        {
            r = 0;
            g = lerp(0, 1, (val - lowLimit) / (range));
            b = 1;
        }
        else if (val < lowLimit + 2 * range)
        {
            r = 0;
            g = 1;
            b = lerp(1, 0, (val - (lowLimit + range)) / (range));
        }
        else if (val < lowLimit + 3 * range)
        {
            r = (val - (lowLimit + 2 * range)) / (range);
            g = 1;
            b = 0;
        }
        else
        {
            r = 1;
            g = lerp(1, 0, (val - (lowLimit + 3 * range)) / (range));
            b = 0;
        }
        return create_color(r, g, b, 1);
    }

    std_msgs::msg::ColorRGBA create_color(float r, float g, float b, float a)
    {
        std_msgs::msg::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = a;
        return color;
    }

    double getYaw(const geometry_msgs::msg::Quaternion& quat)
    {
        tf2::Quaternion tfquat;
        tf2::fromMsg(quat, tfquat);

        tf2::Matrix3x3 m(tfquat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    geometry_msgs::msg::Pose compose(const geometry_msgs::msg::Pose& referenceSystem, const geometry_msgs::msg::Pose& pose)
    {
        double theta1 = getYaw(referenceSystem.orientation);
        double theta2 = getYaw(pose.orientation);
        geometry_msgs::msg::Pose result;
        result.position.x = referenceSystem.position.x + cos(theta1) * pose.position.x - sin(theta1) * pose.position.y;
        result.position.y = referenceSystem.position.y + sin(theta1) * pose.position.x + cos(theta1) * pose.position.y;
        result.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), theta1 + theta2));
        return result;
    }

} // namespace GSL::Utils
