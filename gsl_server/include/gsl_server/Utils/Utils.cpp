#include <gsl_server/Utils/Utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <glm/gtx/rotate_vector.hpp>

namespace GSL::Utils
{

    double lerp(double start, double end, double proportion)
    {
        if (proportion < 0 || std::isnan(proportion))
            return start;

        return start + (end - start) * std::min(1.0, proportion);
    }

    double clamp(double val, double min, double max)
    {
        return std::max(min, std::min(val, max));
    }

    double remapRange(double value, double low1, double high1, double low2, double high2)
    {
        return low2 + (value - low1) * (high2 - low2) / (high1 - low1);
    }

    double evaluate1DGaussian(double distance, double sigma)
    {
        return exp(-0.5 * (std::pow(distance, 2) / std::pow(sigma, 2))) / (sigma * std::sqrt(2 * M_PI));
    }

    double evaluate2DGaussian(const Vector2& sampleOffset, const Vector2& sigma, float distributionRotation)
    {
        // we rotate the vector instead of the gaussian because keeping the distribution axis-aligned removes several terms from the PDF equation
        Vector2 v = glm::rotate(sampleOffset, -distributionRotation);

        return std::exp(-0.5 * (std::pow(v.x / sigma.x, 2) + std::pow(v.y / sigma.y, 2))) / (2 * M_PI * sigma.x * sigma.y);
    }

    double logOddsToProbability(double lo)
    {
        return 1.0 - 1.0 / (1 + std::exp(lo));
    }

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

    geometry_msgs::msg::Pose compose(geometry_msgs::msg::Pose referenceSystem, geometry_msgs::msg::Pose pose)
    {
        double theta1 = getYaw(referenceSystem.orientation);
        double theta2 = getYaw(pose.orientation);
        geometry_msgs::msg::Pose result;
        result.position.x = referenceSystem.position.x + cos(theta1) * pose.position.x - sin(theta1) * pose.position.y;
        result.position.y = referenceSystem.position.y + sin(theta1) * pose.position.x + cos(theta1) * pose.position.y;
        result.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), theta1 + theta2));
        return result;
    }

    geometry_msgs::msg::Point rotateVector(geometry_msgs::msg::Point p1, geometry_msgs::msg::Pose ref)
    {
        double theta = getYaw(ref.orientation);
        geometry_msgs::msg::Point result;
        result.x = std::cos(theta) * p1.x - std::sin(theta) * p1.y;
        result.y = std::sin(theta) * p1.x + std::cos(theta) * p1.y;
        return result;
    }

    double randomFromGaussian(double mean, double stdev)
    {
        static thread_local std::mt19937 engine;

        static thread_local std::normal_distribution<> dist{0, stdev};
        static thread_local double previousStdev = stdev;

        if (stdev != previousStdev)
        {
            dist = std::normal_distribution<>{0, stdev};
            previousStdev = stdev;
        }

        return mean + dist(engine);
    }

    double uniformRandom(double min, double max)
    {
        static thread_local std::mt19937 engine;
        static thread_local std::uniform_real_distribution<double> distribution{0.0, 0.999};

        return min + distribution(engine) * (max - min);
    }

    double KLD(std::vector<std::vector<double>>& a, std::vector<std::vector<double>>& b)
    {
        double total = 0;
        for (int r = 0; r < a.size(); r++)
        {
            for (int c = 0; c < a[0].size(); c++)
            {
                double aux = a[r][c] * std::log(a[r][c] / b[r][c]) + (1 - a[r][c]) * std::log((1 - a[r][c]) / (1 - b[r][c]));
                total += std::isnan(aux) ? 0 : aux;
            }
        }
        return total;
    }

} // namespace GSL::Utils
