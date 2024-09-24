#include <filesystem>
#include <gsl_server/core/ros_typedefs.hpp>
#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>
#include <gsl_server/core/Macros.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
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

    Vector3 fromMsg(const geometry_msgs::msg::Vector3& v)
    {
        return Vector3(v.x, v.y, v.z);
    }

    Vector3 fromMsg(const geometry_msgs::msg::Point& v)
    {
        return Vector3(v.x, v.y, v.z);
    }

    std::vector<Occupancy> parseMapImage(const std::string& path, Grid2DMetadata& metadata)
    {
        if (!std::filesystem::exists(path))
        {
            GSL_ERROR("Tried to parse map image at path {}, but it does not exist", path);
            CLOSE_PROGRAM;
        }

        cv::Mat mapImage = cv::imread(path, cv::IMREAD_GRAYSCALE);
        cv::flip(mapImage, mapImage, 0);
        size_t width = mapImage.size().width;
        size_t height = mapImage.size().height;
        std::vector<int8_t> imageAsVec(width * height);
        for (int i = 0; i < width * height; i++)
            imageAsVec[i] = (int8_t) std::clamp(100 - (int)mapImage.data[i], 0, 100);

        std::vector<Occupancy> occupancyGrid(width * height / metadata.scale);
        GridUtils::reduceOccupancyMap(imageAsVec, width, occupancyGrid, metadata);

        return occupancyGrid;
    }

    void publishDebugMarkers(Grid2D<std_msgs::msg::ColorRGBA> grid)
    {
        static auto debugNode = std::make_shared<rclcpp::Node>("debugNode");
        static auto pub = debugNode->create_publisher<Marker>("/debugMarkers", 1);


        constexpr auto emptyMarker = []()
                                     {
                                         Marker points;
                                         points.header.frame_id = "map";
                                         points.ns = "cells";
                                         points.id = 0;
                                         points.type = Marker::POINTS;
                                         points.action = Marker::ADD;

                                         points.color.r = 1.0;
                                         points.color.g = 0.0;
                                         points.color.b = 1.0;
                                         points.color.a = 1.0;
                                         points.scale.x = 0.15;
                                         points.scale.y = 0.15;
                                         return points;
                                     };

        Marker points = emptyMarker();
        points.header.stamp = debugNode->now();

        for (int row = 0; row < grid.metadata.dimensions.y; row++)
        {
            for (int col = 0; col < grid.metadata.dimensions.x; col++)
            {
                if (grid.freeAt(col, row))
                {
                    auto coords = grid.metadata.indexToCoordinates(col, row);
                    Point p;
                    p.x = coords.x;
                    p.y = coords.y;
                    p.z = 0;

                    points.points.push_back(p);
                    points.colors.push_back(grid.dataAt(col,row));
                }
            }
        }
        GSL_INFO("Publishing debug markers at {}", pub->get_topic_name());
        pub->publish(points);

    } // namespace GSL::Utils
}