#include <filesystem>
#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>
#include <gsl_server/core/Macros.hpp>
#include <gsl_server/core/ros_typedefs.hpp>
#include <map>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/publisher.hpp>
#include <string>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>

namespace GSL::Utils
{
    static rclcpp::Node::SharedPtr debugNode;

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

    std::vector<Occupancy> parseMapImage(const std::string& path, const Grid2DMetadata& metadata)
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
            imageAsVec[i] = (int8_t)std::clamp(100 - (int)mapImage.data[i], 0, 100);

        std::vector<Occupancy> occupancyGrid(width * height / metadata.scale);
        GridUtils::reduceOccupancyMap(imageAsVec, width, occupancyGrid, metadata);

        return occupancyGrid;
    }

    OccupancyGrid toOccupancyGrid(const std::vector<Occupancy>& occupancy, const Grid2DMetadata& metadata)
    {
        OccupancyGrid msg;
        msg.header.frame_id = "map";
        msg.info.resolution = metadata.cellSize;
        msg.info.origin.position.x = metadata.origin.x;
        msg.info.origin.position.y = metadata.origin.y;
        msg.info.width = metadata.dimensions.x;
        msg.info.height = metadata.dimensions.y;

        std::transform(occupancy.begin(), occupancy.end(), std::back_inserter(msg.data), [](const Occupancy value) -> int8_t
                       {
                           return static_cast<int8_t>(value);
                       });
        return msg;
    }

    void parseMapData(const std::string& yamlPath, Grid2DMetadata& outMetadata, std::vector<Occupancy>& outOccupancy)
    {
        if (!std::filesystem::exists(yamlPath))
        {
            GSL_ERROR("Tried to parse map image at path {}, but it does not exist", yamlPath);
            CLOSE_PROGRAM;
        }

        const YAML::Node yaml = YAML::LoadFile(yamlPath);
        outMetadata.origin.x = yaml["origin"][0].as<float>();
        outMetadata.origin.y = yaml["origin"][1].as<float>();
        outMetadata.cellSize = yaml["resolution"].as<float>();
        outMetadata.numFreeCells = 0;
        outMetadata.scale = 1;

        std::filesystem::path imagePath(yaml["image"].as<std::string>());
        if (imagePath.is_relative())
            imagePath = std::filesystem::path(yamlPath).parent_path() / imagePath;

        cv::Mat mapImage = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
        outMetadata.dimensions.x = mapImage.size().width;
        outMetadata.dimensions.y = mapImage.size().height;

        outOccupancy = parseMapImage(imagePath, outMetadata);
    }

    void publishDebugSingleMarker(Vector3 position, ColorRGBA color, const std::string& topic)
    {
        if (!debugNode)
            debugNode = std::make_shared<rclcpp::Node>("debugNode");

        static std::map<std::string, std::shared_ptr<rclcpp::Publisher<Marker>>> publisherMap;

        if (!publisherMap.contains(topic))
            publisherMap[topic] = debugNode->create_publisher<Marker>(topic, 1);
        auto pub = publisherMap[topic];

        Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = debugNode->now();
        marker.type = Marker::SPHERE;

        marker.color = color;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        marker.pose.position.x = position.x;
        marker.pose.position.y = position.y;
        marker.pose.position.z = position.z;

        pub->publish(marker);
    }

    void publishDebugMarkers(Grid2D<std_msgs::msg::ColorRGBA> grid, const std::string& topic)
    {
        if (!debugNode)
            debugNode = std::make_shared<rclcpp::Node>("debugNode");

        static std::map<std::string, std::shared_ptr<rclcpp::Publisher<Marker>>> publisherMap;

        if (!publisherMap.contains(topic))
            publisherMap[topic] = debugNode->create_publisher<Marker>(topic, 1);
        auto pub = publisherMap[topic];

        Marker points;
        points.header.frame_id = "map";
        points.header.stamp = debugNode->now();
        points.type = Marker::POINTS;
        points.action = Marker::ADD;

        points.color.r = 1.0;
        points.color.g = 0.0;
        points.color.b = 1.0;
        points.color.a = 1.0;
        points.scale.x = grid.metadata.cellSize * 0.95;
        points.scale.y = grid.metadata.cellSize * 0.95;

        for (int row = 0; row < grid.metadata.dimensions.y; row++)
        {
            for (int col = 0; col < grid.metadata.dimensions.x; col++)
            {
                if (grid.freeAt(col, row))
                {
                    auto coords = grid.metadata.indicesToCoordinates(col, row);
                    Point p;
                    p.x = coords.x;
                    p.y = coords.y;
                    p.z = 0;

                    points.points.push_back(p);
                    points.colors.push_back(grid.dataAt(col, row));
                }
            }
        }

        // GSL_INFO("Publishing debug markers at {}", pub->get_topic_name());
        pub->publish(points);

    } // namespace GSL::Utils

    void publishDebugSingleArrow(Vector3 start, Vector3 end, std_msgs::msg::ColorRGBA color, const std::string& topic)
    {
        if (!debugNode)
            debugNode = std::make_shared<rclcpp::Node>("debugNode");

        static std::map<std::string, std::shared_ptr<rclcpp::Publisher<Marker>>> publisherMap;

        if (!publisherMap.contains(topic))
            publisherMap[topic] = debugNode->create_publisher<Marker>(topic, 1);
        auto pub = publisherMap[topic];

        Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = debugNode->now();
        marker.type = Marker::ARROW;

        marker.color = color;
        marker.scale.x = 0.1;
        marker.scale.y = 0.2;

        Point startP;
        startP.x = start.x;
        startP.y = start.y;
        startP.z = start.z;

        Point endP;
        endP.x = end.x;
        endP.y = end.y;
        endP.z = end.z;

        marker.points.push_back(startP);
        marker.points.push_back(endP);

        pub->publish(marker);
    }

    void publishDebugSingleArrow(Vector3 start, geometry_msgs::msg::Quaternion rotation, float length, std_msgs::msg::ColorRGBA color, const std::string& topic)
    {
        tf2::Quaternion tf2Quat;
        tf2::fromMsg(rotation, tf2Quat);
        tf2::Vector3 rotated = tf2::quatRotate(tf2Quat, tf2::Vector3(length, 0, 0));
        Vector3 end = start + vmath::FromTF2(rotated);

        publishDebugSingleArrow(start, end, color, topic);
    }

    rclcpp::executors::SingleThreadedExecutor::SharedPtr createExecutor(rclcpp::Node::SharedPtr node)
    {
        rclcpp::ExecutorOptions options;
        options.context = node->get_node_base_interface()->get_context();
        auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(options);
        exec->add_node(node);
        return exec;
    }
} // namespace GSL::Utils