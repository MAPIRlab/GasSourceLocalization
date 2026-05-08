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

        points.color = valueToColor(0.5, 0, 1, ValueColorMode::Linear);
        points.scale.x = scale.x;
        points.scale.y = scale.y;
        return points;
    }

    std_msgs::msg::ColorRGBA valueToColor(double val, double lowLimit, double highLimit, ValueColorMode mode, Colors::ColorMaps colormap)
    {
        auto [r, g, b] = Colors::SampleColorMap(val, colormap);
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
            if (mapImage.data[i] == 255)
                imageAsVec[i] = (int)Occupancy::Free;
            else if (mapImage.data[i] == 0)
                imageAsVec[i] = (int)Occupancy::Obstacle;
            else
                imageAsVec[i] = (int)Occupancy::Unknown;

        std::vector<Occupancy> occupancyGrid(metadata.dimensions.x * metadata.dimensions.y);
        GridUtils::reduceOccupancyMap(imageAsVec, width, height, occupancyGrid, metadata);

        return occupancyGrid;
    }

    OccupancyGrid toOccupancyGrid(const Grid2D<Occupancy> grid)
    {
        OccupancyGrid msg;
        msg.header.frame_id = "map";
        msg.info.resolution = grid.metadata.cellSize;
        msg.info.origin.position.x = grid.metadata.origin.x;
        msg.info.origin.position.y = grid.metadata.origin.y;
        msg.info.width = grid.metadata.dimensions.x;
        msg.info.height = grid.metadata.dimensions.y;

        std::transform(grid.occupancy.begin(), grid.occupancy.end(), std::back_inserter(msg.data), [](const Occupancy value) -> int8_t
                       { return static_cast<int8_t>(value); });
        return msg;
    }

    Map2D parseMapData(const std::string& yamlPath, std::optional<float> desiredCellSize)
    {
        if (!std::filesystem::exists(yamlPath))
        {
            GSL_ERROR("Tried to parse map image at path {}, but it does not exist", yamlPath);
            CLOSE_PROGRAM;
        }

        Map2D map;

        const YAML::Node yaml = YAML::LoadFile(yamlPath);
        float originalCellSize = yaml["resolution"].as<float>();
        if (!desiredCellSize)
            desiredCellSize = originalCellSize;

        map.metadata.scale = *desiredCellSize / originalCellSize;
        map.metadata.origin.x = yaml["origin"][0].as<float>();
        map.metadata.origin.y = yaml["origin"][1].as<float>();
        map.metadata.cellSize = originalCellSize * map.metadata.scale;
        map.metadata.numFreeCells = 0;

        std::filesystem::path imagePath(yaml["image"].as<std::string>());
        if (imagePath.is_relative())
            imagePath = std::filesystem::path(yamlPath).parent_path() / imagePath;

        cv::Mat mapImage = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
        map.metadata.dimensions.x = std::ceil(mapImage.size().width / (float)map.metadata.scale);
        map.metadata.dimensions.y = std::ceil(mapImage.size().height / (float)map.metadata.scale);

        map.occupancy = parseMapImage(imagePath, map.metadata);

        for (size_t i = 0; i < map.occupancy.size(); i++)
            if (map.occupancy.at(i) == Occupancy::Free)
                map.metadata.numFreeCells++;
        return map;
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

    Marker createPointsMarker(Grid2D<std_msgs::msg::ColorRGBA> grid, float height)
    {
        Marker points;
        points.header.frame_id = "map";
        points.type = Marker::POINTS;
        points.action = Marker::ADD;

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
                    p.z = height;

                    points.points.push_back(p);
                    points.colors.push_back(grid.dataAt(col, row));
                }
            }
        }
        return points;
    }

    MarkerArray createArrowsMarkers(Grid2D<Vector2> vectors,
                                    float height,
                                    float size,
                                    std::optional<float> saturateLength)
    {
        MarkerArray arrow_array;
        // Add an ARROW marker for each node
        Marker marker;
        marker.header.frame_id = "map";
        marker.ns = "WindVector";
        marker.type = Marker::ARROW;
        marker.action = Marker::ADD;

        // Get max wind vector in the map (to normalize the plot)
        double max_module;
        if (saturateLength)
            max_module = *saturateLength;
        else
        {
            max_module = 0.0;
            for (size_t i = 0; i < vectors.data.size(); i++)
            {
                if (vmath::length(vectors.data[i]) > max_module)
                    max_module = vmath::length(vectors.data[i]);
            }
        }

        for (size_t i = 0; i < vectors.data.size(); i++)
        {
            if (vectors.occupancy[i] == Occupancy::Free)
            {
                double module = vmath::length(vectors.data[i]);
                double angle = std::atan2(vectors.data[i].y, vectors.data[i].x);
                if (module > 0.001)
                {
                    marker.id = i;
                    // Set the pose of the marker.
                    Vector2Int indices2D = vectors.metadata.indices2D(i);
                    Vector2 coords = vectors.metadata.indicesToCoordinates(indices2D.x, indices2D.y);
                    marker.pose.position.x = coords.x;
                    marker.pose.position.y = coords.y;
                    marker.pose.position.z = height;
                    marker.pose.orientation = Utils::createQuaternionMsgFromYaw(angle);
                    // shape
                    marker.scale.x = vectors.metadata.cellSize * std::clamp(module / max_module, 0., 1.); // arrow length,
                    marker.scale.y = size;                                                                // arrow width
                    marker.scale.z = size;                                                                // arrow height

                    // if we have a manually specified speed to correspond to the max arrow length, but this exceeds it, give it a different color
                    if (module <= max_module)
                        marker.color = create_color(0.7, 0.7, 0.7);
                    else
                        marker.color = create_color(1, 0, 0);

                    // Push Arrow to array
                    arrow_array.markers.push_back(marker);
                }
            }
        }
        return arrow_array;
    }

    Marker createPointsOccupancyMarker(const Grid2D<Occupancy> occupancy)
    {
        Marker points;
        points.header.frame_id = "map";
        points.type = Marker::POINTS;
        points.action = Marker::ADD;

        points.scale.x = occupancy.metadata.cellSize * 0.95;
        points.scale.y = occupancy.metadata.cellSize * 0.95;

        for (int row = 0; row < occupancy.metadata.dimensions.y; row++)
        {
            for (int col = 0; col < occupancy.metadata.dimensions.x; col++)
            {
                auto coords = occupancy.metadata.indicesToCoordinates(col, row);
                Point p;
                p.x = coords.x;
                p.y = coords.y;
                p.z = 0;

                points.points.push_back(p);

                Occupancy occ = occupancy.occupancyAt(col, row);
                if (occ == Occupancy::Free)
                    points.colors.push_back(create_color(1, 1, 1));
                else if (occ == Occupancy::Unknown)
                    points.colors.push_back(create_color(0.3, 0.3, 0.3));
                else
                    points.colors.push_back(create_color(0, 0, 0));
            }
        }
        return points;
    }

    void publishDebugMarkers(Grid2D<std_msgs::msg::ColorRGBA> grid, const std::string& topic)
    {
        if (!debugNode)
            debugNode = std::make_shared<rclcpp::Node>("debugNode");

        static std::map<std::string, std::shared_ptr<rclcpp::Publisher<Marker>>> publisherMap;

        if (!publisherMap.contains(topic))
            publisherMap[topic] = debugNode->create_publisher<Marker>(topic, 1);
        auto pub = publisherMap[topic];
        Marker points = createPointsMarker(grid);
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

    void ClearMarkers(rclcpp::Publisher<MarkerArray>::SharedPtr pub)
    {
        // clear old data
        Marker clear;
        clear.action = Marker::DELETEALL;
        MarkerArray array;
        array.markers.push_back(clear);
        pub->publish(array);
    }
} // namespace GSL::Utils