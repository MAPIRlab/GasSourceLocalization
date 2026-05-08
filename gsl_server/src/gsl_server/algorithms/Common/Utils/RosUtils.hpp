#pragma once

#include "gsl_server/algorithms/Common/Utils/ColorMaps.hpp"
#include "gsl_server/core/ros_typedefs.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <gsl_server/algorithms/Common/Grid2D.hpp>
#include <gsl_server/core/Vectors.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <string>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace GSL
{

    namespace Utils
    {
        visualization_msgs::msg::Marker emptyMarker(Vector2 scale, rclcpp::Clock::SharedPtr clock);

        enum class ValueColorMode
        {
            Linear,
            Logarithmic
        };
        std_msgs::msg::ColorRGBA valueToColor(double val, double lowLimit, double highLimit, ValueColorMode mode, Colors::ColorMaps colormap = Colors::ColorMaps::Viridis);
        std_msgs::msg::ColorRGBA create_color(float r, float g, float b, float a = 1.0f);

        geometry_msgs::msg::Pose compose(const geometry_msgs::msg::Pose& referenceSystem, const geometry_msgs::msg::Pose& pose);

        double getYaw(const geometry_msgs::msg::Quaternion& quat);

        geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);

        template <typename T>
        T getParam(rclcpp::Node::SharedPtr node, const std::string& name, T defaultValue)
        {
            if (node->has_parameter(name))
                return node->get_parameter_or<T>(name, defaultValue);
            else
                return node->declare_parameter<T>(name, defaultValue);
        }

        Vector3 fromMsg(const geometry_msgs::msg::Vector3& v);
        Vector3 fromMsg(const geometry_msgs::msg::Point& v);

        std::vector<Occupancy> parseMapImage(const std::string& imagePath, const Grid2DMetadata& metadata);
        Map2D parseMapData(const std::string& yamlPath, std::optional<float> desiredCellSize);
        OccupancyGrid toOccupancyGrid(const Grid2D<Occupancy> grid);

        void publishDebugSingleMarker(Vector3 position, std_msgs::msg::ColorRGBA color, const std::string& topic);
        void publishDebugSingleArrow(Vector3 start, Vector3 end, std_msgs::msg::ColorRGBA color, const std::string& topic);
        void publishDebugSingleArrow(Vector3 start, geometry_msgs::msg::Quaternion rotation, float length, std_msgs::msg::ColorRGBA color, const std::string& topic);
        rclcpp::executors::SingleThreadedExecutor::SharedPtr createExecutor(rclcpp::Node::SharedPtr node);
        void ClearMarkers(rclcpp::Publisher<MarkerArray>::SharedPtr pub);
        Marker createPointsMarker(Grid2D<std_msgs::msg::ColorRGBA> grid, float height = 0);
        MarkerArray createArrowsMarkers(Grid2D<Vector2> vectors,
                                        float height = 0,
                                        float size = 0.05,
                                        std::optional<float> saturateLength = std::nullopt);
        Marker createPointsOccupancyMarker(const Grid2D<Occupancy> occupancy);
        void publishDebugMarkers(Grid2D<std_msgs::msg::ColorRGBA> grid, const std::string& topic);
    }; // namespace Utils
}; // namespace GSL