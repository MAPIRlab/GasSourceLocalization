#pragma once

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

using Pose = geometry_msgs::msg::Pose;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using Point = geometry_msgs::msg::Point;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Marker = visualization_msgs::msg::Marker;
using ColorRGBA = std_msgs::msg::ColorRGBA;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;