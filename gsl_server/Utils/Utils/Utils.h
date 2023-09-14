#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <random>

#include <Utils/Vector2.h>
#include <Utils/Vector3.h>
#include <Utils/Vector2Int.h>

#include <eigen3/Eigen/Dense>
namespace Utils
{
	visualization_msgs::msg::Marker emptyMarker(Vector2 scale, rclcpp::Clock::SharedPtr clock);
	double lerp(double start, double end, double proportion);
	double remapRange(double value, double low1, double high1, double low2, double high2);
	double clamp(double val, double min, double max);

	enum valueColorMode
	{
		Linear,
		Logarithmic
	};
	std_msgs::msg::ColorRGBA valueToColor(double val, double low, double high, valueColorMode mode);
	std_msgs::msg::ColorRGBA create_color(float r, float g, float b, float a);
	double evaluate1DGaussian(double distance, double sigma);
	double evaluate2DGaussian(const Vector2& sampleOffset, const Vector2& sigma, float distributionRotation);
	double logOddsToProbability(double l);

	geometry_msgs::msg::Pose compose(geometry_msgs::msg::Pose referenceSystem, geometry_msgs::msg::Pose pose);
	geometry_msgs::msg::Point rotateVector(geometry_msgs::msg::Point vector, geometry_msgs::msg::Pose reference);

	double getYaw(const geometry_msgs::msg::Quaternion& quat);

	double randomFromGaussian(double mean, double stdev);
	double uniformRandom(double min, double max);

	double KLD(std::vector<std::vector<double>>& a, std::vector<std::vector<double>>& b);

	geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);
};