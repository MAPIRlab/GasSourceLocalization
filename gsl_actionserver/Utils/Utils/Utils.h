#pragma once

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <random>

#include <Utils/Vector2.h>
#include <Utils/Vector3.h>
#include <Utils/Vector2Int.h>

#include <eigen3/Eigen/Dense>
namespace Utils{
    visualization_msgs::Marker emptyMarker(Vector2 scale);
    double lerp(double start, double end, double proportion);
    double remapRange(double value, double low1, double high1, double low2, double high2);
    double clamp(double val, double min, double max);
    
    enum valueColorMode {Linear, Logarithmic};
    std_msgs::ColorRGBA valueToColor(double val, double low, double high, valueColorMode mode);
    std_msgs::ColorRGBA create_color(float r, float g, float b, float a);
    double evaluate1DGaussian(double distance, double sigma);
    double evaluate2DGaussian(const Vector2& sampleOffset, const Vector2& sigma, float distributionRotation);
    double logOddsToProbability(double l);

    geometry_msgs::Pose compose(geometry_msgs::Pose referenceSystem, geometry_msgs::Pose pose);
    geometry_msgs::Point rotateVector(geometry_msgs::Point vector, geometry_msgs::Pose reference);

    double randomFromGaussian(double mean, double stdev);
    double uniformRandom(double min, double max);

    double KLD(std::vector<std::vector<double> >& a, std::vector<std::vector<double> >& b);

};