#pragma once
#include "gsl_server/core/Vectors.hpp"
#include <rclcpp/publisher.hpp>
#include <gsl_server/core/ros_typedefs.hpp>

namespace GSL::GrGSL_internal
{
    struct WindVector
    {
        int col;
        int row;
        double speed;
        double angle;
    };
    struct Cell
    {
        Cell() : sourceProb(0), auxWeight(0), distance(0)
        {}
        double sourceProb;
        double auxWeight, originalPropagatedWeight;
        double distance;
    };

    struct Settings
    {
        float stdevHit;
        float stdevMiss;
        float convergence_thr;

        bool infoTaxis;
        bool allowMovementRepetition;
        bool useDiffusionTerm;
        
        //visualization
        Vector2 colorScaleLimits = {0.0001f, 0.4f};
        bool headless;
    };

    struct Markers
    {
        rclcpp::Publisher<Marker>::SharedPtr probabilityMarkers;
        rclcpp::Publisher<Marker>::SharedPtr estimationMarkers;
        float markersHeight;
    };
}