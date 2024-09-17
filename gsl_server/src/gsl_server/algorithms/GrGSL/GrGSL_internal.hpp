#pragma once
#include <rclcpp/publisher.hpp>
#include <gsl_server/core/ros_typedefs.hpp>

namespace GSL::GrGSL_internal
{
    struct WindVector
    {
        int i;
        int j;
        double speed;
        double angle;
    };
    struct Cell
    {
        Cell(bool _free, double _weight) : free(_free), weight(_weight), auxWeight(0), distance(0)
        {}
        bool free;
        double weight;
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
    };

    struct Markers
    {
        rclcpp::Publisher<Marker>::SharedPtr probabilityMarkers;
        rclcpp::Publisher<Marker>::SharedPtr estimationMarkers;
        float markersHeight;
    };
}