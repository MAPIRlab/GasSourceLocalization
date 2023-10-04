#pragma once
#include <algorithms/gsl_algorithm.h>
#include <gmrf_wind_mapping/srv/wind_estimation.hpp>

#ifdef USE_GADEN
    #include <gaden_player/srv/wind_position.hpp>
#endif

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace PMFS
{

    using WindEstimation = gmrf_wind_mapping::srv::WindEstimation;
    struct PublishersAndSubscribers
    {

        WindEstimation::Request::SharedPtr GMRFRequest;
        rclcpp::Client<WindEstimation>::SharedPtr clientWindGMRF;
#ifdef USE_GADEN
        gaden_player::srv::WindPosition::Request::SharedPtr groundTruthWindRequest;
        rclcpp::Client<gaden_player::srv::WindPosition>::SharedPtr clientWindGroundTruth;
#endif

        struct Markers
        {
            rclcpp::Publisher<Marker>::SharedPtr source_probability_markers;
            rclcpp::Publisher<Marker>::SharedPtr hitProbabilityMarkers;
            rclcpp::Publisher<MarkerArray>::SharedPtr quadtreePublisher;
            rclcpp::Publisher<MarkerArray>::SharedPtr gradientMarkers;
            rclcpp::Publisher<MarkerArray>::SharedPtr windArrowMarkers;
            rclcpp::Publisher<Marker>::SharedPtr confidenceMarkers;

            struct Debug
            {
                rclcpp::Publisher<Marker>::SharedPtr explorationValue;
                rclcpp::Publisher<Marker>::SharedPtr varianceHit;
                rclcpp::Publisher<Marker>::SharedPtr movementSets;
            };
            Debug debug;
        };
        Markers markers;
        PublishersAndSubscribers()
        {
        }
    };
} // namespace PMFS