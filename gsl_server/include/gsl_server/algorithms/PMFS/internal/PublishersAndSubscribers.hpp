#pragma once
#include <gsl_server/core/ros_typedefs.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/client.hpp>
#include <gmrf_wind_mapping/srv/wind_estimation.hpp>

#ifdef USE_GADEN
#include <gaden_player/srv/wind_position.hpp>
#endif

namespace GSL
{
    namespace PMFS_internal
    {

        using WindEstimation = gmrf_wind_mapping::srv::WindEstimation;
        struct PublishersAndSubscribers
        {
            struct GMRFWind
            {
                WindEstimation::Request::SharedPtr request;
                rclcpp::Client<WindEstimation>::SharedPtr client;
            } gmrfWind;
#ifdef USE_GADEN
            struct GroundTruthWind
            {
                gaden_player::srv::WindPosition::Request::SharedPtr request;
                rclcpp::Client<gaden_player::srv::WindPosition>::SharedPtr client;
            } groundTruthWind;
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
                } debug;
            };
            Markers markers;
            PublishersAndSubscribers()
            {}
        };
    } // namespace PMFS_internal
} // namespace GSL