#pragma once
#include <gsl_server/core/ros_typedefs.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/client.hpp>
#include <gmrf_msgs/srv/wind_estimation.hpp>

#ifdef USE_GADEN
#include <gaden_msgs/srv/wind_position.hpp>
#endif

namespace GSL
{
    namespace PMFS_internal
    {

        using WindEstimation = gmrf_msgs::srv::WindEstimation;

        struct GMRFWind
        {
            WindEstimation::Request::SharedPtr request;
            rclcpp::Client<WindEstimation>::SharedPtr client;
        };
#ifdef USE_GADEN
        struct GroundTruthWind
        {
            gaden_msgs::srv::WindPosition::Request::SharedPtr request;
            rclcpp::Client<gaden_msgs::srv::WindPosition>::SharedPtr client;
        };
#endif

        struct PublishersAndSubscribers
        {
            std::shared_ptr<rclcpp::Clock> clock;
            GMRFWind gmrfWind;
#ifdef USE_GADEN
            GroundTruthWind groundTruthWind;
#endif

            struct Markers
            {
                rclcpp::Publisher<Marker>::SharedPtr sourceProbabilityMarkers;
                rclcpp::Publisher<Marker>::SharedPtr hitProbabilityMarkers;
                rclcpp::Publisher<MarkerArray>::SharedPtr quadtreePublisher;
                rclcpp::Publisher<MarkerArray>::SharedPtr windArrowMarkers;
                rclcpp::Publisher<Marker>::SharedPtr confidenceMarkers;
            };
            Markers markers;

            PublishersAndSubscribers(std::shared_ptr<rclcpp::Clock> _clock) : clock(_clock) {}
        };
    } // namespace PMFS_internal
} // namespace GSL