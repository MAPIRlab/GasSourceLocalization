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

        struct GMRFWind
        {
            WindEstimation::Request::SharedPtr request;
            rclcpp::Client<WindEstimation>::SharedPtr client;
        };
#ifdef USE_GADEN
        struct GroundTruthWind
        {
            gaden_player::srv::WindPosition::Request::SharedPtr request;
            rclcpp::Client<gaden_player::srv::WindPosition>::SharedPtr client;
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
                rclcpp::Publisher<Marker>::SharedPtr source_probability_markers;
                rclcpp::Publisher<Marker>::SharedPtr hitProbabilityMarkers;
                rclcpp::Publisher<MarkerArray>::SharedPtr quadtreePublisher;
                rclcpp::Publisher<MarkerArray>::SharedPtr windArrowMarkers;
                rclcpp::Publisher<Marker>::SharedPtr confidenceMarkers;
            };
            Markers markers;

            PublishersAndSubscribers(std::shared_ptr<rclcpp::Clock> _clock) : clock(_clock){} 
        };
    } // namespace PMFS_internal
} // namespace GSL