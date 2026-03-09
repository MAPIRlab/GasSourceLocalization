#pragma once
#include "Graph.hpp"
#include "GraphUI.hpp"
#include "gsl_server/algorithms/Common/Utils/Time.hpp"
#include <gsl_server/algorithms/Common/Algorithm.hpp>

namespace GSL
{
    class GraphGSL : public Algorithm
    {
    public:
        GraphGSL(std::shared_ptr<rclcpp::Node> _node);
        void Initialize() override;
        void OnUpdate() override;

        void processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection) override; // called from StopAndMeasure once we have enough data for this position
        Vector2 windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg) override;
        void Visualize();

    private:
        Graph graph;
        gmrfw::CGMRF_map::Parameters gmrfParams;

        struct Pubs
        {
            rclcpp::Publisher<MarkerArray>::SharedPtr graphPub;
            rclcpp::Publisher<MarkerArray>::SharedPtr occupancyPub;
            rclcpp::Publisher<MarkerArray>::SharedPtr windPub;
        } pubs;

        Utils::Time::Countdown visualizationCD;
        bool drawGraph = true;

#if USE_GUI
        friend class GraphUI;
        GraphUI gui;
#endif
    };
} // namespace GSL