#pragma once
#include "Graph.hpp"
#include "GraphUI.hpp"
#include <gsl_server/algorithms/Common/Algorithm.hpp>

namespace GSL
{
    class GraphGSL : public Algorithm
    {
    public:
        GraphGSL(std::shared_ptr<rclcpp::Node> _node);
        void processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection); // called from StopAndMeasure once we have enough data for this position
    private:
        Graph graph;

#if USE_GUI
        friend class GraphUI;
        GraphUI gui;
#endif
    };
} // namespace GSL