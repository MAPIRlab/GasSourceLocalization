#include "GraphGSL.hpp"

namespace GSL
{

    GraphGSL::GraphGSL(std::shared_ptr<rclcpp::Node> _node)
        : Algorithm(_node),
          gui(this)
    {
        graph = Graph::ReadFromDisk("/home/pepe/Desktop/test_graph");
    }

    void GraphGSL::processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection)
    {
    }

} // namespace GSL