#include "GraphGSL.hpp"

namespace GSL
{

    GraphGSL::GraphGSL(std::shared_ptr<rclcpp::Node> _node)
        : Algorithm(_node),
          gui(this)
    {}

    void GraphGSL::Initialize()
    {
        // Algorithm::Initialize();
        startTime = node->now();
        declareParameters();

        graph = Graph::ReadFromDisk("/home/pepe/Desktop/test_graph");
        IF_GUI(gui.run());
    }

    void GraphGSL::OnUpdate()
    {
        // Algorithm::OnUpdate();
    }

    void GraphGSL::processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection)
    {
    }

} // namespace GSL