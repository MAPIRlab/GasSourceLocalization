#include "GraphGSL.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

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

        graph = Graph::ReadFromDisk(std::filesystem::path(ament_index_cpp::get_package_share_directory("graphgsl_env")) / "test_graph");
        IF_GUI(gui.Run());
    }

    void GraphGSL::OnUpdate()
    {
        // Algorithm::OnUpdate();
    }

    void GraphGSL::processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection)
    {
    }

} // namespace GSL