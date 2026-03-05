#include "GraphGSL.hpp"
#include "gsl_server/algorithms/Common/States/ManualNavigation.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>

namespace GSL
{

    GraphGSL::GraphGSL(std::shared_ptr<rclcpp::Node> _node)
        : Algorithm(_node),
          gui(this)
    {}

    void GraphGSL::Initialize()
    {
        Algorithm::Initialize();

        float cellSize = 0.25; // TODO scale
        gmrfParams.cell_size = cellSize;
        std::filesystem::path path = std::filesystem::path(ament_index_cpp::get_package_share_directory("graphgsl_env")) / "second_graph";
        graph = Graph::ReadFromDisk(path, cellSize, gmrfParams);
        IF_GUI(gui.Run());

        waitForGasState = std::make_unique<WaitForGasState>(this);
        waitForMapState = std::make_unique<WaitForMapState>(this);
        waitForMapState->shouldWaitForGas = false;

        stopAndMeasureState = std::make_unique<StopAndMeasureState>(this);
        movingState = std::make_unique<ManualNavigationState>(this);
        stateMachine.forceSetState(movingState.get());
    }

    void GraphGSL::OnUpdate()
    {
        Algorithm::OnUpdate();
    }

    void GraphGSL::processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection)
    {
        // graph.AddObservation(currentRobotPosition, Utils::polarToCartesian(windSpeed, windDirection), concentration);
        stateMachine.forceResetState(stopAndMeasureState.get());
    }

    //TODO we probably don't want to override this at all! this is here for testing purposes
    Vector2 GraphGSL::windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg)
    {
        Vector2 wind = Algorithm::windCallback(msg);
        graph.AddObservation(currentRobotPosition, wind, 0);
        return wind;
    }

} // namespace GSL