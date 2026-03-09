#include "GraphGSL.hpp"
#include "gsl_server/algorithms/Common/States/ManualNavigation.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>

namespace GSL
{

    GraphGSL::GraphGSL(std::shared_ptr<rclcpp::Node> _node)
        : Algorithm(_node),
          gui(this),
          visualizationCD(0.1)
    {}

    void GraphGSL::Initialize()
    {
        Algorithm::Initialize();

        float cellSize = node->declare_parameter<float>("cell_size", 0.15);

        // GMRF
        gmrfParams.cell_size = cellSize;
        gmrfParams.m_lambdaPrior_flux_conservation = node->declare_parameter<float>("GMRF_lambda_flux");
        gmrfParams.m_lambdaPrior_obstacles = node->declare_parameter<float>("GMRF_lambda_obstacles");
        gmrfParams.m_lambdaPrior_reg = node->declare_parameter<float>("GMRF_lambda_reg");

        // graph creation
        std::filesystem::path path =
            node->declare_parameter<std::string>("graph_path",
                                                 std::filesystem::path(ament_index_cpp::get_package_share_directory("graphgsl_env")) / "second_graph");
        float artificialSeparation = node->declare_parameter<float>("node_separation_mult", 1);
        graph = Graph::ReadFromDisk(path, cellSize, artificialSeparation, gmrfParams);

        // GUI
        IF_GUI(gui.Run());

        // publishers
        pubs.graphPub = node->create_publisher<MarkerArray>("gsl_graph", 1);
        pubs.occupancyPub = node->create_publisher<MarkerArray>("gsl_occupancy", 1);
        pubs.windPub = node->create_publisher<MarkerArray>("gsl_wind", 1);

        // state machine
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

        if (visualizationCD.isDone())
        {
            Visualize();
            visualizationCD.Restart();
        }
    }

    void GraphGSL::processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection)
    {
        // graph.AddObservation(currentRobotPosition, Utils::polarToCartesian(windSpeed, windDirection), concentration);
        stateMachine.forceSetState(movingState.get());
    }

    // TODO we probably don't want to override this at all! this is here for testing purposes
    Vector2 GraphGSL::windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg)
    {
        Vector2 wind = Algorithm::windCallback(msg);
        graph.AddObservation(currentRobotPosition, wind, 0);
        graph.UpdateAllWindMaps(); // TODO remove this! it's a test
        return wind;
    }

    void GraphGSL::Visualize()
    {
        if (drawGraph)
            pubs.graphPub->publish(graph.VisualizeGraph());
        else
            Utils::ClearMarkers(pubs.graphPub);

        pubs.occupancyPub->publish(graph.VisualizeOccupancy());
        pubs.windPub->publish(graph.VisualizeWind());
    }

} // namespace GSL