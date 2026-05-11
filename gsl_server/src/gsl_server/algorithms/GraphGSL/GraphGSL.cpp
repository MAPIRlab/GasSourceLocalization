#include "GraphGSL.hpp"
#include "gsl_server/algorithms/Common/States/ManualNavigation.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"
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
        gmrfParams.lambdaPrior_advection = node->declare_parameter<float>("GMRF_lambdaPrior_advection");
        gmrfParams.lambdaPrior_diffusion = node->declare_parameter<float>("GMRF_lambdaPrior_diffusion");
        gmrfParams.lambdaPrior_mass_conservation = node->declare_parameter<float>("GMRF_lambdaPrior_mass_conservation");
        gmrfParams.lambdaPrior_obstacles = node->declare_parameter<float>("GMRF_lambdaPrior_obstacles");

        // graph creation
        std::filesystem::path path =
            node->declare_parameter<std::string>("graph_path",
                                                 std::filesystem::path(ament_index_cpp::get_package_share_directory("graphgsl_env")) / "second_graph");
        graph = Graph::ReadFromDisk(path, cellSize, gmrfParams);
        float artificialSeparation = node->declare_parameter<float>("node_separation_mult", 1);
        graph.nodeSeparationViz = artificialSeparation;
        simulationSystem.graph = &graph;

        // GUI
        IF_GUI(gui.Run());

        // publishers
        pubs.graphPub = node->create_publisher<MarkerArray>("/gsl_graph", rclcpp::QoS(1).transient_local());
        pubs.occupancyPub = node->create_publisher<MarkerArray>("/gsl_occupancy", 1);
        pubs.windPub = node->create_publisher<MarkerArray>("/gsl_wind", 1);
        pubs.gasMapsPub = node->create_publisher<MarkerArray>("gasMaps", 1);

        // state machine
        waitForGasState = std::make_unique<WaitForGasState>(this);
        waitForMapState = std::make_unique<WaitForMapState>(this);
        waitForMapState->shouldWaitForGas = false;

        stopAndMeasureState = std::make_unique<StopAndMeasureState>(this);
        movingState = std::make_unique<ManualNavigationState>(this);

        std::string simulatedMeasurementsPath = node->declare_parameter<std::string>("sim_measurements_path", "?");
        if (simulatedMeasurementsPath != "?")
        {
            SimulateMeasurements(simulatedMeasurementsPath);
            graph.UpdateAllWindMaps();
        }

        stateMachine.forceSetState(movingState.get());

        // std::vector<float> simulated = {0, 0.1, 0.2, 0.4, 1.0, 3.0};
        // std::vector<float> observed =    {0, 0.2, 0.4, 0.7, 3.0, 1.0};
        // std::vector<float> uncertainty = {0.5,  0.5,   0.5,   0.5,   0.5,   0.5};
        // float scale = LeastSquaresScale(simulated, observed, uncertainty);
        // GSL_INFO("Best scale: {:.2f}", scale);
        // float evaluation =  LossFunction(simulated, observed, uncertainty, scale);
        // GSL_INFO("Loss evaluation: {:.2f}", evaluation);
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
        graph.AddObservation(currentRobotPosition, Utils::polarToCartesian(windSpeed, windDirection), concentration);
        // graph.UpdateAllWindMaps();
        // stateMachine.forceSetState(movingState.get());
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
        pubs.gasMapsPub->publish(simulationSystem.VisualizeCachedResults(nodeSelectedForVisualization, graph.nodeSeparationViz));
    }

} // namespace GSL