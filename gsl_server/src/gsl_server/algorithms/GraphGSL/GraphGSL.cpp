#include "GraphGSL.hpp"
#include "NACCompare.hpp"
#include "gsl_server/algorithms/Common/States/ManualNavigation.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"
#include "gsl_server/algorithms/Common/Utils/Pointers.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <execution>
#include <fmt/ranges.h>
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>

namespace GSL
{

    GraphGSL::GraphGSL(std::shared_ptr<rclcpp::Node> _node)
        : Algorithm(_node),
          gui(this),
          visualizationCD(0.1),
          naiveSimulationSystem(simulationSystem.options)
    {}

    void GraphGSL::Initialize()
    {
        Algorithm::Initialize();

        float cellSize = rclnode->declare_parameter<float>("cell_size", 0.15);

        // GMRF
        gmrfParams.cell_size = cellSize;
        gmrfParams.lambdaPrior_advection = rclnode->declare_parameter<float>("GMRF_lambdaPrior_advection");
        gmrfParams.lambdaPrior_diffusion = rclnode->declare_parameter<float>("GMRF_lambdaPrior_diffusion");
        gmrfParams.lambdaPrior_mass_conservation = rclnode->declare_parameter<float>("GMRF_lambdaPrior_mass_conservation");
        gmrfParams.lambdaPrior_obstacles = rclnode->declare_parameter<float>("GMRF_lambdaPrior_obstacles");

        // graph creation
        std::filesystem::path path =
            rclnode->declare_parameter<std::string>("graph_path",
                                                 std::filesystem::path(ament_index_cpp::get_package_share_directory("graphgsl_env")) / "second_graph");
        graph = Graph::ReadFromDisk(path, cellSize, gmrfParams);
        float artificialSeparation = rclnode->declare_parameter<float>("node_separation_mult", 1);
        graph.nodeSeparationViz = artificialSeparation;
        simulationSystem.graph = &graph;

#if ENABLE_NAIVE_EVALUATION
        naiveEntireMap = std::make_shared<RoomNode>(graph.completeMap.AsGrid());
#endif
        // GUI
        IF_GUI(gui.Run());

        // publishers
        pubs.graphPub = rclnode->create_publisher<MarkerArray>("/gsl_graph", rclcpp::QoS(1).transient_local());
        pubs.occupancyPub = rclnode->create_publisher<MarkerArray>("/gsl_occupancy", 1);
        pubs.windPub = rclnode->create_publisher<MarkerArray>("/gsl_wind", 1);
        pubs.simGasMapsPub = rclnode->create_publisher<MarkerArray>("simGasMaps", 1);
        pubs.measuredGasMapsPub = rclnode->create_publisher<MarkerArray>("measuredGasMaps", 1);
        pubs.quadtreePub = rclnode->create_publisher<MarkerArray>("quadtree", 1);
#if ENABLE_NAIVE_EVALUATION
        naiveMapsPub = rclnode->create_publisher<MarkerArray>("/gsl_naive_maps", 1);
#endif
        // state machine
        waitForGasState = std::make_unique<WaitForGasState>(this);
        waitForMapState = std::make_unique<WaitForMapState>(this);
        waitForMapState->shouldWaitForGas = false;

        stopAndMeasureState = std::make_unique<StopAndMeasureState>(this);
        movingState = std::make_unique<ManualNavigationState>(this);

        std::string simulatedMeasurementsPath = rclnode->declare_parameter<std::string>("sim_measurements_path", "?");
        if (simulatedMeasurementsPath != "?")
        {
            SimulateMeasurements(simulatedMeasurementsPath);
            graph.UpdateAllWindMaps();
        }

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
        graph.AddObservation(currentRobotPosition, Utils::polarToCartesian(windSpeed, windDirection), concentration);

#if ENABLE_NAIVE_EVALUATION
        naiveEntireMap->AddObservation(currentRobotPosition, Utils::polarToCartesian(windSpeed, windDirection), concentration);
#endif

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

    void GraphGSL::EvaluateSourceProbabilities()
    {
        simulationSystem.Reset();

        {
            ScopedStopwatch watch("evaluation");
            // simulate all possible room sources
            std::vector<std::pair<std::shared_ptr<PlaceNode>, Vector2>> simsToRun;
            for (const auto& node : graph.nodes)
            {
                auto roomNode = As<RoomNode>(node);
                for (Vector2 point : node->RepresentativePoints())
                    simsToRun.push_back({node, point});
            }

#pragma omp parallel for
            for (const auto& [node, point] : simsToRun)
                simulationSystem.SimulateEntireGraph(node, point);
        }

        // get the measured concentration maps
        std::map<std::shared_ptr<RoomNode>, Grid2D<KernelDMVW::KernelCell>> measuredMaps;
        for (auto& node : graph.nodes)
        {
            auto room = As<RoomNode>(node);
            if (!room)
                continue;
            measuredMaps.insert({room, room->GetGasMap()});
        }

        // clang-format off
        std::for_each(std::execution::par, simulationSystem.gasMapsWithRoomSource.begin(), 
        simulationSystem.gasMapsWithRoomSource.end(), [&](const auto& pair)
        {
            auto& [sourceRoom, completeMaps] = pair;
            for (auto& simCompleteMap : completeMaps)
            {
                // scaling
                std::vector<float> measured;
                std::vector<float> simulated;
                std::vector<float> uncertainty;
                std::vector<float> confidence;
                for (const auto& node : graph.nodes)
                {
                    auto room = As<RoomNode>(node);
                    if (!room)
                        continue;

                    const std::vector<float>& simLocalMap = simCompleteMap.gasMaps.at(room);

                    // add any relevant cells to the comparison arrays
                    // we skip the cells with very low measurement confidence for optimization, since they shouldn't really affect the result anyways
                    Grid2D<KernelDMVW::KernelCell> measuredLocal = measuredMaps.at(room);
                    for (size_t i = 0; i < measuredLocal.data.size(); i++)
                    {
                        if (!measuredLocal.occupancy.at(i))
                            continue;
                        KernelDMVW::KernelCell& cell = measuredLocal.data.at(i);
                        if (cell.confidence < 0.05)
                            continue;

                        constexpr float maxUncertainty = 100.0f;
                        measured.push_back(cell.meanAndVariance.mean);
                        simulated.push_back(simLocalMap.at(i));
                        uncertainty.push_back( maxUncertainty * (1 - measuredLocal.data.at(i).confidence));
                        confidence.push_back(measuredLocal.data.at(i).confidence);
                    }
                }
                float scale = NAC::LeastSquaresScale(measured, simulated, uncertainty);
                float loss = NAC::LossFunction(measured, simulated, confidence, scale);
                GSL_INFO("{}: scale {:.4f}  -- Loss {:.4f}", sourceRoom->id, scale, loss);
                if (resultLoss.contains(sourceRoom))
                    resultLoss.at(sourceRoom) = std::min(resultLoss.at(sourceRoom), loss);
                else
                    resultLoss[sourceRoom] = loss;
            }
        });
        // clang-format on
    }

    void GraphGSL::CalculateProbs()
    {
        GSL_INFO("Results with sigma={:.2f}", likelihoodSigma);
        // calculate the probabilities from the loss evaluation
        std::map<std::shared_ptr<PlaceNode>, double> scores;
        double scoresSum = 0;
        for (auto& [room, loss] : resultLoss)
            if (!std::isnan(loss))
            {
                scores[room] = std::exp(-loss / likelihoodSigma);
                scoresSum += scores[room];
            }
            else
                scores[room] = 0;

        for (const auto& [room, score] : scores)
        {
            double prob = score / scoresSum;

            GSL_INFO("\tp({}) = {:.2f}", room->id, prob);
        }
    }

#if ENABLE_NAIVE_EVALUATION
    void GraphGSL::EvaluateSourceProbabilitiesNaive()
    {
        naiveEntireMap->UpdateWindMap(graph.gmrf);
        naiveCompleteMaps.clear();

        {
            ScopedStopwatch watch("Evaluation (naive)");
            // simulate all possible room sources
            std::vector<Vector2> simsToRun;
            for (const auto& node : graph.nodes)
            {
                auto roomNode = As<RoomNode>(node);
                for (Vector2 point : node->RepresentativePoints())
                    if (naiveEntireMap->IsValidPoint(point))
                        simsToRun.push_back(point);
            }

#pragma omp parallel for
            for (const auto& sourcePoint : simsToRun)
            {
                Graph_internal::SimWithResult result = naiveSimulationSystem.SimulateSourceFromPoint(naiveEntireMap, sourcePoint);
#pragma omp critical
                {
                    naiveCompleteMaps.push_back(naiveSimulationSystem.AsCompleteMap(naiveEntireMap, result));
                }
            }
        }

        std::map<Vector2*, float> resultLoss;

        for (auto& simCompleteMap : naiveCompleteMaps)
        {
            // scaling
            std::vector<float> measured;
            std::vector<float> simulated;
            std::vector<float> uncertainty;

            // add any relevant cells to the comparison arrays
            // we skip the cells with very low measurement confidence for optimization, since they shouldn't really affect the result anyways
            Grid2D<KernelDMVW::KernelCell> measuredLocal = naiveEntireMap->GetGasMap();
            for (size_t i = 0; i < measuredLocal.data.size(); i++)
            {
                if (!measuredLocal.occupancy.at(i))
                    continue;
                KernelDMVW::KernelCell& cell = measuredLocal.data.at(i);
                if (cell.confidence < 0.05)
                    continue;

                measured.push_back(cell.meanAndVariance.mean);
                simulated.push_back(simCompleteMap.gasMaps.begin()->second.at(i));
                uncertainty.push_back(1 - measuredLocal.data.at(i).confidence); // TODO uncertainty scale?
            }
            float scale = NAC::LeastSquaresScale(measured, simulated, uncertainty);
            float loss = NAC::LossFunction(measured, simulated, uncertainty, scale);
            resultLoss[&simCompleteMap.sourcePoint] = loss;
        }

        // calculate the probabilities from the loss evaluation
        std::map<Vector2*, float> scores;
        constexpr float sigma = 100;
        float scoresSum = 0;
        for (auto& [sourcePoint, loss] : resultLoss)
            if (!std::isnan(loss))
            {
                scores[sourcePoint] = std::exp(-loss / sigma);
                scoresSum += scores[sourcePoint];
            }
            else
                scores[sourcePoint] = 0;

        for (const auto& [sourcePoint, score] : scores)
        {
            float prob = score / scoresSum;

            GSL_INFO("\tp({}) = {:.2f}", *sourcePoint, prob);
        }
    }
#endif

    void GraphGSL::Visualize()
    {
        if (drawGraph)
            pubs.graphPub->publish(graph.VisualizeGraph());
        else
            Utils::ClearMarkers(pubs.graphPub);

        pubs.occupancyPub->publish(graph.VisualizeOccupancy());
        pubs.windPub->publish(graph.VisualizeWind());
        pubs.measuredGasMapsPub->publish(graph.VisualizeGasReadings());
        pubs.simGasMapsPub->publish(simulationSystem.VisualizeCachedResults(simulationViz.selectedNode, simulationViz.simulationIndex, graph.nodeSeparationViz));
#if ENABLE_NAIVE_EVALUATION
        if (naiveSimulationIndex < naiveCompleteMaps.size())
            naiveMapsPub->publish(Graph_internal::VisualizeCompleteMap(naiveCompleteMaps.at(naiveSimulationIndex), {naiveEntireMap}, graph.nodeSeparationViz, 0.2));
#endif
        pubs.quadtreePub->publish(graph.VisualizeMapSegmentation());
    }

} // namespace GSL