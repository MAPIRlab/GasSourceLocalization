#include "GraphGSL.hpp"
#include "NACCompare.hpp"
#include "gsl_server/algorithms/Common/States/ManualNavigation.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"
#include "gsl_server/algorithms/Common/Utils/Pointers.hpp"
#include "gsl_server/algorithms/Common/Utils/ThreadPool.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
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

    void GraphGSL::EvaluateRoomProbabilities()
    {
        simulationSystem.Reset();
        ThreadPool pool;
        {
            ScopedStopwatch watch("evaluation");
            // simulate all possible room sources
            std::vector<std::shared_ptr<DoorwayNode>> simsToRun;
            for (const auto& node : graph.nodes)
            {
                auto roomNode = As<RoomNode>(node);
                if (!roomNode)
                    continue;

                for (auto doorway : roomNode->doorways)
                    pool.QueueJob([&, doorway]()
                                  {
                                      simulationSystem.SimulateEntireGraph(doorway);
                                  });
            }
            pool.Wait();
        }
        std::map<std::shared_ptr<PlaceNode>, float> roomResiduals;

        // get the measured concentration maps
        std::map<std::shared_ptr<RoomNode>, Grid2D<KernelDMVW::KernelCell>> measuredMaps;
        for (auto& node : graph.nodes)
        {
            auto room = As<RoomNode>(node);
            if (!room)
                continue;
            measuredMaps.insert({room, room->GetGasMap()});
        }

        std::mutex mtx;
        auto loop_body = [&](const auto& sourceNode)
        {
            std::vector<std::vector<float>> simulated;

            // while we fill a row of the matrix for each simulation out of this node, we only want one copy of the measurements and their uncertainty
            // if we are done recording those, skip them on the next iteration
            bool fillMeasurements = true;
            std::vector<float> measured;
            std::vector<float> uncertainty;
            std::vector<float> confidence;

            for (const Graph_internal::CompleteMap& simulation : simulationSystem.gasMapsWithRoomSource.at(sourceNode))
            {
                simulated.push_back(std::vector<float>());
                for (const auto& [room, localSimMap] : simulation.gasMaps)
                {
                    if (room == sourceNode)
                        continue;

                    Grid2D<KernelDMVW::KernelCell> measuredLocal = room->GetGasMap();
                    for (size_t i = 0; i < localSimMap.size(); i++)
                    {
                        if (!measuredLocal.occupancy.at(i))
                            continue;
                        KernelDMVW::KernelCell& cell = measuredLocal.data.at(i);
                        if (cell.confidence < 0.05)
                            continue;

                        simulated.back().push_back(localSimMap.at(i));
                        if (fillMeasurements)
                        {
                            measured.push_back(cell.meanAndVariance.mean);
                            uncertainty.push_back(1 - measuredLocal.data.at(i).confidence); // TODO uncertainty scale?
                            confidence.push_back(cell.confidence);
                        }
                    }
                }

                fillMeasurements = false;
            }

            std::vector<float> weights = NAC::LeastSquaresDoorwayCombination(measured, simulated, uncertainty);
            mtx.lock();
            roomResiduals[sourceNode] = NAC::ResidualDoorways(measured, simulated, confidence, weights);
            mtx.unlock();
        };

        for (const auto& node : graph.nodes)
            pool.QueueJob(std::bind(loop_body, node));
        pool.Wait();

        // calculate the probabilities from the optimization residuals
        //----------------------------------------------------
        roomSourceProbabilities.clear();
        GSL_INFO("Results with sigma={:.2f}", likelihoodSigma);
        std::map<std::shared_ptr<PlaceNode>, double> scores;
        double scoresSum = 0;
        for (auto& [room, loss] : roomResiduals)
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
            roomSourceProbabilities[room] = prob;
            GSL_INFO("\tp({}) = {:.2f}", room->id, prob);
        }

        std::vector<std::shared_ptr<RoomNode>> roomNodes;
        for (const auto& [node, prob] : roomSourceProbabilities)
        {
            auto roomNode = As<RoomNode>(node);
            if (roomNode && prob > 0.3)
                roomNodes.push_back(roomNode);
        }

        EvaluateSourceProbabilitiesInRooms(roomNodes);
    }

    void GraphGSL::EvaluateSourceProbabilitiesInRooms(std::vector<std::shared_ptr<RoomNode>> roomNodes)
    {
        FrameMarkStart("a");
        ZoneScopedN("Room level");
        ScopedStopwatch watch("Evaluation (source probabilities in room)");
        ThreadPool pool;
        std::mutex mtx;

        // start by using all the leaves in all the rooms of interest
        std::deque<std::pair<std::shared_ptr<RoomNode>, NQA::Node>> queue;
        for (const auto& roomNode : roomNodes)
            for (const auto& nqaNode : roomNode->GetQuadtreeLeaves())
                queue.push_back({roomNode, nqaNode});

        size_t numSimulations = 0;
        do
        {
            struct Result
            {
                std::shared_ptr<RoomNode> roomNode;
                NQA::Node node;
                Graph_internal::CompleteMap* map;
            };
            // run the queued up simulations and register the results
            std::vector<Result> results;
            while (!queue.empty())
            {
                auto [roomNode, nqaNode] = queue.front();
                queue.pop_front();
                pool.QueueJob([&, this, roomNode, nqaNode]()
                              {
                                  AABB2D aabb = roomNode->GetOccupancy().metadata.indicesToCoordinates(nqaNode.getAABB());
                                  Graph_internal::CompleteMap& result = simulationSystem.SimulateEntireGraph(roomNode, aabb.center());
                                  mtx.lock();
                                  numSimulations++;
                                  results.push_back({roomNode, nqaNode, &result});
                                  mtx.unlock();
                              });
            }
            pool.Wait();

            // calculate the residuals from the simulation results and sort accordingly
            std::vector<std::pair<Result, float>> residuals;
            for (const auto& result : results)
            {
                float residual = ResidualSingleSimulation(*result.map);
                residuals.push_back({result, residual});
            }
            std::sort(residuals.begin(), residuals.end(), [](const auto& a, const auto& b)
                      {
                          return a.second < b.second;
                      });

            constexpr float proportionBest = 0.15;
            // subdivide the nodes with the best residuals and add the smaller bits to the queue
            for (size_t i = 0; i < residuals.size() * proportionBest; i++)
            {
                auto [result, residual] = residuals.at(i);
                result.node.ForceSubdivide();
                for (const auto& child : result.node.children)
                    if (child)
                        queue.push_back({result.roomNode, *child});
            }
        } while (!queue.empty());
        GSL_INFO("Ran {} simulations at the geometric level", numSimulations);
        FrameMarkStart("b");
    }

    float GraphGSL::ResidualSingleSimulation(const Graph_internal::CompleteMap& simMap)
    {
        ZoneScopedN("Residual calculation");
        std::vector<float> measured;
        std::vector<float> simulated;
        std::vector<float> uncertainty;
        for (const auto& [room, localSimMap] : simMap.gasMaps)
        {
            // add any relevant cells to the comparison arrays
            // we skip the cells with very low measurement confidence for optimization, since they shouldn't really affect the result anyways
            Grid2D<KernelDMVW::KernelCell> measuredLocal = room->GetGasMap();
            for (size_t i = 0; i < measuredLocal.data.size(); i++)
            {
                if (!measuredLocal.occupancy.at(i))
                    continue;
                KernelDMVW::KernelCell& cell = measuredLocal.data.at(i);
                if (cell.confidence < 0.05)
                    continue;

                measured.push_back(cell.meanAndVariance.mean);
                simulated.push_back(localSimMap.at(i));
                uncertainty.push_back(1 - measuredLocal.data.at(i).confidence); // TODO uncertainty scale?
            }
        }

        float scale = NAC::LeastSquaresScale(measured, simulated, uncertainty);
        float residual = NAC::Residual(measured, simulated, uncertainty, scale);
        return residual;
    }

#if ENABLE_NAIVE_EVALUATION
    void GraphGSL::EvaluateRoomProbabilitiesNaive()
    {
        naiveEntireMap->UpdateWindMap(graph.gmrf);
        naiveCompleteMaps.clear();

        // run the simulations
        {
            ScopedStopwatch watch("Evaluation (naive)");
            ThreadPool pool;
            for (const auto& node : graph.nodes)
            {
                auto roomNode = As<RoomNode>(node);
                for (Vector2 point : node->RepresentativePoints())
                    if (naiveEntireMap->IsValidPoint(point))
                    {
                        auto job = [point, this]()
                        {
                            naiveSimulationSystem.SimulateSourceFromPoint(naiveEntireMap, point);
                        };
                        pool.QueueJob(job);
                    }
            }
            pool.Wait();
        }

        // evaluate the results
        std::map<std::shared_ptr<Graph_internal::Source>, float> resultLoss;
        for (auto& simCompleteMap : naiveCompleteMaps)
        {
            float loss = ResidualSingleSimulation(simCompleteMap);
            resultLoss[simCompleteMap.source] = loss;
        }

        // calculate the probabilities from the loss evaluation
        std::map<std::shared_ptr<Graph_internal::Source>, float> scores;
        constexpr float sigma = 100;
        double scoresSum = 0;
        for (auto& [source, loss] : resultLoss)
            if (!std::isnan(loss))
            {
                scores[source] = std::exp(-loss / likelihoodSigma);
                scoresSum += scores[source];
            }
            else
                scores[source] = 0;

        for (const auto& [source, score] : scores)
        {
            double prob = score / scoresSum;
            GSL_INFO("\tp({}) = {:.2f}", source->GetPoint(), prob);
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