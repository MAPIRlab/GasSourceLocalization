#include "GraphGSL.hpp"
#include "NACCompare.hpp"
#include "gsl_server/algorithms/Common/States/ManualNavigation.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"
#include "gsl_server/algorithms/Common/Utils/Pointers.hpp"
#include "gsl_server/algorithms/Common/Utils/ThreadPool.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fmt/ranges.h>
#include <gsl_server/algorithms/Common/Utils/Collections.hpp>
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
        pubs.sourceProbPub = rclnode->create_publisher<MarkerArray>("sourceProb", 1);
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
        Algorithm::processGasAndWindMeasurements(concentration, windSpeed, windDirection);
        graph.AddObservation(currentRobotPosition, Utils::polarToCartesian(windSpeed, windDirection), concentration);

#if ENABLE_NAIVE_EVALUATION
        naiveEntireMap->AddObservation(currentRobotPosition, Utils::polarToCartesian(windSpeed, windDirection), concentration);
#endif

        stateMachine.forceSetState(movingState.get());
    }

    // // TODO we probably don't want to override this at all! this is here for testing purposes
    // Vector2 GraphGSL::windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg)
    // {
    //     Vector2 wind = Algorithm::windCallback(msg);
    //     graph.AddObservation(currentRobotPosition, wind, 0);
    //     graph.UpdateAllWindMaps(); // TODO remove this! it's a test
    //     return wind;
    // }

    void GraphGSL::EvaluateRoomProbabilities()
    {
        graph.UpdateAllWindMaps();
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
        auto loop_body = [&](const auto sourceNode)
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
        graph.roomSourceProbabilities.clear();
        GSL_INFO("Results with sigma={:.2f}", likelihoodSigma);
        std::map<std::shared_ptr<PlaceNode>, double> scores;
        double scoresSum = 0;
        for (auto& [room, residual] : roomResiduals)
            if (!std::isnan(residual))
            {
                scores[room] = ProbFromResidual(residual);
                scoresSum += scores[room];
            }
            else
                scores[room] = 0;

        for (const auto& [room, score] : scores)
        {
            double prob = score / scoresSum;
            graph.roomSourceProbabilities[room] = prob;
            GSL_INFO("\tp({}) = {:.2f}", room->id, prob);
        }

        std::vector<std::shared_ptr<RoomNode>> roomNodes;
        for (const auto& [node, prob] : graph.roomSourceProbabilities)
        {
            auto roomNode = As<RoomNode>(node);
            if (roomNode && prob > 0.3)
                roomNodes.push_back(roomNode);
        }

        EvaluateSourceProbabilitiesInRooms(roomNodes);
    }

    void GraphGSL::EvaluateSourceProbabilitiesInRooms(std::vector<std::shared_ptr<RoomNode>> roomNodes)
    {
        ZoneScopedN("Room level");
        ScopedStopwatch watch("Evaluation (source probabilities in room)");
        ThreadPool pool;
        std::mutex mtx;

        struct Region
        {
            std::shared_ptr<RoomNode> room;
            NQA::Node nqaNode;
        };
        std::map<std::shared_ptr<Region>, float> finalResiduals;

        // start by using all the leaves in all the rooms of interest
        std::deque<Region> queue;
        for (const auto& roomNode : roomNodes)
            for (const auto& nqaNode : roomNode->GetQuadtreeLeaves())
                queue.push_back({roomNode, nqaNode});

        size_t numSimulations = 0;
        do
        {
            struct Result
            {
                std::shared_ptr<Region> region;
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

                                  std::scoped_lock lock(mtx);
                                  numSimulations++;
                                  results.push_back({std::make_shared<Region>(roomNode, nqaNode), &result});
                              });
            }
            pool.Wait();

            // calculate the residuals from the simulation results and sort accordingly
            std::vector<std::pair<Result, float>> residualsThisLevel;
            for (const auto& result : results)
            {
                float residual = ResidualSingleSimulation(*result.map);
                GSL_ASSERT(std::isfinite(residual));
                residualsThisLevel.push_back({result, residual});
            }
            std::sort(residualsThisLevel.begin(), residualsThisLevel.end(), [](const auto& a, const auto& b)
                      {
                          return a.second < b.second;
                      });

            constexpr float proportionBest = 0.15;
            // subdivide the nodes with the best residuals and add the smaller bits to the queue
            for (size_t i = 0; i < residualsThisLevel.size(); i++)
            {
                auto [result, residual] = residualsThisLevel.at(i);
                if (i < residualsThisLevel.size() * proportionBest)
                {
                    bool subdivided = result.region->nqaNode.ForceSubdivide();
                    if (subdivided)
                    {
                        for (const auto& child : result.region->nqaNode.children)
                            if (child)
                                queue.push_back({result.region->room, *child});
                    }
                    else
                        finalResiduals[result.region] = residual;
                }
                else
                    finalResiduals[result.region] = residual;
            }
            GSL_INFO("Completed a simulation level -- total simulations: {}", numSimulations);
        } while (!queue.empty());

        // turn the residuals into probabilities
        for (const auto& [region, residual] : finalResiduals)
            for (Vector2Int pos : region->nqaNode.getAABB())
            {
                float prob = ProbFromResidual(residual);
                GSL_ASSERT(std::isfinite(prob));
                region->room->GetSourceProbabilities().dataAt(pos) = prob;
            }

        // normalize the conditional probabilities -- p(s | room)
        for (auto node : graph.nodes)
        {
            auto room = As<RoomNode>(node);
            if (!room)
                continue;

            Grid2D sourceProbs = room->GetSourceProbabilities();

            // if we did simulations in this room, use the probabilities we just calculated from the residuals
            if (Utils::contains(roomNodes, room))
                Utils::PowerMaxNormalize(sourceProbs.data, sourceProbs.occupancy);
            // otherwise, set all the cells in the room to the same probability (old probs might not be reliable anymore)
            else
            {
                for (size_t i = 0; i < sourceProbs.data.size(); i++)
                    if (sourceProbs.occupancy.at(i))
                        sourceProbs.data.at(i) = 1.f / sourceProbs.metadata.numFreeCells;
            }
        }

        GSL_INFO("Ran {} simulations at the geometric level", numSimulations);
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

    float GraphGSL::ProbFromResidual(float residual)
    {
        return std::exp(-residual / likelihoodSigma);
    }

#if ENABLE_NAIVE_EVALUATION
    void GraphGSL::EvaluateRoomProbabilitiesNaive()
    {
        naiveEntireMap->UpdateWindMap(graph.gmrf);
        // TODO store the results for visualization?
        EvaluateSourceProbabilitiesInRooms({naiveEntireMap});
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
        pubs.sourceProbPub->publish(graph.VisualizeSourceProbs());
    }

} // namespace GSL