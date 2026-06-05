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
        graph.vizOptions.nodeSeparationViz = artificialSeparation;
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
            UpdateWindMaps();
        }

        UpdateExpectedValue();
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

    void GraphGSL::UpdateWindMaps()
    {
        graph.UpdateAllWindMaps();
        naiveEntireMap->UpdateWindMap(graph.gmrf);
    }

    // // TODO we probably don't want to override this at all! this is here for testing purposes
    // Vector2 GraphGSL::windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg)
    // {
    //     Vector2 wind = Algorithm::windCallback(msg);
    //     graph.AddObservation(currentRobotPosition, wind, 0);
    //     UpdateWindMaps(); // TODO remove this! it's a test
    //     return wind;
    // }

    void GraphGSL::EvaluateRoomProbabilities()
    {
        UpdateWindMaps();
        simulationSystem.Reset();
        ThreadPool pool;
        {
            ScopedStopwatch watch("Room simulations");
            // simulate all possible room sources
            std::vector<std::shared_ptr<DoorwayNode>> simsToRun;
            for (const auto& node : graph.nodes)
            {
                auto roomNode = As<RoomNode>(node);
                if (!roomNode)
                    continue;

                for (auto doorway : roomNode->doorways)
                    // pool.QueueJob([&, doorway]()
                                  {
                                      simulationSystem.SimulateEntireGraph(doorway);
                                  }//);
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
            // to make the layout convenient for the optimization, each row corresponds to a cell in the map, and each column to a simulation
            std::vector<std::vector<float>> simulated;
            simulated.reserve(300);

            // while we fill a row of the matrix for each simulation out of this node, we only want one copy of the measurements and their uncertainty
            // if we are done recording those, skip them on the next iteration
            std::vector<float> measured;
            std::vector<float> uncertainty;

            size_t simIndex = 0;
            for (const Graph_internal::CompleteMap& simulation : simulationSystem.gasMapsWithRoomSource.at(sourceNode))
            {
                size_t cellIdx = 0; // count the valid cells
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

                        if (simIndex == 0)
                        {
                            simulated.push_back(std::vector<float>());
                            measured.push_back(cell.meanAndVariance.mean);
                            uncertainty.push_back(1 - measuredLocal.data.at(i).confidence);
                        }
                        simulated.at(cellIdx).push_back(localSimMap.at(i));

                        cellIdx++;
                    }
                }
                simIndex++;
            }

            for (size_t i = 0; i < simulated.size(); i++)
                GSL_ASSERT(simulated.at(i).size() == simulated.at(0).size());

            float confidenceSum = 0;
            for (const auto& u : uncertainty)
                confidenceSum += 1 - u;

            mtx.lock();
            roomResiduals[sourceNode] = GSL::NACCeres::FitDoorwayScales(simulated, measured, uncertainty) / confidenceSum;
            GSL_INFO("Residual at {}: {}", sourceNode->id, roomResiduals[sourceNode]);
            mtx.unlock();
        };

        {
            ScopedStopwatch watch("Room residuals");

            for (const auto& node : graph.nodes)
                pool.QueueJob(std::bind(loop_body, node));
            pool.Wait();
        }

        // calculate the probabilities from the optimization residuals
        //----------------------------------------------------
        graph.roomSourceProbabilities.clear();
        GSL_INFO("Results with sigma={:.2f}", likelihoodSigma);
        std::map<std::shared_ptr<PlaceNode>, long double> scores;
        long double scoresSum = 0;
        for (auto& [room, residual] : roomResiduals)
        {
            scores[room] = ProbFromResidual(residual);
            scoresSum += scores[room];
        }

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

        pubs.graphPub->publish(graph.VisualizeGraph());

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
            std::vector<std::pair<Result, float>> residualsThisLevel;
            while (!queue.empty())
            {
                Region region = queue.front();
                auto roomNode = region.room;
                auto nqaNode = region.nqaNode;
                queue.pop_front();
                pool.QueueJob([&, this, roomNode, nqaNode]()
                              {
                                  AABB2D aabb = roomNode->GetOccupancy().metadata.indicesToCoordinates(nqaNode.getAABB());
                                  Graph_internal::CompleteMap& map = simulationSystem.SimulateEntireGraph(roomNode, aabb.center());
                                  Result result{std::make_shared<Region>(roomNode, nqaNode), &map};
                                  float residual = ResidualSingleSimulation(*result.map);
                                  GSL_ASSERT(std::isfinite(residual));

                                  std::scoped_lock lock(mtx);
                                  numSimulations++;
                                  residualsThisLevel.push_back({result, residual});
                              });
            }
            pool.Wait();

            // sort the results by the residuals
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

        std::map<std::shared_ptr<RoomNode>, std::vector<long double>> sourceProbsSimulatedRooms;
        // turn the residuals into probabilities
        for (const auto& [region, residual] : finalResiduals)
        {
            if (!sourceProbsSimulatedRooms.contains(region->room))
                sourceProbsSimulatedRooms[region->room] = std::vector<long double>(region->room->GetSourceProbabilities().data.size(), 0.0);

            long double prob = ProbFromResidual(residual);
            GSL_INFO("Residual {:.3f} -> Prob {:.3f}", residual, prob);
            GSL_ASSERT(std::isfinite(prob));

            for (Vector2Int pos : region->nqaNode.getAABB())
            {
                size_t idx = region->room->GetSourceProbabilities().metadata.indexOf(pos);
                sourceProbsSimulatedRooms.at(region->room).at(idx) = prob;
            }
        }

        // normalize the conditional probabilities -- p(s | room)
        for (auto node : graph.nodes)
        {
            auto room = As<RoomNode>(node);
            if (!room)
                continue;

            // if we did simulations in this room, use the probabilities we just calculated from the residuals
            if (Utils::contains(roomNodes, room))
            {
                Utils::NormalizeDistribution(sourceProbsSimulatedRooms.at(room), room->GetSourceProbabilities().occupancy);
                for (size_t i = 0; i < sourceProbsSimulatedRooms.at(room).size(); i++)
                    room->GetSourceProbabilities().data.at(i) = graph.roomSourceProbabilities.at(room) * sourceProbsSimulatedRooms.at(room).at(i);
            }
            // otherwise, set all the cells in the room to the same probability (old probs might not be reliable anymore)
            else
            {
                Grid2D<float> sourceProbs = room->GetSourceProbabilities();
                for (size_t i = 0; i < sourceProbs.data.size(); i++)
                    if (sourceProbs.occupancy.at(i))
                        sourceProbs.data.at(i) = graph.roomSourceProbabilities.at(room) / sourceProbs.metadata.numFreeCells;
            }
        }

        GSL_INFO("Ran {} simulations at the geometric level", numSimulations);

        UpdateExpectedValue();
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
                uncertainty.push_back(1 - measuredLocal.data.at(i).confidence);
            }
        }

        float confidenceSum = 0;
        for (const auto& u : uncertainty)
            confidenceSum += 1 - u;
        float residual = NACCeres::FitSingleScale(simulated, measured, uncertainty);
        return residual / confidenceSum;
    }

    long double GraphGSL::ProbFromResidual(long double residual)
    {
        if (!std::isfinite(residual))
            return 0.0;
        return std::exp(-residual / likelihoodSigma);
    }

    void GraphGSL::UpdateExpectedValue()
    {
        std::vector<Grid2D<float>> sourceProbs;
        for (const auto& node : graph.nodes)
        {
            auto room = As<RoomNode>(node);
            if (room)
                sourceProbs.push_back(room->GetSourceProbabilities());
        }

        MultiGrid mgrid(sourceProbs);
        expectedValue = Utils::ExpectedValue(mgrid, 1.0);
        cov = Utils::Covariance(mgrid);
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
        pubs.windPub->publish(graph.VisualizeWind(naiveEntireMap));
        pubs.measuredGasMapsPub->publish(graph.VisualizeGasReadings());
        pubs.simGasMapsPub->publish(simulationSystem.VisualizeCachedResults(simulationViz.selectedNode, simulationViz.simulationIndex, graph.vizOptions.nodeSeparationViz));
#if ENABLE_NAIVE_EVALUATION
        if (naiveSimulationIndex < naiveCompleteMaps.size())
            naiveMapsPub->publish(Graph_internal::VisualizeCompleteMap(naiveCompleteMaps.at(naiveSimulationIndex), {naiveEntireMap}, graph.vizOptions.nodeSeparationViz, 0.2));
#endif
        pubs.quadtreePub->publish(graph.VisualizeMapSegmentation());
        pubs.sourceProbPub->publish(graph.VisualizeSourceProbs());
        Utils::publishPositionWCovariance(vmath::WithZ(expectedValue, 0.6), cov, "/expected_source_position");
    }

} // namespace GSL