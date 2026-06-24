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

#define SCALE_EXPECTED_MAPS 0
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
        gmrfw::CGMRF_map::Parameters gmrfParams;
        gmrfParams.cell_size = cellSize;
        gmrfParams.lambdaPrior_advection = rclnode->declare_parameter<float>("GMRF_lambdaPrior_advection");
        gmrfParams.lambdaPrior_diffusion = rclnode->declare_parameter<float>("GMRF_lambdaPrior_diffusion");
        gmrfParams.lambdaPrior_mass_conservation = rclnode->declare_parameter<float>("GMRF_lambdaPrior_mass_conservation");
        gmrfParams.lambdaPrior_obstacles = rclnode->declare_parameter<float>("GMRF_lambdaPrior_obstacles");
        gmrfParams.picard_convergence_thr = rclnode->declare_parameter<float>("GMRF_picard_convergence_thr");
        gmrfParams.lambda_regularization = rclnode->declare_parameter<float>("GMRF_lambda_regularization");

        // kernel
        KernelDMVW::GasMap::Params kernelParams;
        kernelParams.kernelSigma = rclnode->declare_parameter<float>("kernel_sigma", 0.3);
        kernelParams.kernelStretchConstant = rclnode->declare_parameter<float>("kernel_stretch_constant", 0.5);
        kernelParams.sigmaOmega = rclnode->declare_parameter<float>("kernel_sigma_omega", 0.1);
        kernelParams.omegaConcentrationSpatial = rclnode->declare_parameter<float>("kernel_omega_concentration_spatial", 5.0);

        // graph creation
        std::filesystem::path path =
            rclnode->declare_parameter<std::string>("graph_path",
                                                    std::filesystem::path(ament_index_cpp::get_package_share_directory("graphgsl_env")) / "second_graph");
        graph = Graph::ReadFromDisk(path, cellSize, gmrfParams, kernelParams);
        float artificialSeparation = rclnode->declare_parameter<float>("node_separation_mult", 1);
        graph.vizOptions.nodeSeparationViz = artificialSeparation;
        simulationSystem.graph = &graph;

#if ENABLE_NAIVE_EVALUATION
        naiveEntireMap = std::make_shared<RoomNode>(graph.completeMap.AsGrid(), kernelParams);
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
        pubs.infoGainPub = rclnode->create_publisher<MarkerArray>("infoGain", 1);
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
        expectedGasMaps.clear();
        simulationSystem.Reset();
        ThreadPool pool;

        // Run the simulations
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
                    pool.QueueJob([&, doorway]()
                                  { simulationSystem.SimulateEntireGraph(doorway); });
            }
            pool.Wait();
        }

        // calculate the residuals
        std::vector<NodeResult> nodeResiduals;
        {
            ScopedStopwatch watch("Room residuals");

            nodeResiduals.reserve(graph.nodes.size());
            std::mutex mtx;
            for (const auto& node : graph.nodes)
                pool.QueueJob([&]()
                              { GetNodeResidual(node, nodeResiduals, mtx); });
            pool.Wait();
        }

        // turn the residuals into probabilities
        CalculateNodeProbabilities(nodeResiduals);

        // now, fine level estimations
        // starting with the most promising room, do fine-level estimations
        // if the fine-level simulations are not as good as the first (room-level) estimates, we might need to look at the second-best room as well
        // and so on, until we find an actually good candidate
        std::ranges::sort(nodeResiduals, [](const auto& a, const auto& b)
                          { return a.residual < b.residual; });

        constexpr float doFineLevelThreshold = 0.3;
        std::vector<std::shared_ptr<RoomNode>> simulatedFineLevel;
        if (graph.roomSourceProbabilities.at(nodeResiduals.at(0).node) > doFineLevelThreshold)
        {
            size_t i = 0;
            bool done = false;
            do
            {
                auto room = As<RoomNode>(nodeResiduals.at(i).node);
                if (room)
                {
                    GSL_INFO("Running fine-level simulations in node: {} (residual: {:.3e})", room->id, nodeResiduals.at(i).residual);
                    simulatedFineLevel.push_back(room);
                    float lowestResidual = EvaluateSourceProbabilitiesInRooms({room});
                    nodeResiduals.at(i).residual = lowestResidual; // if no candidate position matched the ideal doorway distribution, overwrite the room residual with this more realistic value
                    GSL_TRACE("Lowest residual for node {}: {:.3e}", room->id, lowestResidual);

                    constexpr float toleranceFactor = 1.0;
                    if (i + 1 == nodeResiduals.size())
                    {
                        GSL_TRACE("No more nodes available for fine-level simulation");
                        done = true;
                    }
                    else if (lowestResidual <= nodeResiduals.at(i + 1).residual * toleranceFactor)
                    {
                        GSL_TRACE("Stopping fine-level simulation (next residual: {:.3e} -- {})",
                                  nodeResiduals.at(i + 1).residual,
                                  nodeResiduals.at(i + 1).node->id);
                        done = true;
                    }
                }
                else
                {
                    GSL_TRACE("Stopping at node {} (residual: {:.3e}) -- not a room", nodeResiduals.at(i).node->id, nodeResiduals.at(i).residual);
                    done = true;
                }

                i++;
            } while (!done);

            // re-calculate the probabilities from the optimization residuals, including the fine level results
            CalculateNodeProbabilities(nodeResiduals);
        }

        // normalize the conditional probabilities -- p(s|r)
        for (auto node : graph.nodes)
        {
            auto room = As<RoomNode>(node);
            if (!room)
                continue;

            Grid2D<float> sourceProbs = room->GetSourceProbabilities();

            // if we did simulations in this room, calculate the probabilities from the residuals
            if (Utils::contains(simulatedFineLevel, room))
            {
                std::vector<long double> tempProbs(sourceProbs.data.size());
                // residual -> prob
                for (size_t i = 0; i < sourceProbs.data.size(); i++)
                    if (sourceProbs.occupancy.at(i))
                        tempProbs.at(i) = ProbFromResidual(sourceProbs.data.at(i));
                // normalize
                Utils::NormalizeDistribution(tempProbs, room->GetSourceProbabilities().occupancy);

                // asign the probs to the room grid
                for (size_t i = 0; i < sourceProbs.data.size(); i++)
                    room->GetSourceProbabilities().data.at(i) = graph.roomSourceProbabilities.at(room) * tempProbs.at(i);
            }
            // otherwise, set all the cells in the room to the same probability (old probs might not be reliable anymore)
            else
            {
                for (size_t i = 0; i < sourceProbs.data.size(); i++)
                    if (sourceProbs.occupancy.at(i))
                    {
                        sourceProbs.data.at(i) = graph.roomSourceProbabilities.at(room) / sourceProbs.metadata.numFreeCells;
                        // there was no finer-level simulation for this node, so just assume that a source placed at this point would
                        // generate the same gas map as the entire room
                        expectedGasMaps[room->GetCellIdentifier(i)] = expectedGasMaps.at(room->GetNodeIdentifier());
                    }
            }
        }

        UpdateExpectedValue();
        // UpdateInformationGain();
    }

    float GraphGSL::EvaluateSourceProbabilitiesInRooms(std::vector<std::shared_ptr<RoomNode>> roomNodes)
    {
        ZoneScopedN("Room level");
        constexpr float AABBCENTER = -1; // used to distinguish aabb entries in the expectedGasMaps structure

        ScopedStopwatch watch("Evaluation (source probabilities in room)");
        ThreadPool pool;
        std::mutex mtx;

        // remove the whole-room simulation from the expected maps structure, since we are doing finer simulation
        for (auto roomNode : roomNodes)
        {
            CellIdentifier id{roomNode.get(), CellIdentifier::WHOLE_NODE};
            expectedGasMaps.erase(id);
        }

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
                                  Vector2 sourcePoint = aabb.center();
                                  Graph_internal::CompleteMap& map = simulationSystem.SimulateEntireGraph(roomNode, sourcePoint);
                                  Result result{std::make_shared<Region>(roomNode, nqaNode), &map};
                                  auto [residual, scale] = ResidualSingleSimulation(*result.map);
                                  GSL_ASSERT(std::isfinite(residual));

                                  // indices are multiplied by 100 to signal that this is a special case (aabb center, rather than single cell)
                                  CellIdentifier id{.node = roomNode.get(),
                                                    .indices = roomNode->GetOccupancy().metadata.coordinatesToIndices(sourcePoint) * AABBCENTER};

                                  {
                                      std::scoped_lock lock(mtx);
                                      numSimulations++;
                                      residualsThisLevel.push_back({result, residual});

                                      expectedGasMaps[id] = {.map = std::make_shared<Graph_internal::CompleteMap>()};
                                  }

                                  // store the (scaled?) result for movement strategy
                                  for (const auto& [room, localSimMap] : map.gasMaps)
                                  {
                                      expectedGasMaps[id].map->gasMaps[room].resize(localSimMap.size(), 0);
#if SCALE_EXPECTED_MAPS
                                      for (size_t i = 0; i < localSimMap.size(); i++)
                                          expectedGasMaps[id].map->gasMaps[room].at(i) = std::log(localSimMap.at(i) * scale + 1);
#else
                                      for (size_t i = 0; i < localSimMap.size(); i++)
                                          expectedGasMaps[id].map->gasMaps[room].at(i) = localSimMap.at(i);
#endif
                                  } });
            }
            pool.Wait();

            // sort the results by the residuals
            std::ranges::sort(residualsThisLevel, [](const auto& a, const auto& b)
                              { return a.second < b.second; });

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
            GSL_TRACE("Completed a simulation level -- total simulations: {}", numSimulations);
        } while (!queue.empty());
        GSL_INFO("Ran {} simulations at the geometric level", numSimulations);

        // Assign the fine-level residuals to the corresponding cells
        float lowestResidual = std::numeric_limits<float>::max();
        for (const auto& [region, residual] : finalResiduals)
        {
            if (residual < lowestResidual)
                lowestResidual = residual;
            AABB2DInt aabbi = region->nqaNode.getAABB();
            AABB2D aabb = region->room->GetOccupancy().metadata.indicesToCoordinates(aabbi);
            Vector2Int centerIndices = region->room->GetOccupancy().metadata.coordinatesToIndices(aabb.center()) * AABBCENTER;
            CellIdentifier centerID{.node = region->room.get(),
                                    .indices = centerIndices};
            for (Vector2Int pos : aabbi)
            {
                region->room->GetSourceProbabilities().dataAt(pos) = residual;

                // store the results of the finest simulation that includes this cell as representative of the cell itself
                CellIdentifier thisID{region->room.get(), pos};
                expectedGasMaps[thisID] = expectedGasMaps.at(centerID);
            }
        }

        return lowestResidual;
    }

    void GraphGSL::GetNodeResidual(std::shared_ptr<PlaceNode> sourceNode, std::vector<NodeResult>& nodeResiduals, std::mutex& mtx)
    {
        // to make the layout convenient for the optimization, each row corresponds to a cell in the map, and each column to a simulation
        std::vector<std::vector<float>> simulated;
        simulated.reserve(300);

        // while we fill a row of the matrix for each simulation out of this node, we only want one copy of the measurements and their uncertainty
        // if we are done recording those, skip them on the next iteration
        std::vector<float> measured;
        std::vector<float> uncertainty;

        float skippedCellsResidual = 0; // to reduce computational complexity, we are going to avoid adding 0-confidence cells to the optimization problem
                                        // we can calculate what their residual should be anyways, since at 0 confidence it is actually a constant

        // iterate over all the simulated maps and record the simulated-measured-confidence triplets
        {
            size_t simIndex = 0;
            for (const Graph_internal::CompleteMap& simulation : simulationSystem.gasMapsWithRoomSource.at(sourceNode))
            {
                size_t cellIdx = 0; // count the valid cells
                for (const auto& [room, localSimMap] : simulation.gasMaps)
                {
                    if (room == sourceNode)
                    {
                        if (simIndex == 0)
                            skippedCellsResidual += room->GetOccupancy().metadata.numFreeCells * NACCeres::defaultResidual;
                        continue;
                    }

                    Grid2D<KernelDMVW::KernelCell> measuredLocal = room->GetGasMap();
                    for (size_t i = 0; i < localSimMap.size(); i++)
                    {
                        if (!measuredLocal.occupancy.at(i))
                            continue;
                        KernelDMVW::KernelCell& cell = measuredLocal.data.at(i);

                        if (cell.confidence < 0.05)
                        {
                            if (simIndex == 0)
                                skippedCellsResidual += NACCeres::defaultResidual;
                            continue;
                        }

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
        }

        float confidenceSum = 0;
        for (const auto& u : uncertainty)
            confidenceSum += 1 - u;

        NACCeres::MultipleScales result = GSL::NACCeres::FitDoorwayScales(simulated, measured, uncertainty);
        mtx.lock();

        nodeResiduals.push_back({sourceNode, 0, 0});
        nodeResiduals.back().residual = (std::isfinite(result.residual) ? result.residual : 0) + skippedCellsResidual;
        nodeResiduals.back().residual /= graph.TotalFreeCellsCount();
        nodeResiduals.back().confidenceSum = confidenceSum;
        GSL_INFO("Residual at {}: {:.2e}, Confidence Sum: {:.2e}. Scales: {:.2f}",
                 sourceNode->id,
                 nodeResiduals.back().residual,
                 nodeResiduals.back().confidenceSum,
                 fmt::join(result.scales.begin(), result.scales.end(), ", "));

        // store the scaled sum of the simulation results, to later evaluate the most interesting points for future measurement
        CellIdentifier id{.node = sourceNode.get(), .indices = CellIdentifier::WHOLE_NODE};
        expectedGasMaps[id] = {.map = std::make_shared<Graph_internal::CompleteMap>()};
        mtx.unlock();

        if (std::isfinite(result.residual))
        {
            float scaleSum = std::accumulate(result.scales.begin(), result.scales.end(), 0.0f);
            size_t simIndex = 0;
            for (const Graph_internal::CompleteMap& simulation : simulationSystem.gasMapsWithRoomSource.at(sourceNode))
            {
                for (const auto& [room, localSimMap] : simulation.gasMaps)
                {
                    if (room == sourceNode)
                        continue;
                    if (!expectedGasMaps[id].map->gasMaps.contains(room))
                        expectedGasMaps[id].map->gasMaps[room].resize(localSimMap.size(), 0);
#if SCALE_EXPECTED_MAPS
                    for (size_t i = 0; i < localSimMap.size(); i++)
                        expectedGasMaps[id].map->gasMaps[room].at(i) += std::log(result.scales.at(simIndex) * localSimMap.at(i) + 1);
#else
                    for (size_t i = 0; i < localSimMap.size(); i++)
                        expectedGasMaps[id].map->gasMaps[room].at(i) += (result.scales.at(simIndex) / scaleSum) * localSimMap.at(i);
#endif
                }

                simIndex++;
            }
        }
    }

    std::pair<float, float> GraphGSL::ResidualSingleSimulation(const Graph_internal::CompleteMap& simMap)
    {
        ZoneScopedN("Residual calculation");

#define MEASUREMENT_SORTING 0
#if MEASUREMENT_SORTING
        // this whole thing comes from here: https://arxiv.org/abs/2605.13208

        struct CellValue
        {
            const KernelDMVW::KernelCell* kernelCell;
            float value;
        };
        std::vector<CellValue> measuredValues;
        std::vector<CellValue> simulatedValues;

        struct NormalizedOrder
        {
            float measured;
            float simulated;
        };

        std::map<const KernelDMVW::KernelCell*, NormalizedOrder> normalizedOrders;
        auto EvaluateSorting = [&]() -> float
        {
            std::ranges::sort(measuredValues, [](const CellValue& a, const CellValue& b)
                              { return a.value < b.value; });
            std::ranges::sort(simulatedValues, [](const CellValue& a, const CellValue& b)
                              { return a.value < b.value; });

            for (size_t i = 0; i < measuredValues.size(); i++)
                normalizedOrders[measuredValues[i].kernelCell].measured = float(i) / measuredValues.size();

            for (size_t i = 0; i < simulatedValues.size(); i++)
                normalizedOrders[simulatedValues[i].kernelCell].simulated = float(i) / simulatedValues.size();

            float sum = 0;
            size_t n = 0;
            constexpr float sigma = 10; // ¯\_(ツ)_/¯
            for (const auto& [kernelCell, normalizedOrder] : normalizedOrders)
            {
                float diff = normalizedOrder.measured - normalizedOrder.simulated;
                sum += diff * diff / (sigma * sigma);
                n++;
            }
            return 0.5f * (float(n) / n + 1) * sum;
        };
#endif

        std::vector<float> measured;
        std::vector<float> simulated;
        std::vector<float> uncertainty;

        float skippedCellsResidual = 0;
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
                {
                    skippedCellsResidual += NACCeres::defaultResidual;
                    continue;
                }

                measured.push_back(cell.meanAndVariance.mean);
                simulated.push_back(localSimMap.at(i));
                uncertainty.push_back(1 - measuredLocal.data.at(i).confidence);
#if MEASUREMENT_SORTING
                measuredValues.push_back({&cell, measured.back()});
                simulatedValues.push_back({&cell, simulated.back()});
#endif
            }
        }

#if MEASUREMENT_SORTING
        return EvaluateSorting();
#else
        NACCeres::SingleScale result = NACCeres::FitSingleScale(simulated, measured, uncertainty);

        float finalResidual = result.residual + skippedCellsResidual;
        return {finalResidual / graph.TotalFreeCellsCount(), result.scale};
#endif
    }

    long double GraphGSL::ProbFromResidual(long double residual)
    {
        if (!std::isfinite(residual))
            return 0.0;
        return std::exp(-residual / likelihoodSigma);
    }

    void GraphGSL::CalculateNodeProbabilities(const std::vector<NodeResult>& residuals)
    {
        graph.roomSourceProbabilities.clear();
        GSL_INFO("Results with sigma={:.2f}", likelihoodSigma);
        std::map<std::shared_ptr<PlaceNode>, long double> scores;
        long double scoresSum = 0;
        for (const NodeResult& result : residuals)
        {
            scores[result.node] = ProbFromResidual(result.residual);
            scoresSum += scores[result.node];
        }

        for (const auto& [room, score] : scores)
        {
            double prob = score / scoresSum;
            graph.roomSourceProbabilities[room] = prob;
            GSL_INFO("\t{}: {:.2f}", room->id, prob);
        }
    }

    void GraphGSL::UpdateExpectedValue()
    {
        MultiGrid mgrid = graph.GetAllSourceProbs();
        expectedValue = Utils::ExpectedValue(mgrid, expectedValueProportion);
        cov = Utils::Covariance(mgrid);
    }

    void GraphGSL::UpdateInformationGain()
    {
        GSL_INFO("Updating information gain");
        ScopedStopwatch watch("info gain");
        std::vector<CellIdentifier> allFreeCells = graph.GetAllFreeCells();

        // reset all the information from previous simulations
#pragma omp parallel for schedule(dynamic, 50)
        for (const auto& id : allFreeCells)
            As<RoomNode>(id.node)->GetExpectedVariances().dataAt(id.indices).Reset();

        // start updating the values with the latest results

        auto updateWithExpectedMap = [&](const Graph_internal::CompleteMap& completeMap, float probability)
        {
            for (const auto& entry : completeMap.gasMaps)
            {
                const auto& room = entry.first;
                const auto& map = entry.second;
#pragma omp parallel for schedule(dynamic, 50)
                for (size_t i = 0; i < map.size(); ++i)
                {
                    if (!room->GetOccupancy().data.at(i))
                        continue;
                    float value = map.at(i);
                    room->GetExpectedVariances().data.at(i).Update(value, probability);
                    GSL_ASSERT(std::isfinite(room->GetExpectedVariances().data.at(i).variance));
                }
            }
        };

        for (auto node : graph.nodes)
        {
            auto room = As<RoomNode>(node);
            if (!room)
                continue;

            if (expectedGasMaps.contains({room.get(), CellIdentifier::WHOLE_NODE}))
            {
                float probability = graph.roomSourceProbabilities.at(room);
                Graph_internal::CompleteMap& completeMap = *expectedGasMaps.at({room.get(), CellIdentifier::WHOLE_NODE}).map;
                updateWithExpectedMap(completeMap, probability);
            }
            else
            {
                Grid2D<float> probabilities = room->GetSourceProbabilities();
                for (size_t cellIndex = 0; cellIndex < probabilities.data.size(); cellIndex++)
                {
                    float probability = probabilities.data.at(cellIndex);
                    if (probability < 1e-7)
                        continue;

                    CellIdentifier sourceID = room->GetCellIdentifier(cellIndex);
                    Graph_internal::CompleteMap& completeMap = *expectedGasMaps.at(sourceID).map;
                    updateWithExpectedMap(completeMap, probability);
                }
            }
        }
    }

#if ENABLE_NAIVE_EVALUATION
    void GraphGSL::EvaluateSourceProbabilitiesInAllRooms()
    {
        simulationSystem.Reset();
        naiveEntireMap->UpdateWindMap(graph.gmrf);
        // TODO store the results for visualization?
        std::vector<std::shared_ptr<RoomNode>> roomNodes;
        for (const auto& node : graph.nodes)
        {
            auto room = As<RoomNode>(node);
            if (room)
                roomNodes.push_back(room);
        }
        EvaluateSourceProbabilitiesInRooms(roomNodes);
    }

    void GraphGSL::EvaluateProbabilitiesNaive()
    {
        simulationSystem.Reset();
        naiveEntireMap->UpdateWindMap(graph.gmrf);
        // TODO store the results for visualization?
        EvaluateSourceProbabilitiesInRooms({naiveEntireMap});
    }
#endif

    void GraphGSL::Visualize()
    {
        pubs.graphPub->publish(graph.VisualizeGraph());
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
        pubs.infoGainPub->publish(graph.VisualizeInfoGain());
        UpdateExpectedValue();
        Utils::publishPositionWCovariance(vmath::WithZ(expectedValue, 0.6), cov, "/expected_source_position");
    }

} // namespace GSL