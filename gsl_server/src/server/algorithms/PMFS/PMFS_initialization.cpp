#include <gsl_server/algorithms/PMFS/PMFS.hpp>

namespace GSL
{
    using WindEstimation = gmrf_wind_mapping::srv::WindEstimation;
    PMFS::PMFS(std::shared_ptr<rclcpp::Node> _node) : Algorithm(_node),
        simulations(this)
#ifdef USE_GUI
        ,ui(this)
#endif
    {}

    void PMFS::initialize()
    {
        Algorithm::initialize();
        // A lot of the initialization is done inside of the map callback, rather than here. That is because we need to know the map beforehand

        pubs.markers.source_probability_markers = node->create_publisher<Marker>("probability_markers", 1);
        pubs.markers.hitProbabilityMarkers = node->create_publisher<Marker>("hitProbabilityMarkers", 1);
        pubs.markers.confidenceMarkers = node->create_publisher<Marker>("confidenceMarkers", 1);
        pubs.markers.windArrowMarkers = node->create_publisher<MarkerArray>("arrowMarkers", 1);
        pubs.markers.gradientMarkers = node->create_publisher<MarkerArray>("gradientMarkers", 1);

        pubs.markers.quadtreePublisher = node->create_publisher<MarkerArray>("quadtree", 1);

        pubs.markers.debug.explorationValue = node->create_publisher<Marker>("explorationValue", 1);
        pubs.markers.debug.varianceHit = node->create_publisher<Marker>("varianceHit", 1);
        pubs.markers.debug.movementSets = node->create_publisher<Marker>("movementSets", 1);

        pubs.clientWindGMRF = node->create_client<WindEstimation>("/WindEstimation");
#ifdef USE_GADEN
        pubs.clientWindGroundTruth = node->create_client<gaden_player::srv::WindPosition>("/wind_value");
#endif

#ifdef USE_GUI
		if(!settings.visualization.headless)
        	ui.run();
#endif

        iterationsCounter = 0;

        waitForGasState = std::make_unique<WaitForGasState>(this);
        waitForMapState = std::make_unique<WaitForMapState>(this);
        waitForMapState->shouldWaitForGas = false;

        stopAndMeasureState = std::make_unique<StopAndMeasureState>(this);
        movingState = std::make_unique<MovingStatePMFS>(this);
        stateMachine.forceSetState(waitForMapState.get());
    }

    void PMFS::declareParameters()
    {
        Algorithm::declareParameters();
        settings.hitProbability.prior = getParam<double>("hitPriorProbability", 0.1);

        settings.movement.openMoveSetExpasion =
            getParam<int>("openMoveSetExpasion", 5); // number of cells in each direction that we add to the open move set in each step
        settings.movement.explorationProbability = getParam<double>("explorationProbability", 0.1);
        settings.movement.initialExplorationMoves = getParam<int>("initialExplorationMoves", 5);

        // hit probability
        settings.hitProbability.max_updates_per_stop = getParam<int>("max_updates_per_stop", 3);

        settings.hitProbability.kernel_sigma = getParam<double>("kernel_sigma", 0.5);
        settings.hitProbability.kernel_stretch_constant = getParam<double>("kernel_stretch_constant", 1);

        settings.hitProbability.confidence_measurement_weight = getParam<double>("confidence_measurement_weight", 0.5);
        settings.hitProbability.confidence_sigma_spatial = getParam<double>("confidence_sigma_spatial", 0.5);
        settings.hitProbability.localEstimationWindowSize = getParam<int>("localEstimationWindowSize", 2);

        // Simulation
        settings.simulation.useWindGroundTruth = getParam<bool>("useWindGroundTruth", false);

        settings.simulation.sourceDiscriminationPower = getParam<double>("sourceDiscriminationPower", 1);
        settings.simulation.refineFraction = getParam<double>("refineFraction", 10);
        settings.simulation.steps_between_source_updates = getParam<int>("stepsSourceUpdate", 10);

        settings.simulation.maxRegionSize = getParam<int>("maxRegionSize", 5);
        settings.simulation.deltaTime = getParam<double>("deltaTime", 0.2);
        settings.simulation.noiseSTDev = getParam<double>("noiseSTDev", 0.5);
        settings.simulation.iterationsToRecord = getParam<int>("iterationsToRecord", 200);
        settings.simulation.maxWarmupIterations = getParam<int>("maxWarmupIterations", 500);

        settings.visualization.markers_height = getParam<double>("markers_height", 0);
#if USE_GUI
        settings.visualization.headless = getParam<bool>("headless", false);
#endif

        settings.declaration.threshold = getParam<double>("convergence_thr", 0.5); // threshold for source declaration
        settings.declaration.steps = getParam<int>("convergence_steps", 5);
    }

    void PMFS::onGetMap(OccupancyGrid::SharedPtr msg)
    {
        Algorithm::onGetMap(msg);
        functionQueue.submit(std::bind(&PMFS::initializeMap, this));
    }

    void PMFS::initializeMap()
    {
        std::vector<std::vector<uint8_t>> mapa(map.info.height, std::vector<uint8_t>(map.info.width));
        int index = 0;
        for (int i = 0; i < mapa.size(); i++)
        {
            for (int j = 0; j < mapa[0].size(); j++)
            {
                mapa[i][j] = map.data[index];
                index++;
            }
        }

        int scale = getParam<int>("scale", 65); // scale for dynamic map reduction
        gridData.cellSize = map.info.resolution * scale;
        gridData.origin.x = map.info.origin.position.x;
        gridData.origin.y = map.info.origin.position.y;
        gridData.numFreeCells = 0;

        grid.resize(floor((float)map.info.height / scale), std::vector<Cell>(floor((float)map.info.width / scale)));

        // create the cells
        {
            for (int i = 0; i < grid.size(); i++)
            {
                for (int j = 0; j < grid[0].size(); j++)
                {
                    bool squareIsFree = true;

                    for (int row = i * scale; row < (i + 1) * scale; row++)
                    {
                        for (int col = j * scale; col < (j + 1) * scale; col++)
                        {
                            if (mapa[row][col] != 0)
                            {
                                squareIsFree = false;
                            }
                        }
                    }
                    grid[i][j].free = squareIsFree;
                    grid[i][j].hitProbability.logOdds = std::log(settings.hitProbability.prior / (1 - settings.hitProbability.prior));
                    grid[i][j].hitProbability.auxWeight = -1; // this is used to prune the cells that are free but unreachable
                }
            }
        }

        // pruning unreachable free cells
        {
            std::vector<std::vector<uint8_t>> occupancyMap(grid.size(), std::vector<uint8_t>(grid[0].size()));
            std::string anemometer_frame = getParam<std::string>("anemometer_frame", "anemometer_frame");

            geometry_msgs::msg::TransformStamped tfm;
            bool has_tf = false;
            do
            {
                try
                {
                    tfm = tf_buffer.buffer.lookupTransform("map", anemometer_frame, rclcpp::Time(0));
                    has_tf = true;
                }
                catch (std::exception& e)
                {
                    GSL_ERROR("TF error when looking up map -> anemometer:\n{}", e.what());
                    rclcpp::spin_some(node);
                }
            } while (!has_tf);

            float anemometer_Z = tfm.transform.translation.z;
            GSL_INFO("anemometer z is {}", anemometer_Z);
            hashSet openPropagationSet;
            hashSet activePropagationSet;
            hashSet closedPropagationSet;

            Vector2Int currentIndices = currentPosIndex();
            grid[currentIndices.x][currentIndices.y].hitProbability.auxWeight = 0;
            activePropagationSet.insert(currentIndices);
            propagateProbabilities(grid, openPropagationSet, closedPropagationSet, activePropagationSet, {1, {1, 0}, 0});

            pubs.GMRFRequest = std::make_shared<WindEstimation::Request>();
#ifdef USE_GADEN
            pubs.groundTruthWindRequest = std::make_shared<gaden_player::srv::WindPosition::Request>();
#endif

            for (int i = 0; i < grid.size(); i++)
            {
                for (int j = 0; j < grid[0].size(); j++)
                {
                    if (!grid[i][j].free || grid[i][j].hitProbability.auxWeight == -1)
                    {
                        grid[i][j].hitProbability.logOdds = DBL_MIN;
                        grid[i][j].free = false;
                    }
                    else
                    {
                        gridData.numFreeCells++;
                        Vector2 coords = gridData.indexToCoordinates(i, j);
                        pubs.GMRFRequest->x.push_back(coords.x);
                        pubs.GMRFRequest->y.push_back(coords.y);
#ifdef USE_GADEN
                        pubs.groundTruthWindRequest->x.push_back(coords.x);
                        pubs.groundTruthWindRequest->y.push_back(coords.y);
                        pubs.groundTruthWindRequest->z.push_back(anemometer_Z);
#endif
                    }

                    occupancyMap[i][j] = grid[i][j].free ? 1 : 0;
                }
            }

            simulations.quadtree = std::make_unique<Utils::NQA::Quadtree>(occupancyMap);
            simulations.QTleaves = simulations.quadtree->fusedLeaves(settings.simulation.maxRegionSize);

            simulations.mapSegmentation.resize(grid.size(), std::vector<Utils::NQA::Node*>(grid[0].size(), nullptr));

            GSL_INFO("Number of cells after fusing quadtree: {0}", simulations.QTleaves.size());
            // generate the image of indices so you can map a cell in the map to the corresponding leaf of the quatree
            for (int i = 0; i < simulations.QTleaves.size(); i++)
            {
                Utils::NQA::Node& node = simulations.QTleaves[i];
                Vector2Int start = node.origin;
                Vector2Int end = node.origin + node.size;

                for (int r = start.x; r < end.x; r++)
                {
                    for (int c = start.y; c < end.y; c++)
                    {
                        GSL_ASSERT_MSG(simulations.mapSegmentation[r][c] == nullptr, "fused cells are overlapping");
                        simulations.mapSegmentation[r][c] = &node;
                    }
                }
            }
        }

        // precomputed visibility map
        {
            int visibility_map_range = std::max(settings.movement.openMoveSetExpasion, settings.hitProbability.localEstimationWindowSize);
            for (int i = 0; i < grid.size(); i++)
            {
                for (int j = 0; j < grid[0].size(); j++)
                {
                    Vector2Int ij(i, j);
                    if (!grid[i][j].free)
					{
						visibilityMap.emplace(ij, hashSet{});
                        continue;
					}
                    int oI = std::max(0, i - visibility_map_range) - i;
                    int fI = std::min((int)grid.size() - 1, i + visibility_map_range) - i;
                    int oJ = std::max(0, j - visibility_map_range) - j;
                    int fJ = std::min((int)grid[0].size() - 1, j + visibility_map_range) - j;

                    hashSet visibleCells;
                    double totalX = 0;
                    double totalY = 0;
                    for (int r = oI; r <= fI; r++)
                    {
                        for (int c = oJ; c <= fJ; c++)
                        {
                            Vector2Int thisCell(i + r, j + c);
                            if (pathFree(ij, thisCell))
                            {
                                visibleCells.insert(thisCell);
                            }
                        }
                    }
                    visibilityMap.emplace(ij, visibleCells);
                }
            }
        }

        // source probability before we get any observations
        for (int i = 0; i < grid.size(); i++)
        {
            for (int j = 0; j < grid[0].size(); j++)
            {
                grid[i][j].sourceProbability = 1.0 / gridData.numFreeCells;
            }
        }

        estimatedWindVectors = std::vector<std::vector<Vector2>>(grid.size(), std::vector<Vector2>(grid[0].size()));
        estimateWind(settings.simulation.useWindGroundTruth);

        simulations.varianceOfHitProb.resize(grid.size(), std::vector<double>(grid[0].size(), 0));
        stateMachine.forceSetState(stopAndMeasureState.get());
    }
} // namespace GSL