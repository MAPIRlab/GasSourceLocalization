#include <algorithms/PMFS/PMFS.h>
namespace PMFS
{

	using namespace std::placeholders;

	PMFS_GSL::PMFS_GSL(std::shared_ptr<rclcpp::Node> _node) : GSLAlgorithm(_node)
	{}

	void PMFS_GSL::initialize()
	{
		GSLAlgorithm::initialize();
		// A lot of the initialization is done inside of the map callback, rather than here. That is because we need to know the map beforehand
		// see the file grid_callbacks.cpp

		pubs.markers.source_probability_markers = node->create_publisher<Marker>("probability_markers", 1);
		pubs.markers.hitProbabilityMarkers = node->create_publisher<Marker>("hitProbabilityMarkers", 1);
		pubs.markers.confidenceMarkers = node->create_publisher<Marker>("confidenceMarkers", 1);
		pubs.markers.windArrowMarkers = node->create_publisher<MarkerArray>("arrowMarkers", 1);
		pubs.markers.gradientMarkers = node->create_publisher<MarkerArray>("gradientMarkers", 1);

		pubs.markers.quadtreePublisher = node->create_publisher<MarkerArray>("quadtree", 1);

		pubs.markers.debug.explorationValue = node->create_publisher<Marker>("explorationValue", 1);
		pubs.markers.debug.varianceHit = node->create_publisher<Marker>("varianceHit", 1);
		pubs.markers.debug.movementSets = node->create_publisher<Marker>("movementSets", 1);

		// Init State
		previous_state = State::WAITING_FOR_MAP;
		current_state = State::WAITING_FOR_MAP;
		spdlog::info("WAITING_FOR_MAP");

		pubs.clientWindGMRF = node->create_client<WindEstimation>("/WindEstimation");
		#ifdef USE_GADEN
		pubs.clientWindGroundTruth = node->create_client<gaden_player::srv::WindPosition>("/wind_value");
		#endif
		exploredCells = 0;
		iterationsCounter = 1;
		simulations.grid = this;
	}

	void PMFS_GSL::declareParameters()
	{
		GSLAlgorithm::declareParameters();
		settings.th_gas_present = getParam<double>("th_gas_present", 0.3);
		settings.th_wind_present = getParam<double>("th_wind_present", 0.05);
		settings.stop_and_measure_time = getParam<double>("stop_and_measure_time", 3);

		settings.scale = getParam<int>("scale", 65); // scale for dynamic map reduction

		settings.hitProbability.prior = getParam<double>("hitPriorProbability", 0.1);

		settings.movement.openMoveSetExpasion = getParam<int>("openMoveSetExpasion", 5); // number of cells in each direction that we add to the open move set in each step
		settings.movement.allowMovementRepetition = getParam<bool>("allowMovementRepetition", false);
		settings.movement.explorationProbability = getParam<double>("explorationProbability", 0.1);
		settings.movement.initialExplorationMoves = getParam<int>("initialExplorationMoves", 5);

		// hit probability
		settings.hitProbability.max_updates_per_stop = getParam<int>("max_updates_per_stop", 3);

		settings.hitProbability.kernel_sigma = getParam<double>("kernel_sigma", 0.5);
		settings.hitProbability.kernel_stretch_constant = getParam<double>("kernel_stretch_constant", 1);

		settings.hitProbability.confidence_measurement_weight = getParam<double>("confidence_measurement_weight", 0.5);
		settings.hitProbability.confidence_sigma_spatial = getParam<double>("confidence_sigma_spatial", 0.5);

		//
		settings.simulation.useWindGroundTruth = getParam<bool>("useWindGroundTruth", false);

		settings.simulation.sourceDiscriminationPower = getParam<double>("sourceDiscriminationPower", 1);
		settings.simulation.refineFraction = getParam<double>("refineFraction", 10);
		settings.simulation.steps_between_source_updates = getParam<int>("stepsSourceUpdate", 10);

		settings.simulation.maxRegionSize = getParam<int>("maxRegionSize", 5);
		settings.simulation.deltaTime = getParam<double>("deltaTime", 0.2);
		settings.simulation.noiseSTDev = getParam<double>("noiseSTDev", 0.5);
		settings.simulation.iterationsToRecord = getParam<int>("iterationsToRecord", 200);
		settings.simulation.maxWarmupIterations = getParam<int>("maxWarmupIterations", 500);

		settings.markers_height = getParam<double>("markers_height", 0);

		settings.localEstimationWindowSize = getParam<int>("localEstimationWindowSize", 2);

		settings.declaration.threshold = getParam<double>("convergence_thr", 0.5); // threshold for source declaration
		settings.declaration.steps = getParam<int>("convergence_steps", 5);
	}

	PMFS_GSL::~PMFS_GSL()
	{
		spdlog::info("- Closing...");
	}

	void PMFS_GSL::setUpMaps()
	{
		rclcpp::Rate rate(1);
		while (robot_poses_vector.size() == 0)
		{
			rate.sleep();
			rclcpp::spin_some(node);
			if (verbose)
				spdlog::info("Waiting to hear from localization topic: {}", localization_sub_->get_topic_name());
		}

		std::vector<std::vector<uint8_t>> mapa(map_.info.height, std::vector<uint8_t>(map_.info.width));
		int index = 0;
		for (int i = 0; i < mapa.size(); i++)
		{
			for (int j = 0; j < mapa[0].size(); j++)
			{
				mapa[i][j] = map_.data[index];
				index++;
			}
		}

		cells.resize(floor((float)map_.info.height / settings.scale),
			std::vector<Cell>(floor((float)map_.info.width / settings.scale)));

		// create the cells
		{
			for (int i = 0; i < cells.size(); i++)
			{
				for (int j = 0; j < cells[0].size(); j++)
				{
					bool squareIsFree = true;

					for (int row = i * settings.scale; row < (i + 1) * settings.scale; row++)
					{
						for (int col = j * settings.scale; col < (j + 1) * settings.scale; col++)
						{
							if (mapa[row][col] != 0)
							{
								squareIsFree = false;
							}
						}
					}
					cells[i][j].free = squareIsFree;
					cells[i][j].hitProbability.logOdds = std::log(settings.hitProbability.prior / (1 - settings.hitProbability.prior));
					cells[i][j].hitProbability.auxWeight = -1; // this is used to prune the cells that are free but unreachable
				}
			}
		}

		// pruning unreachable free cells
		{
			std::vector<std::vector<uint8_t>> occupancyMap(cells.size(), std::vector<uint8_t>(cells[0].size()));
			std::string anemometer_frame = getParam<std::string>("anemometer_frame", "anemometer_frame");
			geometry_msgs::msg::TransformStamped tfm = tf_buffer->lookupTransform("map", anemometer_frame, rclcpp::Time(0));
			float anemometer_Z = tfm.transform.translation.z;
			spdlog::info("anemometer z is {}", anemometer_Z);
			numFreeCells = 0;
			hashSet openPropagationSet;
			hashSet activePropagationSet;
			hashSet closedPropagationSet;
			currentPosIndex = coordinatesToIndex(current_robot_pose.pose.pose.position.x, current_robot_pose.pose.pose.position.y);
			;
			Vector2Int& curr = currentPosIndex;
			cells[curr.x][curr.y].hitProbability.auxWeight = 0;
			activePropagationSet.insert(curr);
			propagateProbabilities(cells, openPropagationSet, closedPropagationSet, activePropagationSet, { 1, {1, 0}, 0 });

			pubs.GMRFRequest = std::make_shared<WindEstimation::Request>();
			#ifdef USE_GADEN
			pubs.groundTruthWindRequest = std::make_shared<gaden_player::srv::WindPosition::Request>();
			#endif

			for (int i = 0; i < cells.size(); i++)
			{
				for (int j = 0; j < cells[0].size(); j++)
				{
					if (!cells[i][j].free || cells[i][j].hitProbability.auxWeight == -1)
					{
						cells[i][j].hitProbability.logOdds = DBL_MIN;
						cells[i][j].free = false;
					}
					else
					{
						numFreeCells++;
						Utils::Vector2 coords = indexToCoordinates(i, j);
						pubs.GMRFRequest->x.push_back(coords.x);
						pubs.GMRFRequest->y.push_back(coords.y);
						#ifdef USE_GADEN
						pubs.groundTruthWindRequest->x.push_back(coords.x);
						pubs.groundTruthWindRequest->y.push_back(coords.y);
						pubs.groundTruthWindRequest->z.push_back(anemometer_Z);
						#endif
					}

					occupancyMap[i][j] = cells[i][j].free ? 1 : 0;
				}
			}

			simulations.quadtree = std::unique_ptr<NQA::NQAQuadtree>(new NQA::NQAQuadtree(occupancyMap));
			simulations.QTleaves = simulations.quadtree->fusedLeaves(settings.simulation.maxRegionSize);

			simulations.mapSegmentation.resize(cells.size(), std::vector<NQA::Node*>(cells[0].size(), nullptr));

			spdlog::info("Number of cells after fusing quadtree: {0}", simulations.QTleaves.size());
			// generate the image of indices so you can map a cell in the map to the corresponding leaf of the quatree
			for (int i = 0; i < simulations.QTleaves.size(); i++)
			{
				NQA::Node& node = simulations.QTleaves[i];
				Utils::Vector2Int start = node.origin;
				Utils::Vector2Int end = node.origin + node.size;

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
			int visibility_map_range = std::max(settings.movement.openMoveSetExpasion, settings.localEstimationWindowSize);
			for (int i = 0; i < cells.size(); i++)
			{
				for (int j = 0; j < cells[0].size(); j++)
				{
					if (!cells[i][j].free)
						continue;
					int oI = std::max(0, i - visibility_map_range) - i;
					int fI = std::min((int)cells.size() - 1, i + visibility_map_range) - i;
					int oJ = std::max(0, j - visibility_map_range) - j;
					int fJ = std::min((int)cells[0].size() - 1, j + visibility_map_range) - j;

					Vector2Int ij(i, j);
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
		for (int i = 0; i < cells.size(); i++)
		{
			for (int j = 0; j < cells[0].size(); j++)
			{
				cells[i][j].sourceProbability = 1.0 / numFreeCells;
			}
		}

		estimatedWindVectors = std::vector<std::vector<Utils::Vector2>>(cells.size(), std::vector<Utils::Vector2>(cells[0].size()));
		estimateWind(settings.simulation.useWindGroundTruth);

		simulations.varianceOfHitProb.resize(cells.size(), std::vector<double>(cells[0].size(), 0));

		// Start the fun!!
		cancel_navigation(true);
		start_time = node->now();   // start measuring time
		robot_poses_vector.clear(); // start measuring distance
		inExecution = true;

		spdlog::warn("EXPLORATION");
	}

	Cell::Cell()
	{
		free = false;
		distanceFromRobot = 0;
		sourceProbability = 0;
		hitProbability.logOdds = 0;
		hitProbability.auxWeight = 0;
		hitProbability.omega = 0;
		hitProbability.confidence = 0;
	}

	Cell::~Cell()
	{
	}

}