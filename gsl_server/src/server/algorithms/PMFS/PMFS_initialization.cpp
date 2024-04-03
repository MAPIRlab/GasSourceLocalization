#include <gsl_server/algorithms/PMFS/PMFS.hpp>
#include <gsl_server/algorithms/PMFS/PMFSLib.hpp>

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

        int scale = getParam<int>("scale", 65); // scale for dynamic map reduction
        PMFSLib::initMetadata(gridMetadata, map, scale);

        hitProbability.resize(gridMetadata.height * gridMetadata.width);
        sourceProbability.resize(gridMetadata.height * gridMetadata.width);
        navigationOccupancy.resize(gridMetadata.height * gridMetadata.width);
        simulationOccupancy.resize(gridMetadata.height * gridMetadata.width);

        functionQueue.submit(std::bind(&PMFS::initializeMap, this));
    }

} // namespace GSL