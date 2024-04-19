#include <gsl_server/algorithms/PMFS/PMFS.hpp>
#include <gsl_server/algorithms/PMFS/PMFSLib.hpp>

namespace GSL
{
    using WindEstimation = gmrf_wind_mapping::srv::WindEstimation;
    PMFS::PMFS(std::shared_ptr<rclcpp::Node> _node) : Algorithm(_node),
        simulations(Grid<HitProbability>(hitProbability, occupancy, gridMetadata),
                    Grid<double>(sourceProbability, occupancy, gridMetadata),
                    Grid<Vector2>(estimatedWindVectors, occupancy, gridMetadata),
                    settings.simulation),
        pubs(node->get_clock())
        IF_GUI(,ui(this))
    {}

    void PMFS::Initialize()
    {
        Algorithm::Initialize();
        // A lot of the initialization is done inside of the map callback, rather than here. That is because we need to know the map beforehand

        pubs.markers.source_probability_markers = node->create_publisher<Marker>("probability_markers", 1);
        pubs.markers.hitProbabilityMarkers = node->create_publisher<Marker>("hitProbabilityMarkers", 1);
        pubs.markers.confidenceMarkers = node->create_publisher<Marker>("confidenceMarkers", 1);
        pubs.markers.windArrowMarkers = node->create_publisher<MarkerArray>("arrowMarkers", 1);

        pubs.markers.quadtreePublisher = node->create_publisher<MarkerArray>("quadtree", 1);

        pubs.gmrfWind.client = node->create_client<WindEstimation>("/WindEstimation");
        IF_GADEN( pubs.groundTruthWind.client = node->create_client<gaden_player::srv::WindPosition>("/wind_value") );

        IF_GUI
        (
            if(!settings.visualization.headless)
                ui.run();
        );

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
        PMFSLib::GetHitProbabilitySettings(*this, settings.hitProbability);
        PMFSLib::GetSimulationSettings(*this, settings.simulation);
        
        // number of cells in each direction that we add to the open move set in each step
        settings.movement.openMoveSetExpasion = getParam<int>("openMoveSetExpasion", 5);
        settings.movement.explorationProbability = getParam<double>("explorationProbability", 0.1);
        settings.movement.initialExplorationMoves = getParam<int>("initialExplorationMoves", 5);

        settings.visualization.markers_height = getParam<double>("markers_height", 0);
        IF_GUI(settings.visualization.headless = getParam<bool>("headless", false));

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
        occupancy.resize(gridMetadata.height * gridMetadata.width);

        visibilityMap.range = std::max(settings.movement.openMoveSetExpasion, settings.hitProbability.localEstimationWindowSize);

        PMFSLib::initializeMap(*this, 
            Grid<HitProbability>(hitProbability, occupancy, gridMetadata),
            simulations, visibilityMap);


        // set all variables to the prior probability
        for(HitProbability& h : hitProbability)
            h.setProbability(settings.hitProbability.prior);

        for (double& p : sourceProbability)
            p = 1.0 / gridMetadata.numFreeCells;

        // the wind estimation stuff requires spinning, so it must be done through the function queue
        functionQueue.submit([this]()
        {
            Grid<Vector2> windGrid (estimatedWindVectors, occupancy, gridMetadata);
            PMFSLib::initializeWindPredictions(*this, windGrid,
                pubs.gmrfWind.request IF_GADEN(, pubs.groundTruthWind.request));
            PMFSLib::estimateWind(settings.simulation.useWindGroundTruth, windGrid, node, pubs.gmrfWind IF_GADEN(, pubs.groundTruthWind));
            stateMachine.forceSetState(stopAndMeasureState.get());
        });
    }

} // namespace GSL