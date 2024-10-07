#include <gsl_server/algorithms/PMFS/PMFS.hpp>
#include <gsl_server/algorithms/PMFS/PMFSViz.hpp>
#include <gsl_server/algorithms/PMFS/PMFSLib.hpp>
#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <angles/angles.h>

//Initialization
namespace GSL
{
    using WindEstimation = gmrf_wind_mapping::srv::WindEstimation;
    PMFS::PMFS(std::shared_ptr<rclcpp::Node> _node) : Algorithm(_node),
        simulations(Grid2D<HitProbability>(hitProbability, occupancy, gridMetadata),
                    Grid2D<double>(sourceProbability, occupancy, gridMetadata),
                    Grid2D<Vector2>(estimatedWindVectors, occupancy, gridMetadata),
                    settings.simulation),
        pubs(node->get_clock())
        IF_GUI(, ui(this))
    {}

// A lot of the initialization is done inside of the map callback, rather than here. That is because we need to know the map beforehand
    void PMFS::Initialize()
    {
        Algorithm::Initialize();
        PMFSLib::InitializePublishers(pubs, node);
        anemometer_pub_ = node->create_publisher<olfaction_msgs::msg::Anemometer>("/PioneerP3DX/Anemometer/WindSensor_reading", 1);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);

        IF_GUI
        (
            if (!settings.visualization.headless)
            ui.run();
        );

        iterationsCounter = 0;

        waitForGasState = std::make_unique<WaitForGasState>(this);
        waitForMapState = std::make_unique<WaitForMapState>(this);
        waitForMapState->shouldWaitForGas = false;

        stopAndMeasureState = std::make_unique<StopAndMeasureState>(this);
        movingState = std::make_unique<MovingStatePMFS>(this);
        stateMachine.forceSetState(waitForMapState.get());
        // lanzar map server con el mapa que corresponda
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
        PMFSLib::InitMetadata(gridMetadata, map, scale);

        hitProbability.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y);
        sourceProbability.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y);
        occupancy.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y);

        visibilityMap.emplace(gridMetadata.dimensions.y, gridMetadata.dimensions.x, std::max(settings.movement.openMoveSetExpasion, settings.hitProbability.localEstimationWindowSize));

        PMFSLib::InitializeMap(*this,
                               Grid2D<HitProbability>(hitProbability, occupancy, gridMetadata),
                               simulations, *visibilityMap);


        // set all variables to the prior probability
        for (HitProbability& h : hitProbability)
            h.setProbability(settings.hitProbability.prior);

        for (double& p : sourceProbability)
            p = 1.0 / gridMetadata.numFreeCells;

        // the wind estimation stuff requires spinning, so it must be done through the function queue
        functionQueue.submit([this]()
        {
            Grid2D<Vector2> windGrid (estimatedWindVectors, occupancy, gridMetadata);
            PMFSLib::InitializeWindPredictions(*this, windGrid,
                                               pubs.gmrfWind.request IF_GADEN(, pubs.groundTruthWind.request));
            PMFSLib::EstimateWind(settings.simulation.useWindGroundTruth, windGrid, node, pubs.gmrfWind IF_GADEN(, pubs.groundTruthWind));
            stateMachine.forceSetState(stopAndMeasureState.get());
        });
    }

} // namespace GSL

// Core
namespace GSL
{
    using HashSet = std::unordered_set<Vector2Int>;

    void PMFS::OnUpdate()
    {
        if (!paused)
            Algorithm::OnUpdate();

        // Run anything that was submitted to main thread from the UI or a callback
        functionQueue.run();

        // Update visualization
        dynamic_cast<MovingStatePMFS*>(movingState.get())->publishMarkers();
        PMFSViz::ShowHitProb(Grid2D<HitProbability>(hitProbability, occupancy, gridMetadata), settings.visualization, pubs);
        PMFSViz::ShowSourceProb(Grid2D<double>(sourceProbability, occupancy, gridMetadata), settings.visualization, pubs);
    }

    void PMFS::processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection)
    {
        static int number_of_updates = 0;

        //Update the gas presence map
        // ------------------------------
        Grid2D<HitProbability> grid(hitProbability, occupancy, gridMetadata);
        if (concentration > thresholdGas)
        {
            // Gas & wind
            PMFSLib::EstimateHitProbabilities(grid, *visibilityMap, settings.hitProbability, true, windDirection, windSpeed,
                                              gridMetadata.coordinatesToIndices(currentRobotPose.pose.pose));
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "GAS HIT");
        }
        else
        {
            // Nothing
            PMFSLib::EstimateHitProbabilities(grid, *visibilityMap, settings.hitProbability, false, windDirection, windSpeed,
                                              gridMetadata.coordinatesToIndices(currentRobotPose.pose.pose));
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "NOTHING ");
        }

    // SEPARAR EN OTRA FUNCION (GENERAR PROBABILIDADES)
        //Update the wind estimations
        // ------------------------------
        PMFSLib::EstimateWind(settings.simulation.useWindGroundTruth,
                              Grid2D<Vector2>(estimatedWindVectors, occupancy, gridMetadata),
                              node,
                              pubs.gmrfWind
                              IF_GADEN(, pubs.groundTruthWind));

                simulations.updateSourceProbability(settings.simulation.refineFraction);

        //Visualization
        PMFSViz::ShowHitProb(Grid2D<HitProbability>(hitProbability, occupancy, gridMetadata), settings.visualization, pubs);
        PMFSViz::ShowSourceProb(Grid2D<double>(sourceProbability, occupancy, gridMetadata), settings.visualization, pubs);
        PMFSViz::PlotWindVectors(Grid2D<Vector2>(estimatedWindVectors, occupancy, gridMetadata), settings.visualization, pubs);
    }

    void PMFS::processGasAndWindMeasurements(double x, double y, double concentration, double windSpeed, double windDirection)
    {
        // Update the gas presence map
        // ------------------------------
        Grid2D<HitProbability> grid(hitProbability, occupancy, gridMetadata);
        if (concentration > thresholdGas)
        {
            // Gas & wind
            PMFSLib::EstimateHitProbabilities(grid, *visibilityMap, settings.hitProbability, true, windDirection, windSpeed,
                                              gridMetadata.coordinatesToIndices(currentRobotPose.pose.pose));
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "GAS HIT");
        }
        else
        {
            // Nothing
            PMFSLib::EstimateHitProbabilities(grid, *visibilityMap, settings.hitProbability, false, windDirection, windSpeed,
                                              gridMetadata.coordinatesToIndices(currentRobotPose.pose.pose));
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "NOTHING ");
        }
    }

    void PMFS::updateSourceProbability()
    {
        //Update the wind estimations (request data to GMRF)
        // ------------------------------
        PMFSLib::EstimateWind(settings.simulation.useWindGroundTruth,
                              Grid2D<Vector2>(estimatedWindVectors, occupancy, gridMetadata),
                              node,
                              pubs.gmrfWind
                              IF_GADEN(, pubs.groundTruthWind));

        simulations.updateSourceProbability(settings.simulation.refineFraction);

        //Visualization
        PMFSViz::ShowHitProb(Grid2D<HitProbability>(hitProbability, occupancy, gridMetadata), settings.visualization, pubs);
        PMFSViz::ShowSourceProb(Grid2D<double>(sourceProbability, occupancy, gridMetadata), settings.visualization, pubs);
        PMFSViz::PlotWindVectors(Grid2D<Vector2>(estimatedWindVectors, occupancy, gridMetadata), settings.visualization, pubs);
    }

    void PMFS::publishAnemometer(double x, double y, double windSpeed, double windDirection)
    {
        // 1. Set TF map->anemometer to current sampling location        
        geometry_msgs::msg::TransformStamped transform_stamped;

        // Set the frame IDs
        transform_stamped.header.stamp = node->now();
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "anemometer";

        // Set the translation
        transform_stamped.transform.translation.x = x;
        transform_stamped.transform.translation.y = y;
        transform_stamped.transform.translation.z = 0.0;

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform_stamped);

        // 2. Publish anemometer data over topic
        olfaction_msgs::msg::Anemometer msg;
        msg.header.frame_id = "anemometer";
        msg.wind_speed = windSpeed;
        msg.wind_direction = windDirection;
        anemometer_pub_->publish(msg);
    }

    float PMFS::gasCallback(olfaction_msgs::msg::GasSensor::SharedPtr msg)
    {
        float ppm = Algorithm::gasCallback(msg);
        IF_GUI(ui.addConcentrationReading(ppm));
        return ppm;
    }


} // namespace GSL