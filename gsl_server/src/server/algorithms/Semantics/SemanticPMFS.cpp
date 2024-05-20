#include <gsl_server/algorithms/Semantics/SemanticPMFS/SemanticPMFS.hpp>
#include <gsl_server/algorithms/PMFS/PMFSLib.hpp>
#include <gsl_server/algorithms/PMFS/PMFSViz.hpp>
#include <gsl_server/algorithms/Semantics/SemanticsType.hpp>
#include <gsl_server/algorithms/Semantics/ClassMap2D.hpp>
#include <magic_enum.hpp>
namespace GSL
{

    SemanticPMFS::SemanticPMFS(std::shared_ptr<rclcpp::Node> _node)
        : Algorithm(_node), 
        simulations(Grid<HitProbability>(hitProbability, simulationOccupancy, gridMetadata),
                    Grid<double>(sourceProbability, simulationOccupancy, gridMetadata),
                    Grid<Vector2>(estimatedWindVectors, simulationOccupancy, gridMetadata), settings.simulation),
        pubs(node->get_clock())
    {}

    void SemanticPMFS::Initialize()
    {
        Algorithm::Initialize();
        
        PMFSLib::InitializePublishers(pubs.pmfsPubs, node);

        waitForGasState = std::make_unique<WaitForGasState>(this);
        waitForMapState = std::make_unique<WaitForMapState>(this);
        waitForMapState->shouldWaitForGas = false;

        stopAndMeasureState = std::make_unique<StopAndMeasureState>(this);
        //TODO
        movingState = std::make_unique<MovingStateSemanticPMFS>(this);
        stateMachine.forceSetState(waitForMapState.get());


        // SEMANTICS 
        std::string semanticsTypeParam = node->declare_parameter<std::string>("semanticsType", "ClassMap2D");
        auto semanticsType = magic_enum::enum_cast<SemanticsType>(semanticsTypeParam, magic_enum::case_insensitive);
        if(!semanticsType.has_value())
        {
            constexpr auto names = magic_enum::enum_names<SemanticsType>();
            GSL_ERROR("{} is not a valid semantics type. Valid types are {}", semanticsTypeParam, fmt::join(names, ", "));
            GSL_ERROR("Closing");
            CLOSE_PROGRAM;
        }
        else if (semanticsType == SemanticsType::ClassMap2D)
            semantics = std::make_unique<ClassMap2D>(gridMetadata, simulationOccupancy, tf_buffer,
                       currentRobotPose);
    }

    void SemanticPMFS::OnUpdate()
    {
        Algorithm::OnUpdate();
        semantics->OnUpdate();
        PMFSViz::ShowHitProb(Grid<HitProbability>(hitProbability, simulationOccupancy, gridMetadata), settings.visualization, pubs.pmfsPubs);
        PMFSViz::ShowSourceProb(Grid<double>(sourceProbability, simulationOccupancy, gridMetadata), settings.visualization, pubs.pmfsPubs);
        functionQueue.run();
    }

    void SemanticPMFS::declareParameters()
    {
        Algorithm::declareParameters();
        PMFSLib::GetHitProbabilitySettings(*this, settings.hitProbability);
        PMFSLib::GetSimulationSettings(*this, settings.simulation);
        settings.visualization.markers_height = getParam<double>("markers_height", 0);
        // number of cells in each direction that we add to the open move set in each step
        settings.movement.openMoveSetExpasion = getParam<int>("openMoveSetExpasion", 5);
        settings.movement.explorationProbability = getParam<double>("explorationProbability", 0.1);
        settings.movement.initialExplorationMoves = getParam<int>("initialExplorationMoves", 5);
    }

    void SemanticPMFS::onGetMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        Algorithm::onGetMap(msg);

        int scale = getParam<int>("scale", 65); // scale for dynamic map reduction
        PMFSLib::initMetadata(gridMetadata, map, scale);

        hitProbability.resize(gridMetadata.height * gridMetadata.width);
        sourceProbability.resize(gridMetadata.height * gridMetadata.width);
        navigationOccupancy.resize(gridMetadata.height * gridMetadata.width);
        simulationOccupancy.resize(gridMetadata.height * gridMetadata.width);

        visibilityMap.emplace(gridMetadata.width, gridMetadata.height, std::max(settings.hitProbability.localEstimationWindowSize, settings.movement.openMoveSetExpasion));
        // visibilityMap.range = std::max(settings.movement.openMoveSetExpasion, settings.hitProbability.localEstimationWindowSize);

        PMFSLib::initializeMap(*this, Grid<HitProbability>(hitProbability, simulationOccupancy, gridMetadata), simulations, *visibilityMap);

        // set all variables to the prior probability
        for (HitProbability& h : hitProbability)
            h.setProbability(settings.hitProbability.prior);

        for (double& p : sourceProbability)
            p = 1.0 / gridMetadata.numFreeCells;

        // the wind estimation stuff requires spinning, so it must be done through the function queue
        functionQueue.submit([this]() {
            Grid<Vector2> windGrid(estimatedWindVectors, simulationOccupancy, gridMetadata);
            PMFSLib::initializeWindPredictions(*this, windGrid, pubs.pmfsPubs.gmrfWind.request IF_GADEN(,pubs.pmfsPubs.groundTruthWind.request));
            PMFSLib::estimateWind(settings.simulation.useWindGroundTruth, windGrid, node, pubs.pmfsPubs.gmrfWind IF_GADEN(,pubs.pmfsPubs.groundTruthWind));
            stateMachine.forceSetState(stopAndMeasureState.get());
        });

        // TODO initialize semantics object
    }

    void SemanticPMFS::processGasAndWindMeasurements(double concentration, double wind_speed, double wind_direction)
    {
        static int number_of_updates = 0;

        Grid<HitProbability> grid(hitProbability, simulationOccupancy, gridMetadata);
        if (concentration > thresholdGas)
        {
            // Gas & wind
            PMFSLib::estimateHitProbabilities(grid, *visibilityMap, settings.hitProbability, true, wind_direction, wind_speed,
                                              gridMetadata.coordinatesToIndex(currentRobotPose.pose.pose));
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "GAS HIT");
        }
        else
        {
            // Nothing
            PMFSLib::estimateHitProbabilities(grid, *visibilityMap, settings.hitProbability, false, wind_direction, wind_speed,
                                              gridMetadata.coordinatesToIndex(currentRobotPose.pose.pose));
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "NOTHING ");
        }

        PMFSLib::estimateWind(settings.simulation.useWindGroundTruth, 
                            Grid<Vector2>(estimatedWindVectors, simulationOccupancy, gridMetadata), 
                            node,
                            pubs.pmfsPubs.gmrfWind
                            IF_GADEN(, pubs.pmfsPubs.groundTruthWind));
        PMFSViz::PlotWindVectors(Grid<Vector2>(estimatedWindVectors, simulationOccupancy, gridMetadata), settings.visualization, pubs.pmfsPubs);

        number_of_updates++;

        //TODO
        if (number_of_updates >= settings.hitProbability.max_updates_per_stop)
        {
            number_of_updates = 0;
            bool timeToSimulate = iterationsCounter >= settings.movement.initialExplorationMoves &&
                                  iterationsCounter % settings.simulation.steps_between_source_updates == 0;
            if (timeToSimulate)
            {
                // simulations.compareRefineFractions();
                simulations.updateSourceProbability(settings.simulation.refineFraction);
            }
        
            auto movingStatePMFS = dynamic_cast<MovingStateSemanticPMFS*>(movingState.get());
            if (iterationsCounter > settings.movement.initialExplorationMoves)
                movingStatePMFS->currentMovement = MovingStateSemanticPMFS::MovementType::Search;
            else
                movingStatePMFS->currentMovement = MovingStateSemanticPMFS::MovementType::Exploration;
            movingStatePMFS->chooseGoalAndMove();
            movingStatePMFS->publishMarkers();
        }
        else
            stateMachine.forceResetState(stopAndMeasureState.get());

    }

} // namespace GSL