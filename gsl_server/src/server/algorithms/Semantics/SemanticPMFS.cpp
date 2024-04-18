#include <gsl_server/algorithms/Semantics/SemanticPMFS/SemanticPMFS.hpp>
#include <gsl_server/algorithms/PMFS/PMFSLib.hpp>

namespace GSL
{

    SemanticPMFS::SemanticPMFS(std::shared_ptr<rclcpp::Node> _node)
        : Algorithm(_node), 
        simulations(Grid<HitProbability>(hitProbability, simulationOccupancy, gridMetadata),
                    Grid<double>(sourceProbability, simulationOccupancy, gridMetadata),
                    Grid<Vector2>(estimatedWindVectors, simulationOccupancy, gridMetadata), settings.simulation)
    {}

    void SemanticPMFS::Initialize()
    {
        Algorithm::Initialize();
        //if(semanticType == SemanticType::ClassMap)
        //    semantics = std::make_unique<ClassMap>();


        waitForGasState = std::make_unique<WaitForGasState>(this);
        waitForMapState = std::make_unique<WaitForMapState>(this);
        waitForMapState->shouldWaitForGas = false;

        stopAndMeasureState = std::make_unique<StopAndMeasureState>(this);
        //movingState = std::make_unique<MovingStatePMFS>(this);
        stateMachine.forceSetState(waitForMapState.get());
    }

    void SemanticPMFS::OnUpdate()
    {
        Algorithm::OnUpdate();
        semantics->OnUpdate();
    }


    void SemanticPMFS::declareParameters()
    {
        Algorithm::declareParameters();
        //...
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
         
        visibilityMap.range = settings.hitProbability.localEstimationWindowSize;
        //visibilityMap.range = std::max(settings.movement.openMoveSetExpasion, settings.hitProbability.localEstimationWindowSize);

        PMFSLib::initializeMap(*this, 
            Grid<HitProbability>(hitProbability, simulationOccupancy, gridMetadata),
            simulations, visibilityMap);

        // set all variables to the prior probability
        for(HitProbability& h : hitProbability)
            h.setProbability(settings.hitProbability.prior);

        for (double& p : sourceProbability)
            p = 1.0 / gridMetadata.numFreeCells;


        // the wind estimation stuff requires spinning, so it must be done through the function queue
        functionQueue.submit([this]()
        {
            Grid<Vector2> windGrid (estimatedWindVectors, simulationOccupancy, gridMetadata);
            PMFSLib::initializeWindPredictions(*this, windGrid,
                pubs.gmrfWind.request, pubs.groundTruthWind.request);
            PMFSLib::estimateWind(settings.simulation.useWindGroundTruth, windGrid, node, pubs.gmrfWind, pubs.groundTruthWind);
            stateMachine.forceSetState(stopAndMeasureState.get());
        });

        //TODO initialize semantics object
    }

    void SemanticPMFS::processGasAndWindMeasurements(double concentration, double wind_speed, double wind_direction)
    {

    }

}