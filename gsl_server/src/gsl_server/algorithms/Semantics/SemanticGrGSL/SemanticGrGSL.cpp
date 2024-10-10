#include "SemanticGrGSL.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"
#include "gsl_server/algorithms/Common/Utils/RosUtils.hpp"
#include "gsl_server/algorithms/Semantics/Semantics/Common/SemanticsType.hpp"
#include <gsl_server/algorithms/GrGSL/GrGSLLib.hpp>

namespace GSL
{
    using namespace GrGSL_internal;
    void SemanticGrGSL::OnUpdate()
    {
        Algorithm::OnUpdate();
        functionQueue.run();

        if (semantics) // TODO allow this to run slower that the update loop? kinda messes up the callback-based one
        {
            semantics->OnUpdate();
            updateSourceFromSemantics();
        }

        GrGSLLib::VisualizeMarkers(
            Grid2D<double>(combinedSourceProbability, simulationOccupancy, gridMetadata),
            markers,
            node);
    }

    void SemanticGrGSL::Initialize()
    {
        Algorithm::Initialize();

        markers.probabilityMarkers = node->create_publisher<Marker>("probabilityMarkers", 10);
        markers.estimationMarkers = node->create_publisher<Marker>("estimationMarkers", 10);

        exploredCells = 0;

        waitForMapState = std::make_unique<WaitForMapState>(this);
        waitForGasState = std::make_unique<WaitForGasState>(this);
        stopAndMeasureState = std::make_unique<StopAndMeasureState>(this);
        // TODO movingState = std::make_unique<MovingStateGrGSL>(this);
        stateMachine.forceSetState(waitForMapState.get());
    }

    void SemanticGrGSL::declareParameters()
    {
        Algorithm::declareParameters();
        GrGSLLib::GetSettings(node, settings, markers);
    }

    void SemanticGrGSL::onGetMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        Algorithm::onGetMap(msg);
        GrGSLLib::initMetadata(gridMetadata, map, Utils::getParam(node, "scale", 20));
        cells.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y);
        navigationOccupancy.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y);
        combinedSourceProbability.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y);
        simulationOccupancy = Utils::parseMapImage(getParam<std::string>("wallsOccupancyFile", "?"), gridMetadata);

        GridUtils::reduceOccupancyMap(map.data, map.info.width, navigationOccupancy, gridMetadata);
        GrGSLLib::initializeMap(*this,
                                Grid2D<Cell>(cells, simulationOccupancy, gridMetadata));
        positionOfLastHit = {currentRobotPose.pose.pose.position.x, currentRobotPose.pose.pose.position.y};

        // SEMANTICS
        //----------------------
        std::string semanticsTypeParam = node->declare_parameter<std::string>("semanticsType", "ClassMap2D");
        SemanticsType semanticsType = ParseSemanticsType(semanticsTypeParam);

        if (semanticsType == SemanticsType::ClassMap2D)
            createClassMap2D();
        else if (semanticsType == SemanticsType::ClassMapVoxeland)
            createClassMapVoxeland();
    }

    void SemanticGrGSL::processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection)
    {
        bool gasHit = concentration > thresholdGas;
        bool significantWind = windSpeed > thresholdWind;

        if (gasHit)
            positionOfLastHit = Vector2(currentRobotPose.pose.pose.position.x, currentRobotPose.pose.pose.position.y);

        if (gasHit && significantWind)
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "GAS HIT");
        else if (gasHit)
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "GAS BUT NO WIND");
        else if (significantWind)
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "ONLY WIND");
        else
            GSL_INFO_COLOR(fmt::terminal_color::yellow, "NOTHING");

        // GrGSLLib::estimateProbabilitiesfromGasAndWind(
        //     Grid2D<Cell>(cells, simulationOccupancy, gridMetadata),
        //     settings,
        //     gasHit,
        //     gasHit ? significantWind : true,
        //     windDirection,
        //     positionOfLastHit,
        //     gridMetadata.coordinatesToIndices(currentRobotPose.pose.pose));

        // dynamic_cast<MovingStateGrGSL*>(movingState.get())->chooseGoalAndMove();
    }

    void SemanticGrGSL::updateSourceFromSemantics()
    {
        // wait until we have received the map and initialized everything
        if (!semantics)
            return;

        std::vector<double> sourceProbSemantics = semantics->GetSourceProbability();
#pragma omp parallel for
        for (size_t i = 0; i < sourceProbSemantics.size(); i++)
        {
            combinedSourceProbability[i] = cells[i].sourceProb * sourceProbSemantics[i];
        }
        Utils::NormalizeDistribution(combinedSourceProbability, simulationOccupancy);
    }

} // namespace GSL
