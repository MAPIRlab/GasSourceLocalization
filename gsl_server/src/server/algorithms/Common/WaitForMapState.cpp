#include <gsl_server/algorithms/Common/WaitForMapState.hpp>
#include <gsl_server/algorithms/Algorithm.hpp>

namespace GSL
{

    void WaitForMapState::OnEnterState(State* previous)
    {
        GSL_TRACE("Entering WaitForMap");
        using namespace std::placeholders;

        mapSub = algorithm->node->create_subscription<nav_msgs::msg::OccupancyGrid>(algorithm->getParam<std::string>("map_topic", "map"),
                 rclcpp::QoS(1).reliable().transient_local(),
                 std::bind(&WaitForMapState::mapCallback, this, _1));
        costmapSub = algorithm->node->create_subscription<nav_msgs::msg::OccupancyGrid>(
                         algorithm->getParam<std::string>("costmap_topic", "global_costmap/costmap"), 1, std::bind(&WaitForMapState::costmapCallback, this, _1));
    }

    void WaitForMapState::OnExitState(State* previous)
    {
        algorithm->startTime = algorithm->node->now();
    }

    void WaitForMapState::mapCallback(OccupancyGrid::SharedPtr msg)
    {
        algorithm->onGetMap(msg);
        mapSub = nullptr;
        hasMap = true;
        if (hasCostmap)
            setNextState();
    }

    void WaitForMapState::costmapCallback(OccupancyGrid::SharedPtr msg)
    {
        algorithm->onGetCostMap(msg);
        costmapSub = nullptr;
        hasCostmap = true;
        if (hasMap)
            setNextState();
    }

    void WaitForMapState::setNextState()
    {
        if (shouldWaitForGas)
            algorithm->stateMachine.forceSetState(algorithm->waitForGasState.get());
        else
            algorithm->stateMachine.forceSetState(algorithm->stopAndMeasureState.get());
    }
} // namespace GSL