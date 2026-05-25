#include <gsl_server/algorithms/Common/States/WaitForMapState.hpp>
#include <gsl_server/algorithms/Common/Algorithm.hpp>

namespace GSL
{

    void WaitForMapState::OnEnterState(State* previous)
    {
        GSL_TRACE("Entering WaitForMap");
        GSL_TRACE("Need occupancy map and costmap");
        using namespace std::placeholders;

        mapSub = algorithm->rclnode->create_subscription<nav_msgs::msg::OccupancyGrid>(algorithm->getParam<std::string>("map_topic", "map"),
                 rclcpp::QoS(1).reliable().transient_local(),
                 std::bind(&WaitForMapState::mapCallback, this, _1));
        costmapSub = algorithm->rclnode->create_subscription<nav_msgs::msg::OccupancyGrid>(
                         algorithm->getParam<std::string>("costmap_topic", "global_costmap/costmap"), 1, std::bind(&WaitForMapState::costmapCallback, this, _1));
    }

    void WaitForMapState::OnExitState(State* previous)
    {
        algorithm->startTime = algorithm->rclnode->now();
    }

    void WaitForMapState::mapCallback(OccupancyGrid::SharedPtr msg)
    {
        GSL_TRACE("Got occupancy map");
        algorithm->onGetMap(msg);
        mapSub = nullptr;
        hasMap = true;
        if (hasCostmap)
            setNextState();
    }

    void WaitForMapState::costmapCallback(OccupancyGrid::SharedPtr msg)
    {
        GSL_TRACE("Got cost map");
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

#if USE_GUI
#include "imgui.h"
void GSL::WaitForMapState::RenderUI()
{
    ImGui::Text("WaitForMapState");
}
#endif