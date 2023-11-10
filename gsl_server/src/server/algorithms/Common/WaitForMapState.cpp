#include <gsl_server/algorithms/Common/WaitForMapState.h>
#include <gsl_server/algorithms/Algorithm.h>

namespace GSL
{

    void WaitForMapState::OnEnterState(State* previous)
    {
        GSL_TRACE("Entering WaitForMap");
        using namespace std::placeholders;

        map_sub = algorithm->node->create_subscription<nav_msgs::msg::OccupancyGrid>(algorithm->getParam<std::string>("map_topic", "map"),
                                                                                     rclcpp::QoS(1).reliable().transient_local(),
                                                                                     std::bind(&WaitForMapState::mapCallback, this, _1));
        costmap_sub = algorithm->node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            algorithm->getParam<std::string>("costmap_topic", "global_costmap/costmap"), 1, std::bind(&WaitForMapState::costmapCallback, this, _1));
    }

    void WaitForMapState::OnExitState(State* previous)
    {
    }

    void WaitForMapState::mapCallback(OccupancyGrid::SharedPtr msg)
    {
        algorithm->onGetMap(msg);
        map_sub = nullptr;
        hasMap = true;
        if (hasCostmap)
            algorithm->stateMachine.forceSetState(algorithm->stopAndMeasureState.get());
    }

    void WaitForMapState::costmapCallback(OccupancyGrid::SharedPtr msg)
    {
        algorithm->onGetCostMap(msg);
        costmap_sub = nullptr;
        hasCostmap = true;
        if (hasMap)
            algorithm->stateMachine.forceSetState(algorithm->stopAndMeasureState.get());
    }
} // namespace GSL