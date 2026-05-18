#pragma once
#include "Graph.hpp"
#include "GraphUI.hpp"
#include "NaiveSimulationSystem.hpp"
#include "SimulationSystem.hpp"
#include "gsl_server/algorithms/Common/Utils/Time.hpp"
#include <gsl_server/algorithms/Common/Algorithm.hpp>

namespace GSL
{
    class GraphGSL : public Algorithm
    {
    public:
        GraphGSL(std::shared_ptr<rclcpp::Node> _node);
        void Initialize() override;
        void OnUpdate() override;

        void processGasAndWindMeasurements(double concentration, double windSpeed, double windDirection) override; // called from StopAndMeasure once we have enough data for this position
        Vector2 windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg) override;
        void Visualize();

    private:
        void EvaluateSourceProbabilities();

    private:
        Graph graph;
        gmrfw::CGMRF_map::Parameters gmrfParams;
        Graph_internal::SimulationSystem simulationSystem;

#define ENABLE_NAIVE_EVALUATION 1
#if ENABLE_NAIVE_EVALUATION
        void EvaluateSourceProbabilitiesNaive();
        Graph_internal::NaiveSimulationSystem naiveSimulationSystem;
        std::shared_ptr<RoomNode> entireMap;
#endif
        struct Pubs
        {
            rclcpp::Publisher<MarkerArray>::SharedPtr graphPub;
            rclcpp::Publisher<MarkerArray>::SharedPtr occupancyPub;
            rclcpp::Publisher<MarkerArray>::SharedPtr windPub;
            rclcpp::Publisher<MarkerArray>::SharedPtr simGasMapsPub;
            rclcpp::Publisher<MarkerArray>::SharedPtr measuredGasMapsPub;
            rclcpp::Publisher<MarkerArray>::SharedPtr quadtreePub;
        } pubs;

        struct SimulationViz
        {
            std::shared_ptr<PlaceNode> selectedNode;
            size_t simulationIndex;
        } simulationViz;

        Utils::Time::Countdown visualizationCD;
        bool drawGraph = true;

#if USE_GUI
        friend class GraphUI;
        GraphUI gui;
#endif
    };
} // namespace GSL