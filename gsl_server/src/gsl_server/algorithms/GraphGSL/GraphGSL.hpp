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
        // Vector2 windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg) override;

        void UpdateWindMaps();
        void Visualize();

    private:
        void EvaluateRoomProbabilities();
        float EvaluateSourceProbabilitiesInRooms(std::vector<std::shared_ptr<RoomNode>> roomNodes); // returns the lowest residual found in the fine level
        std::pair<float, float> ResidualSingleSimulation(const Graph_internal::CompleteMap& simMap);
        long double ProbFromResidual(long double residual);
        void UpdateExpectedValue();
        void UpdateInformationGain();
        
    private:
        Graph graph;
        Graph_internal::SimulationSystem simulationSystem;
        float likelihoodSigma = 1e-3;

        struct PredictedMap
        {
            std::shared_ptr<Graph_internal::CompleteMap> map;
        };
        std::map<CellIdentifier, PredictedMap> expectedGasMaps;

#define ENABLE_NAIVE_EVALUATION 1
#if ENABLE_NAIVE_EVALUATION
        void EvaluateSourceProbabilitiesInAllRooms();
        void EvaluateProbabilitiesNaive();
        Graph_internal::NaiveSimulationSystem naiveSimulationSystem;
        std::shared_ptr<RoomNode> naiveEntireMap;
        std::vector<Graph_internal::CompleteMap> naiveCompleteMaps;
        rclcpp::Publisher<MarkerArray>::SharedPtr naiveMapsPub;
        size_t naiveSimulationIndex;
#endif

        struct NodeResult
        {
            std::shared_ptr<PlaceNode> node;
            float residual;
            float confidenceSum;
        };
        void GetNodeResidual(std::shared_ptr<PlaceNode> sourceNode, std::vector<NodeResult>& nodeResiduals, std::mutex& mtx);
        void CalculateNodeProbabilities(const std::vector<NodeResult>& nodeResiduals);

        struct Pubs
        {
            rclcpp::Publisher<MarkerArray>::SharedPtr graphPub;
            rclcpp::Publisher<MarkerArray>::SharedPtr occupancyPub;
            rclcpp::Publisher<MarkerArray>::SharedPtr windPub;
            rclcpp::Publisher<MarkerArray>::SharedPtr simGasMapsPub;
            rclcpp::Publisher<MarkerArray>::SharedPtr measuredGasMapsPub;
            rclcpp::Publisher<MarkerArray>::SharedPtr quadtreePub;
            rclcpp::Publisher<MarkerArray>::SharedPtr sourceProbPub;
            rclcpp::Publisher<MarkerArray>::SharedPtr infoGainPub;
        } pubs;

        struct SimulationViz
        {
            std::shared_ptr<PlaceNode> selectedNode;
            size_t simulationIndex;
        } simulationViz;

        Utils::Time::Countdown visualizationCD;

        Vector2 expectedValue;
        Utils::CovarianceMatrix cov;
        float expectedValueProportion = 1.0;
#if USE_GUI
        friend class GraphUI;
        GraphUI gui;
#endif
    };
} // namespace GSL