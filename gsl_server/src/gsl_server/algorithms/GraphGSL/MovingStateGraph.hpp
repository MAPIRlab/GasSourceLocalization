#pragma once
#include "gsl_server/algorithms/GraphGSL/Simulations.hpp"
#include <gsl_server/algorithms/Common/States/ManualNavigation.hpp>

namespace GSL
{
    class MovingStateGraph : public ManualNavigationState
    {
    public:
        MovingStateGraph(Algorithm* alg);
        
        void OnEnterState(State* previous) override;
        virtual void chooseGoalAndMove() override;

        void ResetVariances();
        void UpdateExpectedGasRoomLevel(std::shared_ptr<PlaceNode> sourceNode,
                                        float residual,
                                        const std::vector<double>& scales,
                                        const std::deque<Graph_internal::CompleteMap>& simulations);
        void UpdateExpectedGasGeometricLevel(std::mutex& mtx, CellIdentifier id, const Graph_internal::CompleteMap& map);
        void AssignAABBGasMapToCell(CellIdentifier aabbID, CellIdentifier cellID);

        void UpdateExpectedVariance();

        MarkerArray VisualizeInfoGain();
        void AutoSetMaxInfoGain();

    private:
        double CalculateExplorationValue(const struct CellIdentifier& c);

    private:
        class GraphGSL* gsl;
        std::map<struct CellIdentifier, float> explorationValue;
        std::map<std::shared_ptr<class PlaceNode>, std::vector<float>> finalInfoValue;
        std::map<std::shared_ptr<class PlaceNode>, std::vector<Utils::RunningVariance>> expectedGasVariances;

        struct PredictedMap
        {
            std::shared_ptr<Graph_internal::CompleteMap> map;
        };
        std::map<CellIdentifier, PredictedMap> expectedGasMaps;
    };
} // namespace GSL