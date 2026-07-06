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

        void UpdateInfoGain();
        void ResetVariances();
        void UpdateExpectedGasRoomLevel(std::shared_ptr<PlaceNode> sourceNode,
                                        const std::vector<double>& scales,
                                        const std::deque<Graph_internal::CompleteMap>& simulations);
        void UpdateExpectedGasGeometricLevel(std::mutex& mtx, CellIdentifier id, float scale, const Graph_internal::CompleteMap& map);
        void AssignAABBGasMapToCell(CellIdentifier aabbID, CellIdentifier cellID);

        void UpdateExpectedVariance();

        MarkerArray VisualizeInfoGain();
        void AutoSetMaxInfoGain();

    private:
        double CalculateExplorationValue(const struct CellIdentifier& c);

    private:
        class GraphGSL* gsl;
        float sigmaDist = 1.f;
        std::map<struct CellIdentifier, float> explorationValue;
        std::map<struct CellIdentifier, float> doorwayValue;
        std::map<std::shared_ptr<class PlaceNode>, std::vector<float>> finalInfoValue;
        std::map<std::shared_ptr<class PlaceNode>, std::vector<Utils::RunningVariance>> expectedGasVariances;

        struct PredictedMap
        {
            std::shared_ptr<Graph_internal::CompleteMap> map;
        };
        std::map<CellIdentifier, PredictedMap> expectedGasMaps;
    
        friend class GraphUI;
    };
} // namespace GSL