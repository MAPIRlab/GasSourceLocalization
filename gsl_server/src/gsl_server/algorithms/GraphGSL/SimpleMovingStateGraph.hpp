#pragma once
#include "gsl_server/algorithms/Common/States/ManualNavigation.hpp"

namespace GSL
{
    class SimpleMovingStateGraph : public ManualNavigationState
    {
    public:
        SimpleMovingStateGraph(Algorithm* alg);

    private:
        double CalculateExplorationValue(const struct CellIdentifier& c);

    private:
        class GraphGSL* gsl;
        
        float sigmaDist = 1.f;
        std::map<struct CellIdentifier, float> explorationValue;
        std::map<struct CellIdentifier, float> doorwayValue;
    };
} // namespace GSL