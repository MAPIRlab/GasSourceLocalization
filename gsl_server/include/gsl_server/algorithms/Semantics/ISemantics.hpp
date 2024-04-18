#pragma once 
#include <vector>

namespace GSL
{
    class ISemantics
    {
    public:
        virtual std::vector<double> GetSourceProbability() = 0;
        virtual void OnUpdate() = 0;
    };
}