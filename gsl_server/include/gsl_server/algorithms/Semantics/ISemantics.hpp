#pragma once 
#include <vector>
#include <gsl_server/core/Vectors.hpp>

namespace GSL
{
    class ISemantics
    {
    public:
        virtual void GetSourceProbabilityInPlace(std::vector<double>& sourceProb) = 0;
        virtual std::vector<double> GetSourceProbability() = 0;
        virtual double GetSourceProbability(const Vector3& point) = 0;
        virtual void OnUpdate() = 0;
    };
}