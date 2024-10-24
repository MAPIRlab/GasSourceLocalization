#pragma once
#include <gsl_server/core/Vectors.hpp>
#include <vector>

namespace GSL
{
    class ISemantics
    {
    public:
        virtual ~ISemantics()
        {}
        virtual void OnUpdate() = 0;

        virtual void GetSourceProbabilityInPlace(std::vector<double>& sourceProb) = 0;
        virtual std::vector<double> GetSourceProbability() = 0;
        virtual double GetSourceProbabilityAt(const Vector3& point) = 0;
        
        virtual double GetEntropyAt(const Vector3& point) = 0;
        virtual std::vector<double> GetEntropy() = 0;
        virtual void GetEntropyInPlace(std::vector<double>& entropy) = 0;
        
        virtual std::string GetDebugInfo(const Vector3& point) = 0;
    };
} // namespace GSL