#pragma once
#include <gsl_server/algorithms/Semantics/Semantics/Common/ClassDistribution.hpp>
#include <map>
#include <vector>

namespace GSL
{
    class ClassMap
    {
    public:
        std::map<std::string, float> sourceProbByClass;
        std::vector<ClassDistribution> classDistributions;

        void computeSourceProbability(ClassDistribution& distribution, double& retValue);
        void parseOntology(const std::string& path, const std::string& targetGas);
        double getSourceProbByClass(const std::string& _class);
        void updateObjectProbabilities(size_t index, const std::vector<std::pair<std::string, float>>& scores);
    };
} // namespace GSL