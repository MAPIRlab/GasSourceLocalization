#pragma once
#include <gsl_server/algorithms/Semantics/Semantics/Common/ClassDistribution.hpp>
#include <map>
#include <vector>
#include <vision_msgs/msg/object_hypothesis.hpp>

namespace GSL
{
    class ClassMap
    {
    public:
        static constexpr const char* otherClassName = "other";
        std::map<std::string, float> sourceProbByClass;
        std::vector<ClassDistribution> classDistributions;

        void computeSourceProbability(ClassDistribution& distribution, double& retValue);
        void parseOntology(const std::string& path, const std::string& targetGas);
        double getSourceProbByClass(const std::string& _class);
        void updateObjectProbabilities(size_t index, const std::vector<std::pair<std::string, float>>& scores);
        void FromMsg(size_t index, const std::vector<vision_msgs::msg::ObjectHypothesis>& msg);
        void filterClassID(std::string& id);
    };

} // namespace GSL