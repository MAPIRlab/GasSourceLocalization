#include <gsl_server/algorithms/Semantics/ClassMap.hpp>

namespace GSL
{
    ClassMap::ClassMap()
    {
        node = std::make_shared<rclcpp::Node>("class_map");
        //TODO create the class distribution vector
    }

    std::vector<double> ClassMap::GetSourceProbability()
    {
        std::vector<double> probs(classDistributions.size(), 0.0);

        for(int i = 0; i< classDistributions.size(); i++)
        {
            auto& classDistribution = classDistributions[i];
            for(auto& [_class, classProb] : classDistribution)
            {
                probs[i] += classProb * sourceProbByClass(_class);
            }
        }

        return probs;
    }

    void ClassMap::OnUpdate()
    {
        
    }
}