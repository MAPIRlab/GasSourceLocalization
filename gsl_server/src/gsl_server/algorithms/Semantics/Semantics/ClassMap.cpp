#include <filesystem>
#include <gsl_server/algorithms/Semantics/Semantics/ClassMap.hpp>
#include <gsl_server/core/Macros.hpp>
#include <yaml-cpp/yaml.h>
#include <gsl_server/algorithms/Common/Utils/Math.hpp>

namespace GSL
{
    void ClassMap::parseOntology(const std::string& path, const std::string& targetGas)
    {
        if (!std::filesystem::exists(path))
        {
            GSL_ERROR("Could not open ontology file: {}", path);
            CLOSE_PROGRAM;
        }

        std::vector<std::string> class_list;
        YAML::Node gases = YAML::LoadFile(path)["Gases"];
        for (const YAML::Node& gas : gases)
        {
            if (gas["gas"].as<std::string>() == targetGas)
            {
                YAML::Node probs = gas["probs"];
                for (YAML::const_iterator it = probs.begin(); it != probs.end(); ++it)
                {
                    sourceProbByClass.insert({it->first.as<std::string>(), it->second.as<float>()});
                    class_list.push_back(it->first.as<std::string>());
                }
            }
        }

        GSL_INFO("Source probability by semantic class:");
        for (auto& kv : sourceProbByClass)
        {
            GSL_INFO("{}: {}", kv.first, kv.second);
        }

        for (ClassDistribution& dist : classDistributions)
        {
            dist.Initialize(class_list);
        }
    }

    void ClassMap::computeSourceProbability(ClassDistribution& distribution, double& retValue)
    {
        for (auto& [_class, classProb] : distribution)
        {
            retValue += classProb * getSourceProbByClass(_class);
        }
    }

    double ClassMap::getSourceProbByClass(const std::string& _class)
    {
        return sourceProbByClass[_class];
    }

    void ClassMap::updateObjectProbabilities(size_t index, const std::vector<std::pair<std::string, float>>& scores)
    {
        for (const auto& pair : scores)
        {
            std::string _class = pair.first;
            float score = pair.second;
            float previous = classDistributions[index].ProbabilityOf(_class);
            float prob = std::min(0.95f, score * previous); // clamp at arbitrary high value to avoid numerical issues as more observations pile up
            classDistributions[index].SetProbOf(_class, prob);
        }

        classDistributions[index].Normalize();
    }

    void ClassMap::FromMsg(size_t index, const std::vector<vision_msgs::msg::ObjectHypothesis>& msg)
    {
        classDistributions[index].Clear();
        for (const auto& hyp : msg)
        {
            std::string name = hyp.class_id;
            filterClassID(name);
            float previous = classDistributions[index].ProbabilityOf(name);
            classDistributions[index].SetProbOf(name, hyp.score + previous);
        }
        GSL_ASSERT_MSG(Utils::approx(classDistributions[index].TotalProb(), 1), "Class distribution is not normalized after updating from msg");
    }

    void ClassMap::filterClassID(std::string& id)
    {
        if (!sourceProbByClass.contains(id))
            id = ClassMap::otherClassName;
    }


} // namespace GSL