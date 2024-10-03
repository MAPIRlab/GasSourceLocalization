#include "ClassMap.hpp"
#include "gsl_server/algorithms/Common/Utils/RosUtils.hpp"
#include "gsl_server/core/Logging.hpp"
#include "gsl_server/core/VectorsImpl/vmath_DDACustomVec.hpp"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <gsl_server/algorithms/Common/Utils/Collections.hpp>
#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <gsl_server/core/Macros.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

namespace GSL
{
    const char* UnknownRoomName = "unknown";

    void ClassMap::Initialize(rclcpp::Node::SharedPtr node, size_t size, MapToPixel indexToPixel)
    {
        classProbabilityZ.resize(size);
        roomOfObjects.resize(size, nullptr);
        std::string ontologyPath = Utils::getParam<std::string>(node, "ontologyPath", "?");
        std::string targetGas = Utils::getParam<std::string>(node, "targetGas", "?");
        parseOntology(ontologyPath, targetGas);

        std::string masksYAMLPath = Utils::getParam<std::string>(node, "masksYAMLPath", "?");
        std::string roomOntologyPath = Utils::getParam<std::string>(node, "roomOntologyPath", "?");
        parseRoomCategorization(masksYAMLPath, roomOntologyPath, indexToPixel);
    }

    void ClassMap::parseOntology(const std::string& path, const std::string& targetGas)
    {
        if (!std::filesystem::exists(path))
        {
            GSL_ERROR("Could not open ontology file: {}", path);
            CLOSE_PROGRAM;
        }

        // read p(o|s) from the ontology file
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

        // Normalize p(o|s)
        float sum = 0;
        for (const auto& [_class, prob] : sourceProbByClass)
            sum += prob;
        for (auto& [_class, prob] : sourceProbByClass)
            prob /= sum;


        //We're done!
        GSL_INFO("Source probability by semantic class (normalized):");
        for (auto& kv : sourceProbByClass)
            GSL_INFO("{}: {}", kv.first, kv.second);

        for (ClassDistribution& dist : classProbabilityZ)
            dist.Initialize(class_list);
    }

    void ClassMap::parseRoomCategorization(const std::string& masksYAMLPath, const std::string& ontologyPath,
                                           MapToPixel indexToPixel)
    {
        if (!std::filesystem::exists(masksYAMLPath))
        {
            GSL_INFO("Not using any room categorization info, could not open room masks file: {}", masksYAMLPath);
            setUpNoRooms();
            return;
        }

        if (!std::filesystem::exists(ontologyPath))
        {
            GSL_WARN("Not using any room categorization info, despite having room masks. Could not open room ontology file: {}", ontologyPath);
            setUpNoRooms();
            return;
        }

        // Create the rooms and store in them the probability of object classes for cells in the room -- p(o_i | room)
        {
            YAML::Node ontology = YAML::LoadFile(ontologyPath);
            size_t numClasses = ontology["number_classes"].as<size_t>();
            YAML::Node roomCategories = ontology["room_categories"];

            for (YAML::const_iterator roomIter = roomCategories.begin(); roomIter != roomCategories.end(); ++roomIter)
            {
                std::string roomName = roomIter->first.as<std::string>();
                Room& room = rooms[roomName];
                room.name = roomName;

                for (YAML::const_iterator classIter = roomIter->second.begin(); classIter != roomIter->second.end(); ++classIter)
                {
                    std::string className = classIter->first.as<std::string>();
                    room.SetProbOf(className, classIter->second.as<float>());
                }
                room.finalize(numClasses, *this);
            }

            // initialize all cells to the unknown room
            Room& unknown = createUnknownRoom(numClasses);
            for (size_t i = 0; i < roomOfObjects.size(); i++)
                roomOfObjects[i] = &unknown;
        }

        // Read the masks and register which room each cell is in
        YAML::Node masksList = YAML::LoadFile(masksYAMLPath)["categories"];
        for (const YAML::Node& yamlRoom : masksList)
        {
            std::string roomName = yamlRoom["name"].as<std::string>();

            Room* room;
            if (rooms.contains(roomName))
                room = &rooms.at(roomName);
            else
                room = &rooms.at(UnknownRoomName);

            std::filesystem::path maskPath(yamlRoom["mask"].as<std::string>());
            // interpret relative paths as relative to the YAML file location, not the working directory
            if (maskPath.is_relative())
                maskPath = std::filesystem::path(masksYAMLPath).parent_path() / maskPath;
            readMask(maskPath, *room, indexToPixel);
        }
    }

    void ClassMap::readMask(const std::filesystem::path& maskPath, const Room& room, MapToPixel indexToPixel)
    {
        cv::Mat maskImage = cv::imread(maskPath.c_str(), cv::IMREAD_GRAYSCALE);
        for (size_t index = 0; index < roomOfObjects.size(); index++)
        {
            Vector2Int indices2D = indexToPixel(index);
            if (maskImage.at<uint8_t>(maskImage.size[0] - 1 - indices2D.y, indices2D.x) != 0)
                roomOfObjects[index] = &room;
        }
    }

    double ClassMap::computeSourceProbability(size_t index)
    {
        const Room* room = roomOfObjects[index];
        double retValue = 0;
        for (auto& [_class, prob] : classProbabilityZ[index])
        {
            // The total probability for the object class is the accumulated probability p(o|Z) times the probability due to room classification (
            // p(o|room) )
            std::string className = filterClassID(_class);
            float totalClassProb = prob * room->GetClassProb(className) / room->GetPrior(className);
            retValue += totalClassProb * getSourceProbByClass(_class) / room->GetPrior(className); //TODO this prior here seems weird, but it appears in the formulation. Check it.
        }
        return retValue;
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
            float previous = classProbabilityZ[index].ProbabilityOf(_class);
            float prob = std::min(0.95f, score * previous); // clamp at arbitrary high value to avoid numerical issues as more observations pile up
            classProbabilityZ[index].SetProbOf(_class, prob);
        }

        classProbabilityZ[index].Normalize();
    }

    void ClassMap::FromMsg(size_t index, const std::vector<vision_msgs::msg::ObjectHypothesis>& msg)
    {
        classProbabilityZ[index].Clear(); // set all classes to 0
        for (const auto& hyp : msg)
        {
            // all classes not present in the ontology get bundled into "other"
            // that means the probability of "other" is the sum of all their probs
            std::string name = filterClassID(hyp.class_id);
            float previous = classProbabilityZ[index].ProbabilityOf(name);
            classProbabilityZ[index].SetProbOf(name, hyp.score + previous);
        }
        GSL_ASSERT_MSG(Utils::approx(classProbabilityZ[index].TotalProb(), 1), "Class distribution is not normalized after updating from msg");
    }

    std::string ClassMap::filterClassID(const std::string& id)
    {
        if (!sourceProbByClass.contains(id))
            return ClassMap::otherClassName;
        return id;
    }

    ClassMap::Room& ClassMap::createUnknownRoom(size_t numberOfClasses)
    {
        // if we dont know the type of room that a certain cell is in, we use this equal-prob value
        Room& unknown = rooms[UnknownRoomName];
        unknown.name = UnknownRoomName;
        unknown.finalize(numberOfClasses, *this);
        return unknown;
    }

    void ClassMap::setUpNoRooms()
    {
        Room& unknown = createUnknownRoom(sourceProbByClass.size());
        for (size_t i = 0; i < roomOfObjects.size(); i++)
            roomOfObjects[i] = &unknown;
    }

    float ClassMap::Room::GetClassProb(const std::string& className) const
    {
        GSL_ASSERT_MSG(classProb.contains(className), "Missing class {} in room {}", className, name);
        return classProb.at(className);
    }

    float ClassMap::Room::GetPrior(const std::string& className) const
    {
        GSL_ASSERT_MSG(classPrior.contains(className), "Missing class {} in room {}", className, name);
        return classPrior.at(className);
    }

    void ClassMap::Room::SetProbOf(const std::string& className, float prob)
    {
        classProb[className] = prob;
    }

    void ClassMap::Room::finalize(uint totalNumberOfClasses, const ClassMap& map)
    {
        float sum = 0; // the amount of probability that has already been assigned. Whatever's left will be distributed among the remaining classes
        for (auto& [name, prob] : classProb)
            sum += prob;

        // create entries for the classes of interest that were not mentioned in the room ontology
        float probOfClassNotMentioned = (1 - sum) / (totalNumberOfClasses - classProb.size());
        for (const auto& [_class, _] : map.sourceProbByClass)
            if (!classProb.contains(_class) && _class != ClassMap::otherClassName)
                classProb[_class] = probOfClassNotMentioned;

        // whatever is left after handling the classes of interest can all be lumped into "other"
        {
            if (classProb.contains(ClassMap::otherClassName))
                GSL_ERROR("The class label '{}' should not appear in the room ontology, it is a special case that covers all other classes", ClassMap::otherClassName);
            else
                classProb[ClassMap::otherClassName] = 0;

            // combine all the non-interesing classes into a single entry
            for (auto it = classProb.cbegin(); it != classProb.cend();)
            {
                if (!map.sourceProbByClass.contains(it->first))
                {
                    classProb[ClassMap::otherClassName] += it->second;
                    it = classProb.erase(it);
                }
                else
                    it++;
            }

            // add to that one entry all the probability remaining (accounting for the classes that were never mentioned)
            sum = 0;
            for (auto& [name, prob] : classProb)
                sum += prob;
            classProb[ClassMap::otherClassName] += 1 - sum;
        }

        // store the prior, which is needed later to combine p(o|z) with P(o|r)
        constexpr float occupancyPrior = 0.5;
        const float priorNormalClass = std::lerp(0, 1. / totalNumberOfClasses, occupancyPrior);
        for (const auto& [_class, _] : classProb)
            if (_class != ClassMap::otherClassName)
                classPrior[_class] = priorNormalClass;

        int numClassesInOther = totalNumberOfClasses - (classProb.size() - 1);                  // -1 because we dont want to count the "other class itself
        classPrior[ClassMap::otherClassName] = (numClassesInOther -1) * priorNormalClass        // the normal classes that went into "other"
                                               + std::lerp(1, 1. / totalNumberOfClasses, occupancyPrior); // the "background" class, which has a higher prior than any of the others due to the occupancy uncertainty

#if GSL_DEBUG
        {
            float sumProb = 0;
            for (auto& [name, prob] : classProb)
                sumProb += prob;
            GSL_ASSERT(Utils::approx(sumProb, 1));


            float sumPrior = 0;
            for (auto& [name, prob] : classPrior)
                sumPrior += prob;
            GSL_ASSERT(Utils::approx(sumPrior, 1));
        }
#endif
    }

} // namespace GSL