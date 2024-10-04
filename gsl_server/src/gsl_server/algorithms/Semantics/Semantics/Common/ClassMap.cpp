#include "ClassMap.hpp"
#include <filesystem>
#include <gsl_server/algorithms/Common/Utils/Collections.hpp>
#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>
#include <gsl_server/core/Logging.hpp>
#include <gsl_server/core/Macros.hpp>
#include <gsl_server/core/VectorsImpl/vmath_DDACustomVec.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

namespace GSL
{
    static constexpr const char* UnknownRoomName = "unknown";

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

        // store the prior of p(class), which is needed later to combine p(class | z) with p(class | room)
        // this needs to account for the occupancy prior
        {
            const float priorNormalClass = std::lerp(0, 1. / numberOfClasses, occupancyPrior);
            for (const auto& [_class, _] : sourceProbByClass)
                if (_class != ClassMap::otherClassName)
                    classPrior[_class] = priorNormalClass;

            int numClassesInOther = numberOfClasses - (sourceProbByClass.size() - 1);                    // -1 because we dont want to count the "other class itself
            classPrior[ClassMap::otherClassName] = (numClassesInOther - 1) * priorNormalClass            // the normal classes that went into "other"
                                                   + std::lerp(1, 1. / numberOfClasses, occupancyPrior); // the "background" class, which has a higher prior than any of the others due to the occupancy uncertainty

#if GSL_DEBUG
            float sumPrior = 0;
            for (auto& [name, prob] : classPrior)
                sumPrior += prob;
            GSL_ASSERT(Utils::approx(sumPrior, 1));
#endif
        }
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

        // We're done!
        GSL_INFO("Source probability by semantic class (normalized):");
        for (auto& kv : sourceProbByClass)
            GSL_INFO("{}: {}", kv.first, kv.second);

        for (ClassDistribution& dist : classProbabilityZ)
            dist.Initialize(class_list); //TODO use the prior
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
            numberOfClasses = ontology["number_classes"].as<size_t>();
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
                room.finalize(numberOfClasses, *this);
            }

            // initialize all cells to the unknown room
            Room& unknown = createUnknownRoom();
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
            float totalClassProb = prob * room->GetClassProb(className) / classPrior.at(className);
            retValue += totalClassProb * sourceProbByClass.at(_class) / classPrior.at(className); // TODO this prior here seems weird, but it appears in the formulation. Check it.
        }
        return retValue;
    }

    void ClassMap::updateObjectProbabilities(size_t index, const std::vector<std::pair<std::string, float>>& scores)
    {
        for (const auto& pair : scores)
        {
            const std::string& _class = pair.first;
            float score = pair.second;
            float previous = classProbabilityZ[index].ProbabilityOf(_class);

            // clamp at arbitrary high value to avoid numerical issues as more observations pile up
            float prob = std::min(0.95f, score * previous / classPrior.at(_class));
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

    ClassMap::Room& ClassMap::createUnknownRoom()
    {
        // if we dont know the type of room that a certain cell is in, we use this equal-prob value
        Room& unknown = rooms[UnknownRoomName];
        unknown.name = UnknownRoomName;
        unknown.finalize(numberOfClasses, *this);
        return unknown;
    }

    void ClassMap::setUpNoRooms()
    {
        numberOfClasses = sourceProbByClass.size();
        Room& unknown = createUnknownRoom();
        for (size_t i = 0; i < roomOfObjects.size(); i++)
            roomOfObjects[i] = &unknown;
    }

    float ClassMap::Room::GetClassProb(const std::string& className) const
    {
        if (!ready)
        {
            GSL_ERROR("Cannot read conditional probabilities from room {}, it has not been finalized yet", name);
            return 0;
        }
        GSL_ASSERT_MSG(classProb.contains(className), "Missing class {} in room {}", className, name);
        return classProb.at(className);
    }

    void ClassMap::Room::SetProbOf(const std::string& className, float prob)
    {
        if (ready)
        {
            GSL_ERROR("Cannot change conditional probabilities on room {}, it has already been finalized", name);
            return;
        }
        classProb[className] = prob;
    }
    void ClassMap::Room::finalize(uint totalNumberOfClasses, const ClassMap& map)
    {
        if (ready)
        {
            GSL_ERROR("Finalize() called twice on room {}. Ignoring the second call.", name);
            return;
        }

        // TODO we are assuming here that the background class never appears in the room ontology. Do we want to modify this to allow it to appear?

        // calculate the conditional -- p(class | room). Since we are not conditioning to the occupancy, we need to do:
        // p(class | room) = p(class | room, occupied) * p(occupied | room) + p(class | room, !occupied) * p(!occupied | room)
        // and we just assume that p(occupied | room) == p(occupied), because what else are you going to do
        // so that's why we lerp based on the occupancyPrior

        float sum = 0; // the amount of probability (p(class | occupied)) that has already been assigned. Whatever's left will be distributed among the remaining classes
        for (auto& [name, prob] : classProb)
            sum += prob;

        //[!] It's important to note that the probabilities specified in the room ontology are actually p(class | room, occupied),
        // so "other" will be somewhat more likely in classProbs than it appears in the ontology
        for (const auto& [_class, _] : classProb)
            classProb[_class] = std::lerp(0, classProb[_class], occupancyPrior);

        // create entries for the classes of interest that were not mentioned in the room ontology
        const float probClassConditional = (1 - sum) / (totalNumberOfClasses - classProb.size()); // p(class | occupied, room)
        const float probClassNotMentioned = std::lerp(0., probClassConditional, occupancyPrior);  // p(class | room)

        for (const auto& [_class, _] : map.sourceProbByClass)
            if (!classProb.contains(_class) && _class != ClassMap::otherClassName)
                classProb[_class] = probClassNotMentioned;

        // whatever is left after handling the classes of interest can all be lumped into "other"
        {
            int numberMissingClasses = totalNumberOfClasses - classProb.size();
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

            // add to that one entry all the probability remaining (accounting for the classes that were never mentioned);
            classProb[ClassMap::otherClassName] += probClassNotMentioned * (numberMissingClasses - 1)    // All the remainig normal classes
                                                   + std::lerp(1, probClassConditional, occupancyPrior); // the background class
        }

        ready = true;
#if GSL_DEBUG
        {
            float sumProb = 0;
            for (auto& [name, prob] : classProb)
                sumProb += prob;
            GSL_ASSERT(Utils::approx(sumProb, 1));
        }
#endif
    }

} // namespace GSL