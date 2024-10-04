#pragma once
#include <gsl_server/core/VectorsImpl/vmath_DDACustomVec.hpp>
#include <cstddef>
#include <filesystem>
#include <gsl_server/algorithms/Semantics/Semantics/Common/ClassDistribution.hpp>
#include <gsl_server/core/Vectors.hpp>
#include <map>
#include <rclcpp/node.hpp>
#include <vector>
#include <vision_msgs/msg/object_hypothesis.hpp>

namespace GSL
{
    class ClassMap
    {
        using MapToPixel = std::function<Vector2Int(size_t)>;

    public:
        static constexpr const char* otherClassName = "other";
        std::map<std::string, float> sourceProbByClass;
        std::vector<ClassDistribution>
            classProbabilityZ; // For each object, p(O | Z). Does not take into account room classification (see computeSourceProbability for that)

        void Initialize(rclcpp::Node::SharedPtr node, size_t size, MapToPixel indexToPixel);
        double computeSourceProbability(size_t index);
        void parseOntology(const std::string& path, const std::string& targetGas);
        void parseRoomCategorization(const std::string& masksYAMLPath, const std::string& ontologyPath, MapToPixel indexToPixel);
        double getSourceProbByClass(const std::string& _class);
        void updateObjectProbabilities(size_t index, const std::vector<std::pair<std::string, float>>& scores);
        void FromMsg(size_t index, const std::vector<vision_msgs::msg::ObjectHypothesis>& msg);
        std::string filterClassID(const std::string& id);

    private:
        // Knowledge of the room types is useful to provide a prediction of the class of the cell before we even observe the cell
        //  e.g. a cell that is in a kitchen is more likely to be an oven than a toilet, I don't need to see it to tell you that
        struct Room
        {
            std::string name;
            float GetClassProb(const std::string& className) const;
            float GetPrior(const std::string& className) const;
            void SetProbOf(const std::string& className, float prob);
            void finalize(uint totalNumberOfClasses, const ClassMap& map);

        private:
            bool ready = false;
            std::unordered_map<std::string, float> classProb;
            std::unordered_map<std::string, float> classPrior;
        };

        std::unordered_map<std::string, Room> rooms; // all rooms that exist
        std::vector<const Room*> roomOfObjects;      // for each object (identified by index) a pointer to the room they are in

        Room& createUnknownRoom(size_t numberOfClasses);
        void setUpNoRooms();
        void readMask(const std::filesystem::path& maskPath, const Room& room, MapToPixel indexToPixel);
    };

} // namespace GSL