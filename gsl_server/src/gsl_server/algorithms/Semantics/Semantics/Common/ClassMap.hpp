#pragma once
#include <cstddef>
#include <filesystem>
#include <gsl_server/algorithms/Semantics/Semantics/Common/ClassDistribution.hpp>
#include <gsl_server/core/Vectors.hpp>
#include <gsl_server/core/VectorsImpl/vmath_DDACustomVec.hpp>
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

        // values read directly from the ontology
        std::map<std::string, float> sourceProbByClass;

        // For each object, p(O | Z). Does not take into account room classification (see computeSourceProbability for that)
        std::vector<ClassDistribution> classProbabilityZ;

        void Initialize(rclcpp::Node::SharedPtr node, size_t size, MapToPixel indexToPixel);

        // get the final value for p(S | semantics)
        double computeSourceProbability(size_t index);

        //get the class distribution considering both observations and rooms
        ClassDistribution classDistributionAt(size_t index);

        // run the bayesian filter for p(O | Z) with a new observation. It is assumed that occupancy is already factored into the new probabilities (if applicable)
        void updateObjectProbabilities(size_t index, const std::vector<std::pair<std::string, float>>& scores);

        // overwrite p(O | Z) with the distribution received from msg. Does not include the room classification stuff (that is always handled locally)
        void FromMsg(size_t index, const std::vector<vision_msgs::msg::ObjectHypothesis>& msg);

        // returns "other" if the class ID does not appear in the gas ontology. Otherwise, returns the input unchanged
        std::string filterClassID(const std::string& id);

    private:
        // We need this concept of an occupancy probability to match how Voxeland does classification
        // It also has the very reasonable side effect of making the background class more likely than any of the others a priori
        // Good, because most cells in any environment will actually be empty
        static constexpr float occupancyPrior = 0.5;


        // Knowledge of the room types is useful to provide a prediction of the class of the cell before we even observe the cell
        //  e.g. a cell that is in a kitchen is more likely to be an oven than a toilet, I don't need to see it to tell you that
        struct Room
        {
            std::string name;
            float GetClassProb(const std::string& className) const;
            void SetProbOf(const std::string& className, float prob);

            // call after inserting the class probabilities read from the ontology. Fills the rest of classes with the appropriate probs
            void finalize(uint totalNumberOfClasses, const ClassMap& map);

        private:
            bool ready = false;
            std::unordered_map<std::string, float> classProb; // p(class | room)
        };

        std::vector<std::string> class_list;
        std::unordered_map<std::string, Room> rooms;       // all rooms that exist
        std::vector<const Room*> roomOfObjects;            // for each object (identified by index) a pointer to the room they are in
        std::unordered_map<std::string, float> classPrior; // not all classes have the same prior -- specifically, the "other" class is more likely due to the occupancy probability

        // this is the number of classes considered by the detector, NOT the number of classes that appear in the gas ontology (sourceProbByClass).
        // It is required to calculate the class priors and to fill in the missing classes in the room ontology
        size_t numberOfClasses;

        void parseOntology(const std::string& path, const std::string& targetGas);
        void parseRoomCategorization(const std::string& masksYAMLPath, const std::string& ontologyPath, MapToPixel indexToPixel);

        ClassMap::Room& createUnknownRoom();
        void setUpNoRooms();
        void readMask(const std::filesystem::path& maskPath, const Room& room, MapToPixel indexToPixel);
    };

} // namespace GSL