#pragma once
#include "ISemantics.hpp"
#include "ClassDistribution.hpp"
#include <gsl_server/core/ros_typedefs.hpp>
#include <gsl_server/Utils/BufferWrapper.hpp>
#include <gsl_server/algorithms/Common/Grid.hpp>
#include <gsl_server/algorithms/Semantics/AABB.hpp>

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

namespace GSL
{
    class ClassMap2D : public ISemantics
    {
        using Detection3D = vision_msgs::msg::Detection3D;
        using Detection3DArray = vision_msgs::msg::Detection3DArray;
    public:
        ClassMap2D(GridMetadata _gridMetadata, std::vector<Occupancy>& occupancy, BufferWrapper& _bufferWrapper,
                 const PoseWithCovarianceStamped& _currentRobotPose);
        std::vector<double> GetSourceProbability() override;
        void OnUpdate() override;

    private:
        rclcpp::Node::SharedPtr node;
        rclcpp::Subscription<Detection3DArray>::SharedPtr cameraSub;
        rclcpp::Publisher<Marker>::SharedPtr classMarkers;

        std::map<std::string, float> sourceProbByClass;
        std::vector<ClassDistribution> classDistributions;
        std::vector<Occupancy> wallsOccupancy; //TODO this is a copy. Should it be?
        GridMetadata gridMetadata;
        BufferWrapper& bufferWrapper;
        const PoseWithCovarianceStamped& currentRobotPose;

        struct FOV
        {
            float angleRads;
            float minDist;
            float maxDist;
        } fov;

        void parseOntology(const std::string& path);
        double getSourceProbByClass(const std::string& _class);
        void updateObjectProbabilities(const Vector2Int& indices, const std::vector<std::pair<std::string, float>>& scores);

        void detectionCallback(Detection3DArray::ConstSharedPtr msg);
        AABB2DInt getAABB(const Detection3D& detection);
        std::unordered_set<Vector2Int> getCellsInFOV();
        void publishClassMarkers();
    };
}