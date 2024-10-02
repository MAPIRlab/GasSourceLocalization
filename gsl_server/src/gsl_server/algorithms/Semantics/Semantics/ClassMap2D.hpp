#pragma once
#include "Common/ISemantics.hpp"
#include "Common/ClassMap.hpp"
#include <gsl_server/core/ros_typedefs.hpp>
#include <gsl_server/algorithms/Common/Utils/BufferWrapper.hpp>
#include <gsl_server/algorithms/Common/Grid2D.hpp>
#include <gsl_server/algorithms/Semantics/Semantics/AABB.hpp>

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

namespace GSL
{
    class ClassMap2D : public ISemantics
    {
        using Detection3D = vision_msgs::msg::Detection3D;
        using Detection3DArray = vision_msgs::msg::Detection3DArray;
    public:
        ClassMap2D(Grid2DMetadata _gridMetadata, std::vector<Occupancy>& occupancy, BufferWrapper& _bufferWrapper,
                   const PoseWithCovarianceStamped& _currentRobotPose);

        std::vector<double> GetSourceProbability() override;
        void GetSourceProbabilityInPlace(std::vector<double>& sourceProb) override;
        double GetSourceProbabilityAt(const Vector3& point) override;
        void OnUpdate() override;
    private:
        rclcpp::Node::SharedPtr node;
        rclcpp::Subscription<Detection3DArray>::SharedPtr cameraSub;
        rclcpp::Publisher<Marker>::SharedPtr classMarkers;

        ClassMap classMap;
        std::vector<Occupancy> wallsOccupancy; //TODO this is a copy. Should it be?
        Grid2DMetadata gridMetadata;
        BufferWrapper& bufferWrapper;
        const PoseWithCovarianceStamped& currentRobotPose;

        struct FOV
        {
            float angleRads;
            float minDist;
            float maxDist;
        } fov;

        void detectionCallback(Detection3DArray::ConstSharedPtr msg);
        AABB2DInt getAABB(const Detection3D& detection);
        std::unordered_set<Vector2Int> getCellsInFOV(Pose robotPose);
        void visualize();
    };
}