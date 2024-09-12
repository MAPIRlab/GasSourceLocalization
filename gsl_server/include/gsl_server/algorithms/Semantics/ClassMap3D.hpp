#pragma once
#include "ISemantics.hpp"
#include "ClassMap.hpp"
#include <gsl_server/core/ros_typedefs.hpp>
#include <gsl_server/Utils/BufferWrapper.hpp>
#include <gsl_server/algorithms/Common/Grid.hpp>
#include <gsl_server/algorithms/Semantics/AABB.hpp>

#include <rclcpp/rclcpp.hpp>

namespace GSL
{
    class ClassMap3D : public ISemantics
    {
    public:
        ClassMap3D(GridMetadata _gridMetadata, std::vector<Occupancy>& occupancy, BufferWrapper& _bufferWrapper,
                 const PoseWithCovarianceStamped& _currentRobotPose);

        std::vector<double> GetSourceProbability() override;
        void GetSourceProbabilityInPlace(std::vector<double>& sourceProb) override;
        double GetSourceProbabilityAt(const Vector3& point) override;
        void OnUpdate() override;
    private:
        rclcpp::Node::SharedPtr node;
        rclcpp::Publisher<Marker>::SharedPtr classMarkers;

        ClassMap classMap;
        std::vector<Occupancy> wallsOccupancy; //TODO this is a copy. Should it be?
        GridMetadata gridMetadata;
        BufferWrapper& bufferWrapper;
        const PoseWithCovarianceStamped& currentRobotPose;
        Vector2 zLimits;

        void getUpdatedMapFromService();
        void visualize();
        size_t indexOf(const Vector3Int& v) const;
        size_t numZLevels() const;
    };
}