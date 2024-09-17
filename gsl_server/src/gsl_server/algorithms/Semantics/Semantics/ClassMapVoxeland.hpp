#pragma once
#include "Common/ISemantics.hpp"
#include "ClassMap.hpp"
#include <gsl_server/core/ros_typedefs.hpp>
#include <gsl_server/algorithms/Common/Utils/BufferWrapper.hpp>
#include <gsl_server/algorithms/Common/Grid2D.hpp>
#include <gsl_server/algorithms/Semantics/Semantics/AABB.hpp>

#include <rclcpp/rclcpp.hpp>

namespace GSL
{
    class ClassMapVoxeland : public ISemantics
    {
    public:
        ClassMapVoxeland(Grid2DMetadata _gridMetadata, std::vector<Occupancy>& occupancy, BufferWrapper& _bufferWrapper,
                   const PoseWithCovarianceStamped& _currentRobotPose);

        std::vector<double> GetSourceProbability() override;
        void GetSourceProbabilityInPlace(std::vector<double>& sourceProb) override;
        double GetSourceProbabilityAt(const Vector3& point) override;
        void OnUpdate() override;
    private:
        rclcpp::Node::SharedPtr node;

        ClassMap classMap;
        std::vector<Occupancy> wallsOccupancy; //TODO this is a copy. Should it be?
        Grid2DMetadata gridMetadata;
        BufferWrapper& bufferWrapper;
        const PoseWithCovarianceStamped& currentRobotPose;
        Vector2 zLimits;

        void getUpdatedMapFromService();
        void visualize();
        size_t indexOf(const Vector3Int& v) const;
        size_t numZLevels() const;
    };
}