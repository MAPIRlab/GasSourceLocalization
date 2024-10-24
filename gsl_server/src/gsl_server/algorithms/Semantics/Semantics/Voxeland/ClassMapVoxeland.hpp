#pragma once
#include <gsl_server/algorithms/Semantics/Semantics/Common/ISemantics.hpp>
#include <gsl_server/algorithms/Semantics/Semantics/Common/ClassMap.hpp>
#include <gsl_server/core/ros_typedefs.hpp>
#include <gsl_server/algorithms/Common/Utils/BufferWrapper.hpp>
#include <gsl_server/algorithms/Common/Grid2D.hpp>
#include <gsl_server/algorithms/Common/Grid3D.hpp>
#include <voxeland_msgs/srv/get_class_distributions.hpp>

#include <rclcpp/rclcpp.hpp>

namespace GSL
{
    class ClassMapVoxeland : public ISemantics
    {
    public:
        ClassMapVoxeland(Grid2DMetadata _gridMetadata, std::vector<Occupancy>& occupancy, BufferWrapper& _bufferWrapper,
                         rclcpp::Node::SharedPtr _node);

        std::vector<double> GetSourceProbability() override;
        void GetSourceProbabilityInPlace(std::vector<double>& sourceProb) override;
        double GetSourceProbabilityAt(const Vector3& point) override;
        void OnUpdate() override;

        double GetEntropyAt(const Vector3& point) override;
        std::vector<double> GetEntropy() override;
        void GetEntropyInPlace(std::vector<double>& entropy) override;

        std::string GetDebugInfo(const Vector3& point) override;
    private:
        rclcpp::Node::SharedPtr node;

        ClassMap classMap;
        std::vector<Occupancy> wallsOccupancy; //TODO this is a copy. Should it be?
        Grid3DMetadata gridMetadata;
        BufferWrapper& bufferWrapper;

        // Voxeland service
        rclcpp::Client<voxeland_msgs::srv::GetClassDistributions>::SharedPtr client;
        std::vector<Point> requestPoints;

        void initializeRequestPoints();
        void getUpdatedMapFromService();
        void visualize();
    };
}