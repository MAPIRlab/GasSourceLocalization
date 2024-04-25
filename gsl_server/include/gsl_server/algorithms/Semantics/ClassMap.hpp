#pragma once
#include "ISemantics.hpp"
#include "ClassDistribution.hpp"
#include <rclcpp/node.hpp>

namespace GSL
{
    class ClassMap : public ISemantics
    {
    public:
        ClassMap();
        std::vector<double> GetSourceProbability() override;
        void OnUpdate() override;

    private:
        std::vector<ClassDistribution> classDistributions;
        rclcpp::Node::SharedPtr node;
    };
}