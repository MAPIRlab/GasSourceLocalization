#pragma once
#ifdef USE_GUI
#include <gsl_server/algorithms/PMFS/internal/HitProbability.hpp>
#include <ament_imgui/ament_imgui.h>
#include <implot/implot.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/node.hpp>

#include <string>
#include <thread>

namespace GSL
{
    class SemanticPMFS;
}

namespace GSL::SemanticPMFS_internal
{

    class UI
    {

    public:
        UI(SemanticPMFS* _pmfs);
        ~UI();
        void run();
        void addConcentrationReading(double ppm);
    protected:
        enum Variable{HitProb, SourceProb, Semantics};
        
        void renderImgui();
        void createUI();
        void createPlots();
        Variable selectVariable();
        std::string printCell(Variable variable);
        void visualizeQueryPoint();
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clickedPointSub;
        
        AmentImgui imgui;
        std::jthread renderThread;
        SemanticPMFS* pmfs;
        double last_concentration_reading = 0;
        Vector3 selectedCoordinates;
        Vector2 goalCoordinates;
        rclcpp::Node::SharedPtr uiNode;
    };
} // namespace GSL::PMFS_internal

#endif