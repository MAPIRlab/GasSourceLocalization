#pragma once
#ifdef USE_GUI
#include "gsl_server/algorithms/Common/Grid2D.hpp"
#include <imgui_gl/imgui_gl.h>
#include <gsl_server/algorithms/GrGSL/GrGSL_internal.hpp>
#include <implot/implot.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>

#include <string>
#include <thread>

namespace GSL
{
    class SemanticGrGSL;
}

namespace GSL::SemanticGrGSL_internal
{

    class UI
    {
    public:
        UI(SemanticGrGSL* _grGSL);
        ~UI();
        void run();
        void addConcentrationReading(double ppm);

    private:
        void renderImgui();
        void createUI();
        void createPlots();
        bool useCoordinates();
        std::string printCell(const Grid2D<GrGSL_internal::Cell>& grid, const int& x, const int& y);
        void simulateInfotaxis(const Vector2Int& selectedCell);

    private:
        rclcpp::Node::SharedPtr uiNode;
        std::jthread renderThread;
        SemanticGrGSL* grgsl;
        double last_concentration_reading = 0;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clickedPointSub;
        Vector2 selectedCoordinates;
        Vector2 goalCoordinates;
    };

} // namespace GSL::SemanticGrGSL_internal

#endif