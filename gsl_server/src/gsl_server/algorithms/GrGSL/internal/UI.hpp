#pragma once
#ifdef USE_GUI
#include "gsl_server/algorithms/Common/Grid2D.hpp"
#include <ament_imgui/ament_imgui.h>
#include <gsl_server/algorithms/GrGSL/GrGSL_internal.hpp>
#include <implot/implot.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/subscription.hpp>

#include <string>
#include <thread>

namespace GSL
{
    class GrGSL;
}

namespace GSL::GrGSL_internal
{

    class UI
    {
    public:
        UI(GrGSL* _grGSL);
        ~UI();
        void run();
        void addConcentrationReading(double ppm);

    private:
        void renderImgui();
        void createUI();
        void createPlots();
        bool useCoordinates();
        std::string printCell(const Grid2D<Cell>& grid, const int& x, const int& y);
    
    private:
        std::jthread renderThread;
        GrGSL* grgsl;
        double last_concentration_reading = 0;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clickedPointSub;
        Vector2 selectedCoordinates;
        Vector2 goalCoordinates;
    };

} // namespace GSL::GrGSL_internal

#endif