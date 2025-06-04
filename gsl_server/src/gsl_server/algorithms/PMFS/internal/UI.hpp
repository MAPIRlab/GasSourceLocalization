#pragma once
#ifdef USE_GUI
#include <gsl_server/algorithms/PMFS/internal/HitProbability.hpp>
#include <ament_imgui/ament_imgui.h>
#include <implot/implot.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/subscription.hpp>

#include <string>
#include <thread>

namespace GSL
{
    class PMFS;
}

namespace GSL::PMFS_internal
{

    class UI
    {

    public:
        UI(PMFS* _pmfs);
        ~UI();
        void run();
        void addConcentrationReading(double ppm);
    protected:
        void renderImgui();
        void createUI();
        void createPlots();
        bool useCoordinates();
        int selectVariable();
        std::string printCell(const Grid2D<HitProbability>& grid, const int& x, const int& y);
        
        AmentImgui imgui;
        std::jthread renderThread;
        PMFS* pmfs;
        double last_concentration_reading = 0;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clickedPointSub;
        Vector2 selectedCoordinates;
        Vector2 goalCoordinates;
    };
} // namespace GSL::PMFS_internal

#endif