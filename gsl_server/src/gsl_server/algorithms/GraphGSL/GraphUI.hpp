#pragma once
#if USE_GUI

#include "gsl_server/core/ros_typedefs.hpp"
#include <rclcpp/publisher.hpp>
#include <thread>

namespace GSL
{
    class GraphGSL;

    class GraphUI
    {
    public:
        GraphUI(GraphGSL* _gsl);
        ~GraphUI();

        void Run();
        void RenderImgui();

    private:
        void CreateUI();
        void DrawGraph();
        void DrawOccupancyMaps();

        GraphGSL* gsl;
        std::jthread renderThread;
        rclcpp::Publisher<MarkerArray>::SharedPtr graphPub;
        rclcpp::Publisher<MarkerArray>::SharedPtr occupancyPub;

        std::map<std::string, bool> selectedOccupancy;
    };
} // namespace GSL

#endif