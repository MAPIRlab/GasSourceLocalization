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
        void DrawMaps();

        void Clear(rclcpp::Publisher<MarkerArray>::SharedPtr pub);

        void MergeWindMarkers(MarkerArray& all, const MarkerArray& _new);

        GraphGSL* gsl;
        std::jthread renderThread;
        rclcpp::Publisher<MarkerArray>::SharedPtr graphPub;
        rclcpp::Publisher<MarkerArray>::SharedPtr occupancyPub;
        rclcpp::Publisher<MarkerArray>::SharedPtr windPub;

        bool occupancyToggleState = true;
        std::map<std::string, bool> selectedOccupancy;
        bool drawGraph = true;
    };
} // namespace GSL

#endif