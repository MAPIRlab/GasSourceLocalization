#pragma once
#if USE_GUI

#include "Graph.hpp"
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

        void run();
        void renderImgui();

        void createUI();

    private:
        GraphGSL* gsl;
        std::jthread renderThread;
        rclcpp::Publisher<Marker>::SharedPtr graphPub;
    };
} // namespace GSL

#endif