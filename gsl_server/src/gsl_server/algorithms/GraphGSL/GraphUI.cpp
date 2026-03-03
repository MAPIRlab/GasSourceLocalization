#if USE_GUI

#include "GraphUI.hpp"
#include "gsl_server/algorithms/GraphGSL/GraphGSL.hpp"
#include "imgui_gl/imgui_gl.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <implot/implot.h>
#include <rclcpp/rclcpp.hpp>

namespace GSL
{

    GraphUI::GraphUI(GraphGSL* _gsl)
        : gsl(_gsl)
    {
        graphPub = gsl->node->create_publisher<Marker>("gsl_graph", 1);
    }

    GraphUI::~GraphUI()
    {
        renderThread.join();
    }

    void GraphUI::run()
    {
        renderThread = std::jthread(std::bind(&GraphUI::renderImgui, this));
    }

    void GraphUI::renderImgui()
    {
        ImguiGL::Setup(
            fmt::format("{}/resources/PMFS_imgui.ini", ament_index_cpp::get_package_share_directory("gsl_server")).c_str(),
            "PMFS",
            900,
            600);
        ImPlot::CreateContext();

        rclcpp::Rate rate(30);

        while (rclcpp::ok() && !gsl->HasEnded())
        {
            ImguiGL::StartFrame();
            createUI();
            // createPlots();

            ImguiGL::Render();
            rate.sleep();
        }

        ImPlot::DestroyContext();
        ImguiGL::Close();
    }

    void GraphUI::createUI()
    {
    }
} // namespace GSL

#endif