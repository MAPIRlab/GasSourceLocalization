#include "gsl_server/algorithms/Common/Utils/Pointers.hpp"
#include "gsl_server/algorithms/Common/Utils/RosUtils.hpp"
#include "gsl_server/algorithms/GraphGSL/Node.hpp"
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
        graphPub = gsl->node->create_publisher<MarkerArray>("gsl_graph", 200);
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
            fmt::format("{}/resources/graph_imgui.ini", ament_index_cpp::get_package_share_directory("gsl_server")).c_str(),
            "GraphGSL",
            900,
            600);
        ImPlot::CreateContext();

        rclcpp::Rate rate(30);

        while (rclcpp::ok())
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
        if (ImGui::Button("Draw maps"))
        {
            {
                Marker clear;
                clear.action = Marker::DELETEALL;
                MarkerArray array;
                array.markers.push_back(clear);
                graphPub->publish(array);
            }

            MarkerArray array;
            size_t id = 0;
            for (auto node : gsl->graph.nodes)
            {
                Vector2 position = node->GetPosition();
                ColorRGBA color;
                if (Is<RealNode>(node))
                    color = Utils::create_color(0, 1, 0);
                else
                    color = Utils::create_color(1, 0, 0);

                Marker marker;
                marker.header.frame_id = "map";
                marker.type = Marker::SPHERE;
                marker.scale.x = 0.3;
                marker.scale.y = 0.3;
                marker.scale.z = 0.3;
                marker.color = color;
                marker.pose.position.x = position.x;
                marker.pose.position.y = position.y;
                marker.id = id;
                id++;
                array.markers.push_back(marker);

                // for(size_t i = 0; i<)
            }
            graphPub->publish(array);
        }
    }
} // namespace GSL

#endif