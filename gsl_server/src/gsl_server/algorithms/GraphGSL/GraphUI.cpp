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
        graphPub = gsl->node->create_publisher<MarkerArray>("gsl_graph", 1);
        occupancyPub = gsl->node->create_publisher<MarkerArray>("gsl_occupancy", 1);
    }

    GraphUI::~GraphUI()
    {
        renderThread.join();
    }

    void GraphUI::Run()
    {
        renderThread = std::jthread(std::bind(&GraphUI::RenderImgui, this));
    }

    void GraphUI::RenderImgui()
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
            CreateUI();
            // createPlots();

            ImguiGL::Render();
            rate.sleep();
        }

        ImPlot::DestroyContext();
        ImguiGL::Close();
    }

    void GraphUI::CreateUI()
    {
        ImGui::Begin("Main", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse);
        {
            DrawGraph();
            DrawOccupancyMaps();

            ImGui::Begin("Current State");
            {
                if (gsl->stateMachine.getCurrentState())
                    gsl->stateMachine.getCurrentState()->RenderUI();
                else
                    ImGui::Text("Null state");
            }
            ImGui::End();
        }
        ImGui::End();
    }

    void GraphUI::DrawGraph()
    {
        Clear(graphPub);
        ImGui::Checkbox("Draw graph", &drawGraph);
        if (drawGraph)
        {
            MarkerArray array = gsl->graph.VisualizeGraph();
            graphPub->publish(array);
        }
    }

    void GraphUI::DrawOccupancyMaps()
    {
        bool somethingChanged = false;
        if (ImGui::Button("Toggle All"))
        {
            occupancyToggleState = !occupancyToggleState;
            for (auto& entry : selectedOccupancy)
                entry.second = occupancyToggleState;
            somethingChanged = true;
        }

        for (auto node : gsl->graph.nodes)
        {
            if (!Is<RealNode>(node))
                continue;

            if (!selectedOccupancy.contains(node->id))
                selectedOccupancy[node->id] = false;

            bool oldValue = selectedOccupancy.at(node->id);
            ImGui::Checkbox(node->id.c_str(), &selectedOccupancy.at(node->id));

            if (selectedOccupancy.at(node->id) != oldValue)
                somethingChanged = true;
        }

        if (somethingChanged)
        {
            Clear(occupancyPub);

            MarkerArray array;
            size_t id = 0;
            for (auto node : gsl->graph.nodes)
            {
                if (!Is<RealNode>(node) || !selectedOccupancy.at(node->id))
                    continue;

                auto realNode = As<RealNode>(node);
                Grid2D<Occupancy> occupancy = realNode->GetOccupancy();
                Marker marker = Utils::createPointsOccupancyMarker(occupancy.occupancy, occupancy.metadata);
                marker.id = id;
                id++;

                array.markers.push_back(marker);
            }
            occupancyPub->publish(array);
        }
    }

    void GraphUI::Clear(rclcpp::Publisher<MarkerArray>::SharedPtr pub)
    {
        // clear old data
        Marker clear;
        clear.action = Marker::DELETEALL;
        MarkerArray array;
        array.markers.push_back(clear);
        pub->publish(array);
    }
} // namespace GSL

#endif