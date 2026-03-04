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
        DrawGraph();
        DrawOccupancyMaps();
    }

    void GraphUI::DrawGraph()
    {
        Clear(graphPub);
        ImGui::Checkbox("Draw graph", &drawGraph);
        if (drawGraph)
        {
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

                // draw the arcs
                for (size_t i = 0; i < node->arcs.size(); i++)
                {
                    Vector2 otherPos = node->arcs.at(i).to.lock()->GetPosition();
                    Marker marker;
                    marker.header.frame_id = "map";
                    marker.type = Marker::LINE_STRIP;
                    marker.scale.x = 0.1;
                    marker.color = Utils::create_color(0, 0, 1);
                    marker.points.push_back(Point{}.set__x(position.x).set__y(position.y));
                    marker.points.push_back(Point{}.set__x(otherPos.x).set__y(otherPos.y));
                    marker.id = id;
                    id++;
                    array.markers.push_back(marker);
                }
            }
            graphPub->publish(array);
        }
    }

    void GraphUI::DrawOccupancyMaps()
    {
        bool somethingChanged = false;
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
                Grid2D<Vector2> wind = realNode->GetWindMap();
                Marker marker = Utils::createPointsOccupancyMarker(wind.occupancy, wind.metadata);
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