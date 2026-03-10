#include "gsl_server/algorithms/Common/Simulation.hpp"
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

    GraphUI::GraphUI(GraphGSL* _gsl) : gsl(_gsl)
    {
        clickedPointSub =
            gsl->node->create_subscription<geometry_msgs::msg::PointStamped>(
                "/clicked_point", 1,
                [this](const geometry_msgs::msg::PointStamped::SharedPtr point)
                {
                    selectedCoordinates.x = point->point.x;
                    selectedCoordinates.y = point->point.y;
                });
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
            fmt::format("{}/resources/graph_imgui.ini",
                        ament_index_cpp::get_package_share_directory("gsl_server"))
                .c_str(),
            "GraphGSL", 900, 600);
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
        ImGui::Begin("Main", nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoScrollbar |
                         ImGuiWindowFlags_NoCollapse);
        {
            ImGui::Checkbox("Draw graph", &gsl->drawGraph);
            ImGui::DragFloat("Node separation", &gsl->graph.nodeSeparationViz, 0.05, 1.,
                             10.);
            SelectNodes();

            if (ImGui::Button("Simulate source"))
            {
                std::shared_ptr<Node> node =
                    gsl->graph.GetCorrespondingNode(selectedCoordinates);
                auto realNode = As<RealNode>(node);
                if (node)
                {
                    Simulation sim{
                        .source = SimulationSource(selectedCoordinates,
                                                   realNode->GetOccupancy().metadata),
                        .minWarmupIterations = 1000,
                        .maxWarmupIterations = 2000,
                        .wind = realNode->GetWindMap(),
                        .outlets = Outlets{
                            .mask = realNode->GetOutletsMask(),
                            .exitsCount = std::vector<size_t>(realNode->arcs.size(), 0),
                        },
                    };

                    sim.outlets->exitsCount.resize(realNode->arcs.size(), 0);
                    sim.outlets->enabled.resize(realNode->arcs.size(), true);

                    for (size_t i = 0; i < realNode->arcs.size(); i++)
                        if (realNode->arcs.at(i).to.lock()->id == "room_2")
                            sim.outlets->enabled.at(i) = false;

                    std::vector<float> hitMap(realNode->GetOccupancy().data.size(), 0.);
                    sim.Run(hitMap);
                    
                    GSL_INFO("Emitted {} filaments in total", sim.totalEmittedFilaments);
                    for (size_t i = 0; i < sim.outlets->exitsCount.size(); i++)
                        GSL_INFO("{} -> {}", sim.outlets->exitsCount.at(i),
                                 realNode->arcs.at(i).to.lock()->id);
                }
                else
                    GSL_ERROR("No node corresponds to coords {}", selectedCoordinates);
            }
        }
        ImGui::End();

        ImGui::Begin("Current State");
        {
            if (gsl->stateMachine.getCurrentState())
                gsl->stateMachine.getCurrentState()->RenderUI();
            else
                ImGui::Text("Null state");
        }
        ImGui::End();
    }

    void GraphUI::SelectNodes()
    {
        bool somethingChanged = false;
        if (ImGui::Button("Toggle All"))
        {
            occupancyToggleState = !occupancyToggleState;
            for (auto& entry : gsl->graph.selectedForVisualization)
                entry.second = occupancyToggleState;
            somethingChanged = true;
        }

        for (auto node : gsl->graph.nodes)
        {
            if (!Is<RealNode>(node))
                continue;

            if (!gsl->graph.selectedForVisualization.contains(node->id))
                gsl->graph.selectedForVisualization[node->id] = true;

            bool oldValue = gsl->graph.selectedForVisualization.at(node->id);
            ImGui::Checkbox(node->id.c_str(),
                            &gsl->graph.selectedForVisualization.at(node->id));

            if (gsl->graph.selectedForVisualization.at(node->id) != oldValue)
                somethingChanged = true;
        }

        if (somethingChanged)
            Utils::ClearMarkers(gsl->pubs.occupancyPub);
    }
} // namespace GSL

#endif