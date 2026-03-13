#include "SimulationSystem.hpp"
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

using namespace GSL::Graph_internal;
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
            ImGui::SetNextItemWidth(100);
            ImGui::DragFloat("Node separation", &gsl->graph.nodeSeparationViz, 0.05, 1., 10.);
            SelectNodes();
        }
        ImGui::End();

        SimulateSourceMenu();

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
        {
            Utils::ClearMarkers(gsl->pubs.occupancyPub);
            Utils::ClearMarkers(gsl->pubs.windPub);
        }
    }

    void GraphUI::SimulateSourceMenu()
    {
        ImGui::Begin("Simulate Source");
        {
            std::shared_ptr<Node> node = gsl->graph.GetCorrespondingNode(selectedCoordinates);
            std::string name = node ? node->id : "Null";
            ImGui::Text("Currently selected node: %s", name.c_str());

            ImGui::Checkbox("Cummulative map", &SimulationSystem::cummulativeMap);
            ImGui::Checkbox("Simulate point", &simulationOptions.exactPoint);

            if (simulationOptions.exactPoint)
                ImGui::DragFloat2("Selected point", &selectedCoordinates.x, 0.02);
            else if (node)
            {
                if (node->arcs.size() == 0)
                    ImGui::Text("Node has no arcs!");
                else
                {
                    if (simulationOptions.selectedArcIdx > node->arcs.size())
                        simulationOptions.selectedArcIdx = 0;

                    ImGui::PushID("node");
                    if (ImGui::BeginCombo("Arc", node->arcs.at(simulationOptions.selectedArcIdx).to.lock()->id.c_str()))
                    {
                        for (size_t i = 0; i < node->arcs.size(); i++)
                            if (ImGui::Selectable(node->arcs.at(i).to.lock()->id.c_str()))
                                simulationOptions.selectedArcIdx = i;

                        ImGui::EndCombo();
                    }
                    ImGui::PopID();
                }
            }
            else
            {
                ImGui::BeginDisabled();
                ImGui::BeginCombo("Arc", "No node selected");
                ImGui::EndDisabled();
            }

            ImGui::BeginDisabled(!simulationOptions.simulationEnabled);
            if (ImGui::Button("Run simulation"))
            {
                auto realNode = As<RealNode>(node);
                if (realNode)
                {
                    // Run
                    gsl->functionQueue.submit([this, realNode]()
                                              {
                                                  simulationOptions.simulationEnabled = false;
                                                  SimulationSystem::SimWithResult result;
                                                  if (simulationOptions.exactPoint)
                                                      result = SimulationSystem::SimulateFromPoint(realNode, selectedCoordinates);
                                                  else
                                                      result = SimulationSystem::SimulateFromArc(realNode->arcs.at(simulationOptions.selectedArcIdx));

                                                  // Log results
                                                  GSL_INFO("Emitted {} filaments in total", result.simulation->totalEmittedFilaments);
                                                  for (size_t i = 0; i < result.simulation->outlets->exitsCount.size(); i++)
                                                      GSL_INFO("{} -> {}", result.simulation->outlets->exitsCount.at(i),
                                                               realNode->arcs.at(i).to.lock()->id);

                                                  result.simulation->displayImage(*result.hitMap, "result", simulationOptions.imageDisplayPower);
                                                  simulationOptions.simulationEnabled = true;
                                              });
                }
                else
                    GSL_ERROR("No node corresponds to coords {}", selectedCoordinates);
            }

            ImGui::SetNextItemWidth(100);
            ImGui::DragFloat("Blur sigma", &SimulationSystem::blurSigma, 0.01, 0, 2.0);
            ImGui::SetNextItemWidth(100);
            ImGui::DragFloat("Image color power", &simulationOptions.imageDisplayPower, 0.05, 0, 10);
            ImGui::EndDisabled();
        }
        ImGui::End();
    }
} // namespace GSL

#endif