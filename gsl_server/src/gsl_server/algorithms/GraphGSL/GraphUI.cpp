#include "SimulationSystem.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"
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
            ImGui::DragFloat("Node separation", &gsl->graph.nodeSeparationViz, 0.005, 1., 10.);
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
            auto node = gsl->graph.GetCorrespondingNode(selectedCoordinates);
            if (selectedNode.node != node)
            {
                selectedNode.node = node;
                selectedNode.combineWeights = std::vector<float>(node->arcs.size(), 0);
            }

            std::string name = selectedNode.node ? selectedNode.node->id : "Null";
            ImGui::Text("Currently selected node: %s", name.c_str());

            ImGui::Checkbox("Simulate point", &simulationOptions.exactPoint);

            if (simulationOptions.exactPoint)
                ImGui::DragFloat2("Selected point", &selectedCoordinates.x, 0.02);
            else if (selectedNode.node)
            {
                if (selectedNode.node->arcs.size() == 0)
                    ImGui::Text("Node has no arcs!");
                else
                {
                    if (simulationOptions.selectedArcIdx > selectedNode.node->arcs.size())
                        simulationOptions.selectedArcIdx = 0;

                    ImGui::PushID("node");
                    if (ImGui::BeginCombo("Arc", selectedNode.node->arcs.at(simulationOptions.selectedArcIdx).to.lock()->id.c_str()))
                    {
                        for (size_t i = 0; i < selectedNode.node->arcs.size(); i++)
                            if (ImGui::Selectable(selectedNode.node->arcs.at(i).to.lock()->id.c_str()))
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
                auto realNode = As<RealNode>(selectedNode.node);
                if (realNode)
                {
                    // Run
                    auto lambda = [this, realNode]()
                    {
                        simulationOptions.simulationEnabled = false;
                        SimulationSystem::SimWithResult result;
                        if (simulationOptions.exactPoint)
                            result = gsl->simulationSystem.SimulateFromPoint(realNode, selectedCoordinates);
                        else
                            result = gsl->simulationSystem.SimulateFromArc(realNode->arcs.at(simulationOptions.selectedArcIdx));

                        // Log results
                        GSL_INFO("Emitted {} filaments in total", result.simulation->totalEmittedFilaments);
                        for (size_t i = 0; i < result.simulation->outlets->exitsCount.size(); i++)
                            GSL_INFO("{} -> {}", result.simulation->outlets->exitsCount.at(i), realNode->arcs.at(i).to.lock()->id);

                        Simulation::displayImage(Grid2D<float>(*result.hitMap, realNode->GetOccupancy()), "result", simulationOptions.imageDisplayPower);
                        simulationOptions.simulationEnabled = true;
                    };
                    gsl->functionQueue.submit(lambda);
                }
                else
                    GSL_ERROR("No node corresponds to coords {}", selectedCoordinates);
            }

            ImGui::SetNextItemWidth(100);
            ImGui::DragFloat("Image color power", &simulationOptions.imageDisplayPower, 0.05, 0, 10);

            // Combine multiple maps
            // ------------------------
            if (ImGui::TreeNode("Arcs"))
            {
                if (selectedNode.node)
                {
                    for (size_t i = 0; i < selectedNode.node->arcs.size(); i++)
                    {
                        const Arc& arc = selectedNode.node->arcs.at(i);
                        ImGui::SetNextItemWidth(100);
                        ImGui::DragFloat(fmt::format("{}##{}", arc.to.lock()->id, i).c_str(), &selectedNode.combineWeights.at(i), 0.01, 0, 1);
                    }
                }
                ImGui::TreePop();
            }

            if (ImGui::Button("Combine results"))
            {
                auto lambda = [this]()
                {
                    simulationOptions.simulationEnabled = false;

                    auto realNode = As<RealNode>(selectedNode.node);
                    std::vector<float> combinedMap(realNode->GetOccupancy().metadata.dimensions.x * realNode->GetOccupancy().metadata.dimensions.y, 0);
                    for (size_t i = 0; i < selectedNode.node->arcs.size(); i++)
                    {
                        float weight = selectedNode.combineWeights.at(i);
                        if (weight <= 0)
                            continue;

                        const Arc& arc = selectedNode.node->arcs.at(i);

                        if (!gsl->simulationSystem.simulationCache.contains(arc.getUID()))
                            gsl->simulationSystem.SimulateFromArc(arc);

                        SimulationSystem::SimWithResult result = gsl->simulationSystem.simulationCache.at(arc.getUID());
                        for (size_t j = 0; j < combinedMap.size(); j++)
                            combinedMap.at(j) += result.hitMap->at(j) * weight;
                    }

                    Utils::Windsorize(combinedMap, 5);
                    Utils::PowerMaxNormalize(combinedMap, realNode->GetOccupancy().data, 1);
                    Simulation::displayImage(Grid2D<float>(combinedMap, realNode->GetOccupancy()));
                    simulationOptions.simulationEnabled = true;
                };
                gsl->functionQueue.submit(lambda);
            }

            ImGui::EndDisabled();
        }
        ImGui::End();

        ImGui::Begin("Simulation configuration");
        ImGui::SetNextItemWidth(100);
        ImGui::InputScalar("Min Warmup iterations", ImGuiDataType_U64, &gsl->simulationSystem.options.minWarmupIterations);

        ImGui::SetNextItemWidth(100);
        ImGui::InputScalar("Max Warmup iterations", ImGuiDataType_U64, &gsl->simulationSystem.options.maxWarmupIterations);

        ImGui::Checkbox("Cummulative map", &gsl->simulationSystem.options.cummulativeMap);
        ImGui::SetNextItemWidth(100);
        ImGui::DragFloat("Noise sigma", &gsl->simulationSystem.options.noiseSTDev, 0.01, 0, 1.0);
        ImGui::SetNextItemWidth(100);
        ImGui::DragFloat("Blur sigma", &gsl->simulationSystem.options.blurSigma, 0.01, 0, 2.0);
        ImGui::SetNextItemWidth(100);
        ImGui::DragFloat("Normalization power", &gsl->simulationSystem.options.normalizationPower, 0.01, 0, 5.0);

        ImGui::End();
    }
} // namespace GSL

#endif