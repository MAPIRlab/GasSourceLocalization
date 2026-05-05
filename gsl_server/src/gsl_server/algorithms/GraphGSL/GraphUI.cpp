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
            if (!Is<RoomNode>(node))
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
            if (node && selectedNode.node != node)
            {
                selectedNode.node = node;
                selectedNode.combineWeights = std::vector<float>(node->doorways.size(), 0);
            }

            std::string name = selectedNode.node ? selectedNode.node->id : "Null";
            ImGui::Text("Currently selected node: %s", name.c_str());

            ImGui::Checkbox("Simulate point", &simulationOptions.exactPoint);

            if (simulationOptions.exactPoint)
                ImGui::DragFloat2("Selected point", &selectedCoordinates.x, 0.02);
            else if (selectedNode.node)
            {
                if (selectedNode.node->doorways.size() == 0)
                    ImGui::Text("Node has no arcs!");
                else
                {
                    if (simulationOptions.selectedArcIdx > selectedNode.node->doorways.size())
                        simulationOptions.selectedArcIdx = 0;

                    ImGui::PushID("node");
                    if (ImGui::BeginCombo("Doorway", selectedNode.node->doorways.at(simulationOptions.selectedArcIdx).GetName().data()))
                    {
                        for (size_t i = 0; i < selectedNode.node->doorways.size(); i++)
                            if (ImGui::Selectable(selectedNode.node->doorways.at(i).GetName().data()))
                                simulationOptions.selectedArcIdx = i;

                        ImGui::EndCombo();
                    }
                    ImGui::PopID();
                }
            }
            else
            {
                ImGui::BeginDisabled();
                ImGui::BeginCombo("Doorway", "No node selected");
                ImGui::EndDisabled();
            }

            ImGui::BeginDisabled(!simulationOptions.simulationEnabled);
            if (ImGui::Button("Run single room simulation"))
            {
                auto roomNode = As<RoomNode>(selectedNode.node);
                if (roomNode)
                {
                    // Run
                    auto lambda = [this, roomNode]()
                    {
                        simulationOptions.simulationEnabled = false;
                        SimulationSystem::SimWithResult result;
                        if (simulationOptions.exactPoint)
                            result = gsl->simulationSystem.SimulateSingleRoomFromPoint(roomNode, selectedCoordinates);
                        else
                            result = gsl->simulationSystem.SimulateSingleRoomFromDoorway(roomNode->doorways.at(simulationOptions.selectedArcIdx));

                        // Log results
                        GSL_INFO("Emitted {} filaments during recording", result.simulation->totalEmittedFilaments);
                        for (size_t i = 0; i < result.simulation->outlets->exitsPerOutlet.size(); i++)
                            GSL_INFO("{} -> {}", result.ProportionInDoorway(i), roomNode->doorways.at(i).to.lock()->id);

                        Simulation::displayImage(Grid2D<float>(*result.hitMap, roomNode->GetOccupancy()), "result", simulationOptions.imageDisplayPower);
                        simulationOptions.simulationEnabled = true;
                    };
                    gsl->functionQueue.submit(lambda);
                }
                else
                    GSL_ERROR("No node corresponds to coords {}", selectedCoordinates);
            }

            if (ImGui::Button("Run whole map simulation"))
            {
                auto roomNode = As<RoomNode>(selectedNode.node);
                if (roomNode)
                {
                    // Run
                    auto lambda = [this, roomNode]()
                    {
                        simulationOptions.simulationEnabled = false;
                        // if (simulationOptions.exactPoint)
                        //     result = gsl->simulationSystem.SimulateSingleRoomFromPoint(roomNode, selectedCoordinates);
                        // else
                        gsl->simulationSystem.SimulateEntireGraphFromRoom(gsl->graph, roomNode);
                        gsl->nodeSelectedForVisualization = roomNode;
                        // Log results
                        GSL_INFO("Done simulating source in room '{}'", roomNode->id);

                        simulationOptions.simulationEnabled = true;
                    };
                    gsl->functionQueue.submit(lambda);
                }
                else
                    GSL_ERROR("No node corresponds to coords {}", selectedCoordinates);
            }

            if (ImGui::Button("Reset simulations"))
                gsl->simulationSystem.Reset();

            ImGui::SetNextItemWidth(100);
            ImGui::DragFloat("Image color power", &simulationOptions.imageDisplayPower, 0.05, 0, 10);

            // Combine multiple maps
            // ------------------------
            if (ImGui::TreeNode("Doorways"))
            {
                if (selectedNode.node)
                {
                    for (size_t i = 0; i < selectedNode.node->doorways.size(); i++)
                    {
                        const DoorwayNode& doorway = selectedNode.node->doorways.at(i);
                        ImGui::SetNextItemWidth(100);
                        ImGui::DragFloat(fmt::format("{}##{}", doorway.GetName(), i).c_str(), &selectedNode.combineWeights.at(i), 0.01, 0, 1);
                    }
                }
                ImGui::TreePop();
            }

            if (ImGui::Button("Combine results"))
            {
                auto lambda = [this]()
                {
                    simulationOptions.simulationEnabled = false;

                    auto roomNode = As<RoomNode>(selectedNode.node);
                    std::vector<float> combinedMap(roomNode->GetOccupancy().metadata.dimensions.x * roomNode->GetOccupancy().metadata.dimensions.y, 0);
                    for (size_t i = 0; i < selectedNode.node->doorways.size(); i++)
                    {
                        float weight = selectedNode.combineWeights.at(i);
                        if (weight <= 0)
                            continue;

                        const DoorwayNode& doorway = selectedNode.node->doorways.at(i);

                        if (!gsl->simulationSystem.simulationCache.contains(&doorway))
                            gsl->simulationSystem.SimulateSingleRoomFromDoorway(doorway);

                        SimulationSystem::SimWithResult result = gsl->simulationSystem.simulationCache.at(&doorway);
                        for (size_t j = 0; j < combinedMap.size(); j++)
                            combinedMap.at(j) += result.hitMap->at(j) * weight;
                    }

                    Utils::Windsorize(combinedMap, 5);
                    Utils::PowerMaxNormalize(combinedMap, roomNode->GetOccupancy().data, 1);
                    Simulation::displayImage(Grid2D<float>(combinedMap, roomNode->GetOccupancy()));
                    simulationOptions.simulationEnabled = true;
                };
                gsl->functionQueue.submit(lambda);
            }

            ImGui::EndDisabled();
        }
        ImGui::End();

        ImGui::Begin("Simulation configuration");
        ImGui::SetNextItemWidth(100);
        ImGui::InputScalar("Iterations", ImGuiDataType_U64, &gsl->simulationSystem.options.iterationLimit);

        ImGui::SetNextItemWidth(100);
        ImGui::InputScalar("Min Warmup iterations", ImGuiDataType_U64, &gsl->simulationSystem.options.minWarmupIterations);
        ImGui::SetNextItemWidth(100);
        ImGui::InputScalar("Max Warmup iterations", ImGuiDataType_U64, &gsl->simulationSystem.options.maxWarmupIterations);
        ImGui::SetNextItemWidth(100);
        ImGui::DragFloat("Delta time", &gsl->simulationSystem.options.deltaTime, 0.01, 0, 2);
        ImGui::SetNextItemWidth(100);
        ImGui::DragFloat("Warmup acceleration", &gsl->simulationSystem.options.warmupTimeAcc, 0.1, 0, 20);
        ImGui::SetNextItemWidth(100);
        ImGui::DragFloat("Filaments/second", &gsl->simulationSystem.options.filamentsPerSecond, 0.1, 0, 50);
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