#include "SimulationSystem.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"
#include "gsl_server/algorithms/Common/Utils/Pointers.hpp"
#include "gsl_server/algorithms/Common/Utils/RosUtils.hpp"
#include "gsl_server/algorithms/GraphGSL/Node.hpp"
#if USE_GUI

#include "GraphUI.hpp"
#include "gsl_server/algorithms/GraphGSL/GraphGSL.hpp"
#include "imgui_gl/imgui_gl.h"
#include "imgui_gl/utils.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <implot/implot.h>
#include <rclcpp/rclcpp.hpp>

using namespace GSL::Graph_internal;
namespace GSL
{
    GraphUI::GraphUI(GraphGSL* _gsl) : gsl(_gsl)
    {
        clickedPointSub =
            gsl->rclnode->create_subscription<geometry_msgs::msg::PointStamped>(
                "/clicked_point", 1,
                [this](const geometry_msgs::msg::PointStamped::SharedPtr point)
                {
                    selectedCoordinates.x = point->point.x;
                    selectedCoordinates.y = point->point.y;
                    size_t nodeIndex = gsl->graph.GetCorrespondingNodeIdx(selectedCoordinates);
                    if (selectedNodeData.nodeIndex != nodeIndex)
                        OnSelectNode(nodeIndex);
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
            size_t previousIndex = selectedNodeData.nodeIndex;
            ImGui::SetNextItemWidth(120);
            ImGui::ComboSelect("Selected Node", gsl->graph.nodes, selectedNodeData.nodeIndex, [](auto& node)
                               {
                                   return node->id;
                               });
            auto node = gsl->graph.nodes.at(selectedNodeData.nodeIndex);
            if (selectedNodeData.nodeIndex != previousIndex)
            {
                selectedCoordinates = node->GetPosition();
                OnSelectNode(selectedNodeData.nodeIndex);
            }

            std::string name = node ? node->id : "Null";
            ImGui::Text("Currently selected node: %s", name.c_str());

            if (node && gsl->simulationSystem.gasMapsWithRoomSource.contains(node))
            {
                std::deque<CompleteMap>& gasMaps = gsl->simulationSystem.gasMapsWithRoomSource.at(node);
                ImGui::SetNextItemWidth(120);
                ImGui::ComboSelect("Visualize simulation", gasMaps, gsl->simulationViz.simulationIndex,
                                   [](const auto& map)
                                   {
                                       return fmt::format("{}", map.source->GetPoint());
                                   });
            }

            ImGui::BeginDisabled(!simulationOptions.simulationEnabled);

            if (ImGui::Button("Run whole map simulation"))
            {
                if (node)
                {
                    auto lambda = [this, node]()
                    {
                        simulationOptions.simulationEnabled = false;
                        Vector2 pos = selectedCoordinates;
                        if (!Is<RoomNode>(node))
                            pos = node->GetPosition();
                        gsl->simulationSystem.SimulateEntireGraph(node, pos);
                        GSL_INFO("Done simulating source in room '{}'", node->id);
                        gsl->simulationViz.simulationIndex = gsl->simulationSystem.gasMapsWithRoomSource.at(node).size() - 1;
                        simulationOptions.simulationEnabled = true;
                    };
                    gsl->functionQueue.submit(lambda);
                }
                else
                    GSL_ERROR("No node corresponds to coords {}", selectedCoordinates);
            }

            if (ImGui::Button("Reset simulations"))
                gsl->simulationSystem.Reset();

            if (ImGui::Button("Evaluate Source Probs"))
            {
                gsl->functionQueue.submit([this]()
                                          {
                                              simulationOptions.simulationEnabled = false;
                                              gsl->EvaluateRoomProbabilities();
                                              simulationOptions.simulationEnabled = true;
                                          });
            }
            ImGui::SetNextItemWidth(100);
            ImGui::InputFloat("Likelihood Sigma", &gsl->likelihoodSigma);

#if ENABLE_NAIVE_EVALUATION
            if (ImGui::Button("Naive Source Probs"))
            {
                gsl->functionQueue.submit([this]()
                                          {
                                              simulationOptions.simulationEnabled = false;
                                              gsl->EvaluateRoomProbabilitiesNaive();
                                              simulationOptions.simulationEnabled = true;
                                          });
            }
            ImGui::SetNextItemWidth(120);
            ImGui::ComboSelect("Naive simulation viz", gsl->naiveCompleteMaps, gsl->naiveSimulationIndex, [](auto& map)
                               {
                                   return fmt::format("{}", map.source->GetPoint());
                               });
#endif

            ImGui::EndDisabled();
        }
        ImGui::End();

        ImGui::Begin("Simulate single room");
        SimulateSingleRoomMenu();
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

    void GraphUI::SimulateSingleRoomMenu()
    {
        auto node = gsl->graph.nodes.at(selectedNodeData.nodeIndex);

        ImGui::Checkbox("Simulate point", &simulationOptions.exactPoint);

        if (simulationOptions.exactPoint)
            ImGui::DragFloat2("Selected point", &selectedCoordinates.x, 0.02);
        else if (node)
        {
            if (node->doorways.size() == 0)
                ImGui::Text("Node has no arcs!");
            else
            {
                if (simulationOptions.selectedArcIdx > node->doorways.size())
                    simulationOptions.selectedArcIdx = 0;

                ImGui::PushID("node");
                ImGui::SetNextItemWidth(120);
                ImGui::ComboSelect("Doorway", node->doorways, simulationOptions.selectedArcIdx, [](auto& door)
                                   {
                                       return door->GetName();
                                   });
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
            auto roomNode = As<RoomNode>(node);
            if (roomNode)
            {
                // Run
                auto lambda = [this, roomNode]()
                {
                    simulationOptions.simulationEnabled = false;
                    SimWithResult result;
                    if (simulationOptions.exactPoint)
                        result = gsl->simulationSystem.SimulateSingleRoomFromPoint(roomNode, selectedCoordinates);
                    else
                        result = gsl->simulationSystem.SimulateSingleRoomFromDoorway(roomNode->doorways.at(simulationOptions.selectedArcIdx));

                    // Log results
                    GSL_INFO("Emitted {} filaments during recording", result.simulation->totalEmittedFilaments);
                    for (size_t i = 0; i < result.simulation->outlets->exitsPerOutlet.size(); i++)
                        GSL_INFO("{} -> {}", result.ProportionInDoorway(i), roomNode->doorways.at(i)->to.lock()->id);

                    Simulation::displayImage(Grid2D<float>(*result.hitMap, roomNode->GetOccupancy()), "result");
                    simulationOptions.simulationEnabled = true;
                };
                gsl->functionQueue.submit(lambda);
            }
            else
                GSL_ERROR("No node corresponds to coords {}", selectedCoordinates);
        }

        // Combine multiple maps
        // ------------------------
        if (ImGui::TreeNode("Doorways"))
        {
            if (node)
            {
                for (size_t i = 0; i < node->doorways.size(); i++)
                {
                    const auto doorway = node->doorways.at(i);
                    ImGui::SetNextItemWidth(100);
                    ImGui::DragFloat(fmt::format("{}##{}", doorway->GetName(), i).c_str(), &selectedNodeData.combineWeights.at(i), 0.01, 0, 1);
                }
            }
            ImGui::TreePop();
        }

        if (ImGui::Button("Combine results"))
        {
            auto lambda = [this]()
            {
                auto node = gsl->graph.nodes.at(selectedNodeData.nodeIndex);
                simulationOptions.simulationEnabled = false;

                auto roomNode = As<RoomNode>(node);
                std::vector<float> combinedMap(roomNode->GetOccupancy().metadata.dimensions.x * roomNode->GetOccupancy().metadata.dimensions.y, 0);
                for (size_t i = 0; i < node->doorways.size(); i++)
                {
                    float weight = selectedNodeData.combineWeights.at(i);
                    if (weight <= 0)
                        continue;

                    const auto doorway = node->doorways.at(i);

                    SimWithResult result = gsl->simulationSystem.simulationCache.Get(doorway);
                    for (size_t j = 0; j < combinedMap.size(); j++)
                        combinedMap.at(j) += result.hitMap->at(j) * weight;
                }

                Utils::Winsorize(combinedMap, 5);
                Utils::PowerMaxNormalize(combinedMap, roomNode->GetOccupancy().data, 1);
                Simulation::displayImage(Grid2D<float>(combinedMap, roomNode->GetOccupancy()));
                simulationOptions.simulationEnabled = true;
            };
            gsl->functionQueue.submit(lambda);
        }
        ImGui::EndDisabled();
    }

    void GraphUI::OnSelectNode(size_t nodeIndex)
    {
        selectedNodeData.nodeIndex = nodeIndex;
        auto node = gsl->graph.nodes.at(nodeIndex);
        selectedNodeData.combineWeights = std::vector<float>(node->doorways.size(), 0);
        gsl->simulationViz.selectedNode = node;
    }
} // namespace GSL

#endif