#include "gsl_server/algorithms/Common/Grid2D.hpp"
#include "gsl_server/algorithms/Common/Utils/RosUtils.hpp"
#include "gsl_server/algorithms/PMFS/internal/HitProbability.hpp"
#include "gsl_server/core/VectorsImpl/vmath_DDACustomVec.hpp"
#include "gsl_server/core/ros_typedefs.hpp"
#include "imgui.h"
#include <fmt/core.h>
#ifdef USE_GUI

// TODO This is currently just a straight-up copy of the PMFS UI. We could definitely do better, but I don't believe it is a good idea to set up an inheritance tree for UIs
// TODO it ends up limiting the design of the actual algorithm classes, and keeping that well organized is higher priority than making the debug UI easy to maintain

#include "UI.hpp"
#include "gsl_server/algorithms/Semantics/SemanticPMFS/SemanticPMFS.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <gsl_server/algorithms/PMFS/PMFSViz.hpp>
#include <gsl_server/algorithms/PMFS/internal/Simulations.hpp>

using namespace GSL::PMFS_internal;

namespace GSL::SemanticPMFS_internal
{
    UI::UI(SemanticPMFS* _pmfs)
        : pmfs(_pmfs)
    {
        clickedPointSub = pmfs->node->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 1,
            [this](const geometry_msgs::msg::PointStamped::SharedPtr point)
            {
                selectedCoordinates.x = point->point.x;
                selectedCoordinates.y = point->point.y;
                selectedCoordinates.z = point->point.z;
            });
    }

    UI::~UI()
    {
        renderThread.join();
    }

    void UI::run()
    {
        renderThread = std::jthread(std::bind(&UI::renderImgui, this));
    }

    void UI::renderImgui()
    {
        AmentImgui::Setup(
            fmt::format("{}/resources/SemanticPMFS_imgui.ini", ament_index_cpp::get_package_share_directory("gsl_server")).c_str(),
            "SemanticPMFS",
            900,
            600);
        ImPlot::CreateContext();

        rclcpp::Rate rate(30);

        while (rclcpp::ok() && !pmfs->HasEnded())
        {
            AmentImgui::StartFrame();
            createUI();
            createPlots();
            visualizeQueryPoint();
            AmentImgui::Render();
            rate.sleep();
        }

        ImPlot::DestroyContext();
        AmentImgui::Close();
    }

    void UI::createUI()
    {
        ImGui::Begin("Queries");
        {
            static int xInd = 0;
            static int yInd = 0;
            Variable selectedVar = selectVariable();

            // Select coordinates
            {
                ImGui::DragFloat("X", &selectedCoordinates.x, 0.03f);
                ImGui::DragFloat("Y", &selectedCoordinates.y, 0.03f);
                ImGui::DragFloat("Z", &selectedCoordinates.z, 0.03f);
                auto indices = pmfs->gridMetadata.coordinatesToIndices(selectedCoordinates.x, selectedCoordinates.y);
                xInd = indices.x;
                yInd = indices.y;
            }

            static std::string result;
            if (ImGui::Button("Print"))
            {
                if (!pmfs->gridMetadata.indicesInBounds({xInd, yInd}))
                {
                    GSL_ERROR("Querying cell {}, which is outside the map!", Vector2Int{xInd, yInd});
                    result = "Error! :(";
                }
                else
                    result = printCell(selectedVar);
            }
            if (ImGui::Button("Simulate leaf") && pmfs->gridMetadata.indicesInBounds({xInd, yInd}))
            {
                Utils::NQA::Node* leaf = pmfs->simulations.mapSegmentation[xInd][yInd];
                if (!leaf)
                    GSL_ERROR("Wrong coordinates!");
                else
                    pmfs->simulations.printImage(SimulationSource(leaf, pmfs->gridMetadata));
            }
            if (ImGui::Button("Simulate cell") && pmfs->gridMetadata.indicesInBounds({xInd, yInd}))
            {
                pmfs->simulations.printImage(SimulationSource(pmfs->gridMetadata.indicesToCoordinates(xInd, yInd), pmfs->gridMetadata));
            }

            ImGui::Text("%s", result.c_str());
        }
        ImGui::End();

        // TODO navigation debug
        //  ImGui::Begin("Goal");
        //  {
        //      static int x, y;
        //      ImGui::TextWrapped("Set a goal manually to be visited during the next movement phase.");
        //      if (useCoordinates())
        //      {
        //          ImGui::InputFloat("X", &goalCoordinates.x);
        //          ImGui::InputFloat("Y", &goalCoordinates.y);
        //          auto indices = pmfs->gridMetadata.coordinatesToIndices(goalCoordinates.x, goalCoordinates.y);
        //          x = indices.x;
        //          y = indices.y;
        //      }
        //      else
        //      {
        //          ImGui::InputInt("X", &x);
        //          ImGui::InputInt("Y", &y);
        //      }
        //      if (ImGui::Button("Go"))
        //      {
        //          pmfs->functionQueue.submit(std::bind(&MovingStatePMFS::debugMoveTo, dynamic_cast<MovingStatePMFS*>(pmfs->movingState.get()), x, y));
        //      }
        //  }
        //  ImGui::End();

        ImGui::Begin("Source Estimation Power");
        {
            ImGui::InputDouble("Source Power", &pmfs->settings.simulation.sourceDiscriminationPower);
        }
        ImGui::End();

        ImGui::Begin("Update Source");
        {
            if (ImGui::Button("Update Source Probability"))
            {
                pmfs->functionQueue.submit([this]()
                                           {
                    pmfs->simulations.updateSourceProbability(pmfs->settings.simulation.refineFraction);
                    PMFSViz::ShowSourceProb(
                        Grid2D<double>(pmfs->combinedSourceProbability, pmfs->simulationOccupancy, pmfs->gridMetadata), 
                        pmfs->settings.visualization, 
                        pmfs->pubs.pmfsPubs); });
            }
        }
        ImGui::End();
        ImGui::Begin("Quadtree");
        {
            if (ImGui::Button("Show Quadtree"))
            {
                PMFSViz::DebugMapSegmentation(pmfs->simulations.QTleaves, pmfs->pubs.pmfsPubs, pmfs->gridMetadata);
            }
        }
        ImGui::End();

        ImGui::Begin("Pause/Play");
        {
            static std::string buttonText;
            if (pmfs->paused)
                buttonText = "Play";
            else
                buttonText = "Pause";
            if (ImGui::Button(buttonText.c_str()))
            {
                pmfs->paused = !pmfs->paused;
            }
        }
        ImGui::End();

        ImGui::Begin("Markers");
        {
            auto& settings = pmfs->settings;
            static int modeHit = settings.visualization.hitMode;
            static int modeSource = settings.visualization.sourceMode;
            static float hitLimits[2] = {settings.visualization.hitLimits.x, settings.visualization.hitLimits.y};
            static float sourceLimits[2] = {settings.visualization.sourceLimits.x, settings.visualization.sourceLimits.y};

            ImGui::Combo("Hit display mode", &modeHit, "Linear\0Logarithmic\0");
            settings.visualization.hitMode = modeHit == 0 ? Utils::valueColorMode::Linear : Utils::valueColorMode::Logarithmic;
            ImGui::InputFloat2("Hit limits", hitLimits, "%.5f");

            ImGui::Combo("Source display mode", &modeSource, "Linear\0Logarithmic\0");
            settings.visualization.sourceMode = modeSource == 0 ? Utils::valueColorMode::Linear : Utils::valueColorMode::Logarithmic;
            ImGui::InputFloat2("Source limits", sourceLimits, "%.5f");

            settings.visualization.hitLimits.x = hitLimits[0];
            settings.visualization.hitLimits.y = hitLimits[1];
            settings.visualization.sourceLimits.x = sourceLimits[0];
            settings.visualization.sourceLimits.y = sourceLimits[1];
        }
        ImGui::End();
    }

    void UI::createPlots()
    {
        ImGui::Begin("Plots");
        {
            static bool paused = false;

            if (ImGui::Button("Toggle Pause"))
                paused = !paused;

            ImGui::BulletText("Gas concentration measured over time");
            static ScrollingBuffer sdata1;
            static float t = 0;
            t += ImGui::GetIO().DeltaTime;
            if (last_concentration_reading != -1)
                sdata1.AddPoint(t, last_concentration_reading);

            static float history = 10.0f;
            ImGui::SliderFloat("History", &history, 1, 30, "%.1f s");

            static ImPlotAxisFlags flags = ImPlotAxisFlags_AutoFit;
            static ImVec2 xAxisLimits;
            if (!paused)
                xAxisLimits = {t - history, t};

            if (ImPlot::BeginPlot("##Concentration", ImVec2(-1, -1)))
            {
                ImPlot::SetupAxes(NULL, NULL, ImPlotAxisFlags_NoGridLines, flags);
                ImPlot::SetupAxisLimits(ImAxis_X1, xAxisLimits.x, xAxisLimits.y, paused ? ImGuiCond_None : ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1);
                ImPlot::SetNextLineStyle({1, 0, 0, 1}, 2);
                if (sdata1.Data.size() > 0)
                    ImPlot::PlotLine("Concentration", &sdata1.Data[0].x, &sdata1.Data[0].y, sdata1.Data.size(), 0, sdata1.IndexOfLast,
                                     2 * sizeof(float));
                ImPlot::EndPlot();
            }
        }
        ImGui::End();
    }

    UI::Variable UI::selectVariable()
    {
        static int selected = 0;
        ImGui::Combo("Select Variable", &selected, "Hit\0Source\0Semantics\0");
        return static_cast<Variable>(selected);
    }

    std::string UI::printCell(Variable variable)
    {
        Vector2Int indices = pmfs->gridMetadata.coordinatesToIndices(selectedCoordinates.x, selectedCoordinates.y);
        size_t index = pmfs->gridMetadata.indexOf({indices.x, indices.y});
        static std::string queryResult;
        if (variable == Variable::HitProb)
        {
            Grid2D<HitProbability> grid(pmfs->hitProbability, pmfs->simulationOccupancy, pmfs->gridMetadata);
            queryResult = fmt::format("Cell {0},{1}:\n", indices.x, indices.y) + fmt::format("free:{} \n", grid.freeAt(indices.x, indices.y)) +
                          fmt::format("Hit probability:{:.3f} \n", Utils::logOddsToProbability(grid.dataAt(indices.x, indices.y).logOdds));
        }
        else if (variable == Variable::SourceProb)
        {
            queryResult = fmt::format("Cell {0},{1}: \n", indices.x, indices.y) +
                          fmt::format("Total probability: {}\n", pmfs->combinedSourceProbability[index]) +
                          fmt::format("Olfaction probability: {}\n", pmfs->sourceProbabilityPMFS[index]) +
                          fmt::format("Semantics probability: {}", pmfs->sourceProbSemantics[index]);
        }
        else if (variable == Variable::Semantics)
        {
            queryResult = pmfs->semantics->GetDebugInfo(selectedCoordinates);
        }

        return queryResult.c_str();
    }

    void UI::visualizeQueryPoint()
    {
        static rclcpp::Publisher<Marker>::SharedPtr pub = pmfs->node->create_publisher<Marker>("UIQueryPoint", 1);
        Marker marker;
        marker.header.frame_id ="map";
        marker.header.stamp = pmfs->node->now();
        marker.pose.position.x = selectedCoordinates.x;
        marker.pose.position.y = selectedCoordinates.y;
        marker.pose.position.z = selectedCoordinates.z;
        marker.type = Marker::SPHERE;
        marker.color = Utils::create_color(1, 0, 0, 1);
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        pub->publish(marker);
    }

    void UI::addConcentrationReading(double ppm)
    {
        last_concentration_reading = ppm;
    }

} // namespace GSL::SemanticPMFS_internal

#endif