#ifdef USE_GUI

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <gsl_server/algorithms/PMFS/PMFS.hpp>
#include <gsl_server/algorithms/PMFS/PMFSViz.hpp>
#include <gsl_server/algorithms/PMFS/internal/Simulations.hpp>
#include <gsl_server/algorithms/PMFS/internal/UI.hpp>

#if ENABLE_SEMANTIC_PMFS
#include <gsl_server/algorithms/Semantics/SemanticPMFS/SemanticPMFS.hpp>
#endif

namespace GSL::PMFS_internal
{
    UI::UI(PMFS* _pmfs)
        : algorithm(_pmfs),
          hitProb(_pmfs->hitProbability, _pmfs->occupancy, _pmfs->gridMetadata),
          sourceProb(_pmfs->sourceProbability, _pmfs->occupancy, _pmfs->gridMetadata),
          simulations(_pmfs->simulations),
          visualizationSettings(_pmfs->settings.visualization),          
          simulationSettings(_pmfs->settings.simulation),
          pubs(_pmfs->pubs),
          paused(_pmfs->paused),
          functionQueue(_pmfs->functionQueue)
    {
        clickedPointSub = _pmfs->node->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point",
            1,
            [this](const geometry_msgs::msg::PointStamped::SharedPtr point)
            {
                selectedCoordinates.x = point->point.x;
                selectedCoordinates.y = point->point.y;
            });
    }

#if ENABLE_SEMANTIC_PMFS
    UI::UI(SemanticPMFS* _pmfs)
        : algorithm(_pmfs),
          hitProb(_pmfs->hitProbability, _pmfs->simulationOccupancy, _pmfs->gridMetadata),
          sourceProb(_pmfs->combinedSourceProbability, _pmfs->simulationOccupancy, _pmfs->gridMetadata),
          simulations(_pmfs->simulations),
          visualizationSettings(_pmfs->settings.visualization),          
          simulationSettings(_pmfs->settings.simulation),
          pubs(_pmfs->pubs.pmfsPubs),
          paused(_pmfs->paused),
          functionQueue(_pmfs->functionQueue)
    {
        clickedPointSub = _pmfs->node->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point",
            1,
            [this](const geometry_msgs::msg::PointStamped::SharedPtr point)
            {
                selectedCoordinates.x = point->point.x;
                selectedCoordinates.y = point->point.y;
            });
    }
#endif

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
            fmt::format("{}/resources/imgui.ini",
                        ament_index_cpp::get_package_share_directory("gsl_server"))
                .c_str(),
            "PMFS",
            900,
            600);
        ImPlot::CreateContext();

        rclcpp::Rate rate(30);

        while (rclcpp::ok() && !algorithm->HasEnded())
        {
            AmentImgui::StartFrame();
            createUI();
            createPlots();

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
            static int x = 0;
            static int y = 0;
            int selectedVar = PMFS_internal::UI::selectVariable();

            if (PMFS_internal::UI::useCoordinates())
            {
                ImGui::InputFloat("X", &selectedCoordinates.x);
                ImGui::InputFloat("Y", &selectedCoordinates.y);
                auto indices = hitProb.metadata.coordinatesToIndices(selectedCoordinates.x, selectedCoordinates.y);
                x = indices.x;
                y = indices.y;
            }
            else
            {
                ImGui::InputInt("X", &x);
                ImGui::InputInt("Y", &y);
            }

            static std::string result;
            if (ImGui::Button("Print") && hitProb.metadata.indicesInBounds({x, y}))
            {
                if (selectedVar == 0)
                    result = PMFS_internal::UI::printCell(hitProb, x, y);
                else if (selectedVar == 1)
                    result = fmt::format("Cell {0},{1}: {2}\n", x, y, sourceProb.dataAt(x, y));
            }
            if (ImGui::Button("Simulate leaf") && hitProb.metadata.indicesInBounds({x, y}))
            {
                Utils::NQA::Node* leaf = simulations.mapSegmentation[x][y];
                if (!leaf)
                    GSL_ERROR("Wrong coordinates!");
                else
                    simulations.printImage(SimulationSource(leaf, hitProb.metadata));
            }
            else if (ImGui::Button("Simulate cell") && hitProb.metadata.indicesInBounds({x, y}))
            {
                simulations.printImage(SimulationSource(hitProb.metadata.indicesToCoordinates(x, y), hitProb.metadata));
            }

            ImGui::Text("%s", result.c_str());
        }
        ImGui::End();

        ImGui::Begin("Goal");
        {
            static int x, y;
            ImGui::TextWrapped("Set a goal manually to be visited during the next movement phase.");
            if (PMFS_internal::UI::useCoordinates())
            {
                ImGui::InputFloat("X", &goalCoordinates.x);
                ImGui::InputFloat("Y", &goalCoordinates.y);
                auto indices = hitProb.metadata.coordinatesToIndices(goalCoordinates.x, goalCoordinates.y);
                x = indices.x;
                y = indices.y;
            }
            else
            {
                ImGui::InputInt("X", &x);
                ImGui::InputInt("Y", &y);
            }
        }
        ImGui::End();

        ImGui::Begin("Source Estimation Power");
        {
            ImGui::InputDouble("Source Power", &simulationSettings.sourceDiscriminationPower);
        }
        ImGui::End();

        ImGui::Begin("Update Source");
        {
            if (ImGui::Button("Update Source Probability"))
            {
                functionQueue.submit([this]()
                                     { simulations.updateSourceProbability(simulationSettings.refineFraction); });
            }
        }
        ImGui::End();
        ImGui::Begin("Quadtree");
        {
            if (ImGui::Button("Show Quadtree"))
            {
                PMFSViz::DebugMapSegmentation(simulations.QTleaves, pubs, hitProb.metadata);
            }
        }
        ImGui::End();

        ImGui::Begin("Pause/Play");
        {
            static std::string buttonText;
            if (paused)
                buttonText = "Play";
            else
                buttonText = "Pause";

            if (ImGui::Button(buttonText.c_str()))
                paused = !paused;
        }
        ImGui::End();

        ImGui::Begin("Markers");
        {
            static int modeHit = visualizationSettings.hitMode;
            static int modeSource = visualizationSettings.sourceMode;
            static float hitLimits[2] = {visualizationSettings.hitLimits.x, visualizationSettings.hitLimits.y};
            static float sourceLimits[2] = {visualizationSettings.sourceLimits.x, visualizationSettings.sourceLimits.y};

            ImGui::Combo("Hit display mode", &modeHit, "Linear\0Logarithmic\0");
            visualizationSettings.hitMode = modeHit == 0 ? Utils::valueColorMode::Linear : Utils::valueColorMode::Logarithmic;
            ImGui::InputFloat2("Hit limits", hitLimits, "%.5f");

            ImGui::Combo("Source display mode", &modeSource, "Linear\0Logarithmic\0");
            visualizationSettings.sourceMode = modeSource == 0 ? Utils::valueColorMode::Linear : Utils::valueColorMode::Logarithmic;
            ImGui::InputFloat2("Source limits", sourceLimits, "%.5f");

            visualizationSettings.hitLimits.x = hitLimits[0];
            visualizationSettings.hitLimits.y = hitLimits[1];
            visualizationSettings.sourceLimits.x = sourceLimits[0];
            visualizationSettings.sourceLimits.y = sourceLimits[1];
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
            static PMFS_internal::ScrollingBuffer sdata1;
            static float t = 0;

            // add data point
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

    bool UI::useCoordinates()
    {
        static int selected = 0;
        ImGui::Combo("Use", &selected, "Coordinates\0Indices\0");
        return selected == 0;
    }

    int UI::selectVariable()
    {
        static int selected = 0;
        ImGui::Combo("Select Variable", &selected, "Hit\0Source\0");
        return selected;
    }

    std::string UI::printCell(const Grid2D<HitProbability>& grid, const int& x, const int& y)
    {
        static std::string queryResult;

        if (grid.metadata.indicesInBounds({x, y}))
        {
            GSL_ERROR("Querying cell outside the map!");
        }
        else
        {
            queryResult = fmt::format("Cell {0},{1}:\n", x, y) + fmt::format("free:{} \n", grid.freeAt(x, y)) +
                          fmt::format("auxWeight:{} \n", Utils::logOddsToProbability(grid.dataAt(x, y).auxWeight)) +
                          fmt::format("weight:{} \n", Utils::logOddsToProbability(grid.dataAt(x, y).logOdds));
        }

        return queryResult.c_str();
    }

    void UI::addConcentrationReading(double ppm)
    {
        last_concentration_reading = ppm;
    }

} // namespace GSL::PMFS_internal

#endif