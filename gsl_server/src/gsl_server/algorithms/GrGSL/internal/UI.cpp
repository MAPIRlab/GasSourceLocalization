#ifdef USE_GUI
#include "gsl_server/algorithms/GrGSL/GrGSLLib.hpp"
#include "imgui.h"

#include "UI.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <gsl_server/algorithms/Common/GUI/ScrollingBuffer.hpp>
#include <gsl_server/algorithms/GrGSL/GrGSL.hpp>

namespace GSL::GrGSL_internal
{
    UI::UI(GrGSL* _grgsl)
        : grgsl(_grgsl)
    {
        clickedPointSub = grgsl->node->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 1,
            [this](const geometry_msgs::msg::PointStamped::SharedPtr point)
            {
                selectedCoordinates.x = point->point.x;
                selectedCoordinates.y = point->point.y;
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
            fmt::format("{}/resources/GrGSL_imgui.ini", ament_index_cpp::get_package_share_directory("gsl_server")).c_str(),
            "PMFS",
            900,
            600);
        ImPlot::CreateContext();

        rclcpp::Rate rate(30);

        while (rclcpp::ok() && !grgsl->HasEnded())
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

            if (UI::useCoordinates())
            {
                ImGui::InputFloat("X", &selectedCoordinates.x);
                ImGui::InputFloat("Y", &selectedCoordinates.y);
                auto indices = grgsl->gridMetadata.coordinatesToIndices(selectedCoordinates.x, selectedCoordinates.y);
                x = indices.x;
                y = indices.y;
            }
            else
            {
                ImGui::InputInt("X", &x);
                ImGui::InputInt("Y", &y);
            }

            static std::string result;
            if (ImGui::Button("Print"))
            {
                if (!grgsl->gridMetadata.indicesInBounds({x, y}))
                {
                    GSL_ERROR("Querying cell {}, which is outside the map!", Vector2Int{x, y});
                    result = "Error! :(";
                }
                else
                    result = fmt::format("Probability of source in cell {0},{1}: {2}\n", x, y, grgsl->cells[grgsl->gridMetadata.indexOf({x, y})].sourceProb);
            }

            ImGui::Text("%s", result.c_str());
        }
        ImGui::End();

        // ImGui::Begin("Goal");
        // {
        //     static int x, y;
        //     ImGui::TextWrapped("Set a goal manually to be visited during the next movement phase.");
        //     if (UI::useCoordinates())
        //     {
        //         ImGui::InputFloat("X", &goalCoordinates.x);
        //         ImGui::InputFloat("Y", &goalCoordinates.y);
        //         auto indices = grgsl->gridMetadata.coordinatesToIndices(goalCoordinates.x, goalCoordinates.y);
        //         x = indices.x;
        //         y = indices.y;
        //     }
        //     else
        //     {
        //         ImGui::InputInt("X", &x);
        //         ImGui::InputInt("Y", &y);
        //     }
        //     if (ImGui::Button("Go"))
        //     {
        //         grgsl->functionQueue.submit(std::bind(&MovingStatePMFS::debugMoveTo, dynamic_cast<MovingStatePMFS*>(grgsl->movingState.get()), x, y));
        //     }
        // }
        // ImGui::End();


        ImGui::Begin("Pause/Play");
        {
            static std::string buttonText;
            if (grgsl->paused)
                buttonText = "Play";
            else
                buttonText = "Pause";
            if (ImGui::Button(buttonText.c_str()))
            {
                grgsl->paused = !grgsl->paused;
            }
            
            ImGui::Checkbox("Debug Propagation", &GrGSLLib::debuggingPropagation);
        }
        ImGui::End();

        ImGui::Begin("Markers");
        {
            auto& settings = grgsl->settings;
            static float sourceLimits[2] = {settings.colorScaleLimits.x, settings.colorScaleLimits.y};

            ImGui::InputFloat2("Hit limits", sourceLimits, "%.5f");

            settings.colorScaleLimits.x = sourceLimits[0];
            settings.colorScaleLimits.y = sourceLimits[1];
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
            static GUI::ScrollingBuffer sdata1;
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

    bool UI::useCoordinates()
    {
        static int selected = 0;
        ImGui::Combo("Use", &selected, "Coordinates\0Indices\0");
        return selected == 0;
    }

    std::string UI::printCell(const Grid2D<Cell>& grid, const int& x, const int& y)
    {
        std::string queryResult;

        queryResult = fmt::format("Cell {0},{1}:\n", x, y) + fmt::format("free:{} \n", grid.freeAt(x, y)) +
                      fmt::format("auxWeight:{} \n", Utils::logOddsToProbability(grid.dataAt(x, y).auxWeight)) +
                      fmt::format("weight:{} \n", Utils::logOddsToProbability(grid.dataAt(x, y).sourceProb));

        return queryResult.c_str();
    }

    void UI::addConcentrationReading(double ppm)
    {
        last_concentration_reading = ppm;
    }

} // namespace GSL::PMFS_internal

#endif