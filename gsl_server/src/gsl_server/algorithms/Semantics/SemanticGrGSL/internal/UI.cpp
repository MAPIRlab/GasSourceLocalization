#include "gsl_server/algorithms/Semantics/SemanticGrGSL/MovingStateSemanticGrGSL.hpp"
#ifdef USE_GUI
#include "UI.hpp"
#include "gsl_server/algorithms/Common/Utils/RosUtils.hpp"
#include "gsl_server/algorithms/GrGSL/GrGSLLib.hpp"
#include "gsl_server/algorithms/GrGSL/GrGSL_internal.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gsl_server/algorithms/Common/GUI/ScrollingBuffer.hpp>
#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <gsl_server/algorithms/Common/Utils/Pointers.hpp>
#include <gsl_server/algorithms/Semantics/SemanticGrGSL/SemanticGrGSL.hpp>

using namespace GSL::GrGSL_internal;
namespace GSL::SemanticGrGSL_internal
{
    UI::UI(SemanticGrGSL* _grgsl)
        : grgsl(_grgsl)
    {
        uiNode = std::make_shared<rclcpp::Node>("UI");
        clickedPointSub = uiNode->create_subscription<geometry_msgs::msg::PointStamped>(
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
        imgui.Setup(
            fmt::format("{}/resources/GrGSL_imgui.ini", ament_index_cpp::get_package_share_directory("gsl_server")).c_str(),
            "GrGSL",
            900,
            600);
        ImPlot::CreateContext();

        rclcpp::Rate rate(30);

        while (rclcpp::ok() && !grgsl->HasEnded())
        {
            rclcpp::spin_some(uiNode);
            imgui.StartFrame();
            createUI();
            createPlots();

            imgui.Render();
            rate.sleep();
        }

        ImPlot::DestroyContext();
        imgui.Close();
    }

    void UI::createUI()
    {
        static Vector2Int selectedCell;
        ImGui::Begin("Queries");
        {
            if (UI::useCoordinates())
            {
                ImGui::InputFloat("X", &selectedCoordinates.x);
                ImGui::InputFloat("Y", &selectedCoordinates.y);
                auto indices = grgsl->gridMetadata.coordinatesToIndices(selectedCoordinates.x, selectedCoordinates.y);
                selectedCell = indices;
            }
            else
            {
                ImGui::InputInt("X", &selectedCell.x);
                ImGui::InputInt("Y", &selectedCell.y);
            }

            static std::string result;
            if (ImGui::Button("Print"))
            {
                if (!grgsl->gridMetadata.indicesInBounds(selectedCell))
                {
                    GSL_ERROR("Querying cell {}, which is outside the map!", selectedCell);
                    result = "Error! :(";
                }
                else
                    result = fmt::format("Probability of source in cell {0}: {1}\n", selectedCell, grgsl->cells[grgsl->gridMetadata.indexOf(selectedCell)].sourceProb);
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
            if (ImGui::Button("Simulate Infotaxis"))
            {
                simulateInfotaxis(selectedCell);
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


        ImGui::Begin("Current State");
        {
            grgsl->stateMachine.getCurrentState()->RenderUI();
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

    void UI::simulateInfotaxis(const Vector2Int& selectedCell)
    {
        const Grid2D<Cell> grid(grgsl->cells, grgsl->navigationOccupancy, grgsl->gridMetadata);
        std::vector<Vector2Int> index = {selectedCell};
        auto windVecs = As<GSL::MovingStateSemanticGrGSL>(grgsl->movingState)->getWindVectors(index);

        auto predictionCells = grid.data; // temp copy of the matrix of cells that we can modify to simulate the effect of a measurement
        auto accessProb = [](const Cell& cell)
        {
            return cell.sourceProb;
        };

        // simulate a hit in the considered position and see how much info that gives us
        GrGSLLib::estimateProbabilitiesfromGasAndWind(
            Grid2D<Cell>(predictionCells, grid.occupancy, grid.metadata),
            grgsl->settings,
            true,
            true,
            windVecs[0].angle,
            grgsl->positionOfLastHit,
            selectedCell);
        double infoHit = Utils::KLD<Cell>(predictionCells, grid.data, grid.occupancy, accessProb);

        std::vector<ColorRGBA> colors;
        const Grid2D<ColorRGBA> markerGrid(colors, grgsl->navigationOccupancy, grgsl->gridMetadata);
        colors.reserve(grgsl->cells.size());
        for (Cell& cell : predictionCells)
            colors.push_back(Utils::valueToColor(cell.sourceProb,
                                                 grgsl->settings.colorScaleLimits.x,
                                                 grgsl->settings.colorScaleLimits.y,
                                                 Utils::valueColorMode::Logarithmic));

        Utils::publishDebugMarkers(markerGrid, "simulatedInfotaxis");
        GSL_INFO("KLD: {}", infoHit);
    }

} // namespace GSL::GrGSL_internal

#endif