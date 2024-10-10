#pragma once
#ifdef USE_GUI
    #include "gsl_server/algorithms/Common/Grid2D.hpp"
    #include "gsl_server/algorithms/PMFS/internal/PublishersAndSubscribers.hpp"
    #include "gsl_server/algorithms/PMFS/internal/Settings.hpp"
    #include "gsl_server/algorithms/PMFS/internal/Simulations.hpp"
    #include "gsl_server/core/FunctionQueue.hpp"
    #include <ament_imgui/ament_imgui.h>
    #include <gsl_server/algorithms/PMFS/internal/HitProbability.hpp>
    #include <implot/implot.h>

    #include <geometry_msgs/msg/point_stamped.hpp>
    #include <rclcpp/subscription.hpp>

    #include <string>
    #include <thread>

namespace GSL
{
    class PMFS;

    #if ENABLE_SEMANTIC_PMFS
    class SemanticPMFS;
    #endif
} // namespace GSL

namespace GSL::PMFS_internal
{
    class UI
    {
    public:
        UI(PMFS* _algo);
    #if ENABLE_SEMANTIC_PMFS
        UI(SemanticPMFS* _algo);
    #endif
        ~UI();
        void run();
        void addConcentrationReading(double ppm);

    protected:
        // PMFS things
        class Algorithm* algorithm;
        Grid2D<HitProbability> hitProb;
        Grid2D<double> sourceProb;
        Simulations& simulations;
        VisualizationSettings& visualizationSettings;
        SimulationSettings& simulationSettings;
        PublishersAndSubscribers& pubs;
        bool& paused;
        FunctionQueue& functionQueue;

        // internal UI state
        std::jthread renderThread;
        double last_concentration_reading = 0;
        Vector2 selectedCoordinates;
        Vector2 goalCoordinates;

        void renderImgui();
        void createUI();
        void createPlots();
        bool useCoordinates();
        int selectVariable();
        std::string printCell(const Grid2D<HitProbability>& grid, const int& x, const int& y);
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clickedPointSub;
    };

    // utility structure for realtime plot
    struct ScrollingBuffer
    {
        int MaxSize;
        int IndexOfLast;
        ImVector<ImVec2> Data;
        ScrollingBuffer(int max_size = 2000)
        {
            MaxSize = max_size;
            IndexOfLast = 0;
            Data.reserve(MaxSize);
        }
        void AddPoint(float x, float y)
        {
            if (Data.size() < MaxSize)
                Data.push_back(ImVec2(x, y));
            else
            {
                Data[IndexOfLast] = ImVec2(x, y);
                IndexOfLast = (IndexOfLast + 1) % MaxSize;
            }
        }
        void Erase()
        {
            if (Data.size() > 0)
            {
                Data.shrink(0);
                IndexOfLast = 0;
            }
        }
    };
} // namespace GSL::PMFS_internal

#endif