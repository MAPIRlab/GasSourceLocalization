#pragma once
#ifdef USE_GUI
#include <gsl_server/algorithms/PMFS/internal/HitProbability.hpp>
#include <ament_imgui/ament_imgui.h>
#include <implot/implot.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/subscription.hpp>

#include <rclcpp/node.hpp>
#include <string>
#include <thread>

namespace GSL
{
    class PMFS;
}

namespace GSL::PMFS_internal
{

    class UI
    {

    public:
        UI(PMFS* _pmfs);
        ~UI();
        void run();
        void addConcentrationReading(double ppm);
    protected:
        void renderImgui();
        std::jthread renderThread;
        PMFS* pmfs;
        rclcpp::Node::SharedPtr node;
        double last_concentration_reading = 0;
        void createUI();
        void createPlots();
        bool useCoordinates();
        int selectVariable();
        std::string printCell(const Grid2D<HitProbability>& grid, const int& x, const int& y);
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clickedPointSub;
        Vector2 selectedCoordinates;
        Vector2 goalCoordinates;
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