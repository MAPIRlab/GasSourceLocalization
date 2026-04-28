#pragma once
#include "geometry_msgs/msg/point_stamped.hpp"
#include "gsl_server/core/VectorsImpl/vmath_DDACustomVec.hpp"
#include <rclcpp/subscription.hpp>
#if USE_GUI

#include <rclcpp/publisher.hpp>
#include <thread>

namespace GSL
{
    class GraphGSL;

    class GraphUI
    {
    public:
        GraphUI(GraphGSL* _gsl);
        ~GraphUI();

        void Run();
        void RenderImgui();

    private:
        void CreateUI();
        void SelectNodes();
        void SimulateSourceMenu();

        GraphGSL* gsl;
        std::jthread renderThread;

        bool occupancyToggleState = true;

        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clickedPointSub;
        Vector2 selectedCoordinates;

        struct SimulationOptions
        {
            bool exactPoint = false;
            size_t selectedArcIdx = 0;
            float imageDisplayPower = 1;
            bool simulationEnabled = true;
        } simulationOptions;

        struct SelectedNode
        {
            std::shared_ptr<class PlaceNode> node;
            std::vector<float> combineWeights;
        } selectedNode;
    };
} // namespace GSL

#endif