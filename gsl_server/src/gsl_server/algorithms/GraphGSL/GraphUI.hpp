#pragma once
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

        GraphGSL* gsl;
        std::jthread renderThread;

        bool occupancyToggleState = true;
    };
} // namespace GSL

#endif