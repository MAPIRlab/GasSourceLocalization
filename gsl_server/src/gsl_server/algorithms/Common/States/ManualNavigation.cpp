#include "ManualNavigation.hpp"
#include <gsl_server/algorithms/Common/Algorithm.hpp>
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>

GSL::ManualNavigationState::ManualNavigationState(Algorithm* _algorithm)
    : MovingState(_algorithm, false)
{
}

void GSL::ManualNavigationState::chooseGoalAndMove()
{
    algorithm->stateMachine.forceSetState(this);
}

void GSL::ManualNavigationState::OnEnterState(State* previous)
{
#if USE_GUI
    GSL_INFO("Entering Manual Driving, use the 'Continue' button to resume algorithm execution");
    paused = true;
    rclcpp::Rate rate(20);
    while (paused)
    {
        algorithm->OnUpdate();
        rate.sleep();
    }
    
#else
    auto exec = Utils::createExecutor(algorithm->node);
    std::jthread spinThread([&]()
                            {
                                exec->spin();
                            });

    GSL_INFO("Entering Manual Driving, press [enter] to resume algorithm execution");
    std::cin.get();
    exec->cancel();
    spinThread.join();
#endif

    GSL_INFO("Resuming execution");
    algorithm->OnCompleteNavigation(GSLResult::Success, previousState);
}

#if USE_GUI
#include <imgui.h>
void GSL::ManualNavigationState::RenderUI()
{
    ImGui::Text("Manual Navigation State");
    if (ImGui::Button("Continue"))
        paused = false;
}
#endif