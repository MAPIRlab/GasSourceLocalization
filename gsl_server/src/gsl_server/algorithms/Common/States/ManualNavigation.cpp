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
    paused = true;
}

void GSL::ManualNavigationState::OnUpdate()
{
    if (!paused)
    {
        GSL_INFO("Resuming execution");
        algorithm->OnCompleteNavigation(GSLResult::Success, previousState);
    }
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