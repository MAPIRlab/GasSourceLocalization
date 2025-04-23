#include "NoNavigation.hpp"
#include "gsl_server/algorithms/Common/Algorithm.hpp"
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>

GSL::NoNavigationState::NoNavigationState(Algorithm* _algorithm)
    : MovingState(_algorithm, false)
{
}

void GSL::NoNavigationState::chooseGoalAndMove()
{
    algorithm->stateMachine.forceSetState(this);
}

void GSL::NoNavigationState::OnEnterState(State* previous)
{
    rclcpp::sleep_for(std::chrono::seconds(1));
    algorithm->OnCompleteNavigation(GSLResult::Success, previousState);
}
