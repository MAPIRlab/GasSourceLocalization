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
    algorithm->OnCompleteNavigation(GSLResult::Success, previousState);
}
