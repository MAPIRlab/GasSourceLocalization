#include "ManualNavigation.hpp"
#include "Algorithm.hpp"
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>
#include <iostream>

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
    auto exec = Utils::createExecutor(algorithm->node);
    std::jthread spinThread([&]()
                            {
                                exec->spin();
                            });

    GSL_INFO("Entering Manual Driving, press [enter] to resume algorithm execution");
    std::cin.get();

    exec->cancel();
    spinThread.join();

    algorithm->OnCompleteNavigation(GSLResult::Success, previousState);
}
