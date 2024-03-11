#include <gsl_server/algorithms/Common/WaitForGasState.hpp>
#include <gsl_server/algorithms/Algorithm.hpp>
#include <gsl_server/core/logging.hpp>

namespace GSL
{
    WaitForGasState::WaitForGasState(Algorithm* _algorithm) : State(_algorithm)
    {
        maxWaitTime = algorithm->getParam<float>("maxWaitForGasTime", 10.0);
    }

    void WaitForGasState::OnEnterState(State* previous)
    {
        GSL_TRACE("Entering WaitForGas");
        startTime = algorithm->node->now();
    }

    void WaitForGasState::OnUpdate()
    {
        if ((algorithm->node->now() - startTime).seconds() > maxWaitTime)
        {
            GSL_WARN("Timed out while waiting for gas, going exploring");
            NavigateToPose::Goal goal;
            goal.pose = algorithm->getRandomPoseInMap();
            algorithm->movingState->sendGoal(goal);
        }
    }

    void WaitForGasState::addMeasurement(double concentration)
    {
        if (algorithm->stateMachine.getCurrentState() != this)
            return;
        if (concentration > algorithm->thresholdGas)
        {
            GSL_INFO("Found gas!");
            algorithm->stateMachine.forceSetState(algorithm->stopAndMeasureState.get());
        }
    }

} // namespace GSL