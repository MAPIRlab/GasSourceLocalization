#pragma once

#include <StateMachines/State.h>
#include <StateMachines/StateMachine.h>

namespace GSL
{
    class Algorithm;
    class State : public StateMachines::State<State>
    {
    public:
        virtual void OnUpdate()
        {
        }
        virtual bool CanEnterState(const State* previousState) const override
        {
            return true;
        }
        virtual bool CanExitState(const State* nextState) const override
        {
            return true;
        }

        virtual void RenderUI(){}

    protected:
        State() = delete;
        State(Algorithm* _algorithm)
        {
            algorithm = _algorithm;
        }

        virtual void OnEnterState(State* previousState) override
        {
        }
        virtual void OnExitState(State* nextState) override
        {
        }

    protected:
        Algorithm* algorithm;
    };
} // namespace GSL