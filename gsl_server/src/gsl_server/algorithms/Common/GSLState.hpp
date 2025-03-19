#pragma once

#include <StateMachines/State.h>
#include <StateMachines/StateMachine.h>

namespace GSL
{
    class Algorithm;
    class State : public StateMachines::State<State>
    {
    public:
        State() = delete;
        State(Algorithm* _algorithm)
        {
            algorithm = _algorithm;
        }

        virtual bool CanEnterState(const State* previousState) const
        {
            return true;
        }
        virtual bool CanExitState(const State* nextState) const
        {
            return true;
        }
        virtual void OnEnterState(State* previousState)
        {
        }
        virtual void OnExitState(State* nextState)
        {
        }
        virtual void OnUpdate()
        {
        }

    protected:
        Algorithm* algorithm;
    };
} // namespace GSL