#pragma once
#include "MovingState.hpp"
#include "gsl_server/core/ConditionalMacros.hpp"

namespace GSL
{
    class ManualNavigationState : public MovingState
    {
    public:
        ManualNavigationState(Algorithm* _algorithm);
        void chooseGoalAndMove() override;

    protected:
        virtual void OnEnterState(State* previous) override;
        void OnUpdate() override;
        IF_GUI(void RenderUI() override;)
        bool paused;
    };
} // namespace GSL