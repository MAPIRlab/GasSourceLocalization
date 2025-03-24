#pragma once
#include "MovingState.hpp"

namespace GSL
{
    class ManualNavigationState : public MovingState
    {
    public:
        ManualNavigationState(Algorithm* _algorithm);
        void chooseGoalAndMove() override;

    protected:
        void OnEnterState(State* previous) override;
        void RenderUI() override;
        bool paused;
    };
} // namespace GSL