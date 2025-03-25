#pragma once
#include "MovingState.hpp"

namespace GSL
{
    // returns immediately. For testing continuous updates with a rosbag or similar
    class NoNavigationState : public MovingState
    {
    public:
        NoNavigationState(Algorithm* _algorithm);
        void chooseGoalAndMove() override;

    protected:
        void OnEnterState(State* previous) override;
    };
} // namespace GSL