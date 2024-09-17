#pragma once

#include <gsl_server/algorithms/Common/MovingState.hpp>

namespace GSL
{
    enum class PTMovement
    {
        None = 0,
        FollowPlume,
        RecoverPlume,
        Exploration,
    };

    class MovingStatePlumeTracking : public MovingState
    {
    public:
        MovingStatePlumeTracking(Algorithm* _algorithm) : MovingState(_algorithm)
        {
        }

        void OnUpdate() override;

        PTMovement currentMovement = PTMovement::None;
    };
} // namespace GSL