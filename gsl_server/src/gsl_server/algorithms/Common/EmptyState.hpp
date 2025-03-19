#pragma once

#include "GSLState.hpp"

namespace GSL
{
    class Algorithm;

    // Literally does nothing
    // Mostly for debugging/visualization purposes
    class EmptyState : public State
    {
    public:
        EmptyState(Algorithm* _alg)
            : State(_alg)
        {}
    };
} // namespace GSL