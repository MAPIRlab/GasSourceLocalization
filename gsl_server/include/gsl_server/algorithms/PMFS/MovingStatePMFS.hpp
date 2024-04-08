#pragma once
#include <gsl_server/algorithms/Common/MovingState.hpp>
#include <gsl_server/core/Vectors.hpp>

namespace GSL
{
    class PMFS;
    class MovingStatePMFS : public MovingState
    {
        using HashSet = std::unordered_set<Vector2Int>;

    public:
        MovingStatePMFS(Algorithm* _algorithm);

        enum class MovementType
        {
            Exploration,
            Search
        } currentMovement = MovementType::Exploration;

        void chooseGoalAndMove();
        double explorationValue(int i, int j);

        void debugMoveTo(int i, int j);
        void publishMarkers();

    protected:
        HashSet closedMoveSet;
        HashSet openMoveSet;
        NavigateToPose::Goal indexToGoal(int i, int j);
        void Fail() override;

        PMFS* pmfs;
    };
} // namespace GSL