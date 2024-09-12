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

        //apply the movement strategy to choose a goal and send it to the action server
        void chooseGoalAndMove();

        //Get the information value of cell (i,j) for the initial exploration phase
        double explorationValue(int i, int j);

        //force the next movement to go to cell (i,j), ignoring the movement strategy
        void debugMoveTo(int i, int j);

        //Rviz visualization of movement sets and information gain estimations
        void publishMarkers();

    protected:
        HashSet closedMoveSet;
        HashSet openMoveSet;
        NavigateToPose::Goal indexToGoal(int i, int j);
        void Fail() override;

        PMFS* pmfs;


        struct Publishers
        {
            rclcpp::Publisher<Marker>::SharedPtr explorationValue;
            rclcpp::Publisher<Marker>::SharedPtr varianceHit;
            rclcpp::Publisher<Marker>::SharedPtr movementSets;
        } publishers;
    };
} // namespace GSL