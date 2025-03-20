#pragma once
#include "gsl_server/core/ConditionalMacros.hpp"
#include <gsl_server/algorithms/Common/MovingState.hpp>
#include <gsl_server/core/Vectors.hpp>

#if USE_GUI
namespace GSL::SemanticPMFS_internal
{
    class UI;
}
#endif

namespace GSL
{
    class SemanticPMFS;
    class MovingStateSemanticPMFS : public MovingState
    {
        using HashSet = std::unordered_set<Vector2Int>;
        IF_GUI(friend class SemanticPMFS_internal::UI);

    public:
        MovingStateSemanticPMFS(Algorithm* _algorithm);

        enum class MovementType
        {
            Exploration,
            Search
        } currentMovement = MovementType::Exploration;

        void chooseGoalAndMove() override;
        double explorationValue(int i, int j);

        // Get the information value of cell (i,j) for the main phase
        double informationValue(int i, int j);

        void debugMoveTo(int i, int j);
        void publishMarkers();

    protected:
        std::vector<double> mutualInformationGas;
        std::vector<double> semanticsEntropy;
        HashSet closedMoveSet;
        HashSet openMoveSet;
        uint movesCounter = 0;
        SemanticPMFS* pmfs;

        NavigateToPose::Goal indexToGoal(int i, int j);
        void Fail() override;
        void calculateMutualInformationGas();

        struct Publishers
        {
            rclcpp::Publisher<Marker>::SharedPtr explorationValue;
            rclcpp::Publisher<Marker>::SharedPtr varianceHit;
            rclcpp::Publisher<Marker>::SharedPtr movementSets;
        } publishers;
    };
} // namespace GSL