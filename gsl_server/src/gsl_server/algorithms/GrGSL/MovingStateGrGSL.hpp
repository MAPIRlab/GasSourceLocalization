#pragma once
#include <gsl_server/algorithms/Common/MovingState.hpp>
#include <gsl_server/core/Vectors.hpp>
#include <gsl_server/algorithms/GrGSL/GrGSL.hpp>

namespace GSL
{

    class MovingStateGrGSL : public MovingState
    {

        typedef std::unordered_set<Vector2Int> HashSet;

    public:
        MovingStateGrGSL(Algorithm* _algorithm);

        void chooseGoalAndMove();

    protected:
        GrGSL* grgsl;

        HashSet openMoveSet;
        HashSet closedMoveSet;

        NavigateToPose::Goal indexToGoal(int i, int j);

        std::optional<NavigateToPose::Goal> getNormalGoal();
        std::optional<NavigateToPose::Goal> getInfotaxisGoal();

        // Wind Estimation (for infotaxis)
        //-------------
        rclcpp::Client<GrGSL::WindEstimation>::SharedPtr clientWind;
        std::vector<GrGSL::WindVector> estimateWind();
    };
} // namespace GSL