#pragma once
#include <gsl_server/algorithms/Common/MovingState.hpp>
#include <gsl_server/core/Vectors.hpp>
#include <gsl_server/algorithms/GrGSL/GrGSL.hpp>

namespace GSL
{

    class MovingStateGrGSL : public MovingState
    {

        typedef std::unordered_set<Vector2Int> hashSet;

    public:
        MovingStateGrGSL(Algorithm* _algorithm);

        void chooseGoalAndMove();

    protected:
        GrGSL* grgsl;

        hashSet openMoveSet;
        hashSet closedMoveSet;

        NavigateToPose::Goal indexToGoal(int i, int j);

        NavigateToPose::Goal getNormalGoal();
        NavigateToPose::Goal getInfotaxisGoal();

        // Wind Estimation (for infotaxis)
        //-------------
        rclcpp::Client<GrGSL::WindEstimation>::SharedPtr clientWind;
        std::vector<GrGSL::WindVector> estimateWind();
    };
} // namespace GSL