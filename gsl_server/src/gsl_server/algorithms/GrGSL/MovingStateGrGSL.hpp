#pragma once
#include "gsl_server/core/VectorsImpl/vmath_DDACustomVec.hpp"
#include <gsl_server/algorithms/Common/MovingState.hpp>
#include <gsl_server/core/Vectors.hpp>
#include <gsl_server/algorithms/GrGSL/GrGSL.hpp>
#include <vector>

namespace GSL
{

    class MovingStateGrGSL : public MovingState
    {
        IF_GUI(friend class GrGSL_internal::UI);
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
        std::vector<GrGSL_internal::WindVector> estimateWind();
        std::vector<GrGSL_internal::WindVector> getWindVectors(const std::vector<Vector2Int>& indices);
    };
} // namespace GSL