#pragma once
#include "gsl_server/core/VectorsImpl/vmath_DDACustomVec.hpp"
#include <gsl_server/algorithms/Common/States//MovingState.hpp>
#include <gsl_server/core/Vectors.hpp>
#include "SemanticGrGSL.hpp"
#include <gsl_server/algorithms/GrGSL/GrGSLLib.hpp>
#include <vector>

namespace GSL
{

    class MovingStateSemanticGrGSL : public MovingState
    {
        IF_GUI(friend class SemanticGrGSL_internal::UI;)
        typedef std::unordered_set<Vector2Int> HashSet;
        using WindEstimation = gmrf_msgs::srv::WindEstimation;

    public:
        MovingStateSemanticGrGSL(Algorithm* _algorithm);

        void chooseGoalAndMove() override;

    protected:
        SemanticGrGSL* grgsl;

        HashSet openMoveSet;
        HashSet closedMoveSet;

        NavigateToPose::Goal indexToGoal(int i, int j);

        std::optional<NavigateToPose::Goal> getNormalGoal();
        std::optional<NavigateToPose::Goal> getInfotaxisGoal();

        // Wind Estimation (for infotaxis)
        //-------------
        rclcpp::Client<WindEstimation>::SharedPtr clientWind;
        std::vector<GrGSL_internal::WindVector> estimateWind();
        std::vector<GrGSL_internal::WindVector> getWindVectors(const std::vector<Vector2Int>& indices);
    };
} // namespace GSL