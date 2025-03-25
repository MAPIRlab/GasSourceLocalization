#pragma once
#include "gsl_server/algorithms/Common/Grid2D.hpp"
#include "gsl_server/algorithms/Common/Occupancy.hpp"
#include "gsl_server/algorithms/GrGSL/GrGSL_internal.hpp"
#include "gsl_server/algorithms/GrGSL/internal/UI.hpp"
#include "gsl_server/core/VectorsImpl/vmath_DDACustomVec.hpp"
#include <gmrf_msgs/srv/detail/wind_estimation__struct.hpp>
#include <gsl_server/algorithms/Common/States/MovingState.hpp>
#include <gsl_server/core/Vectors.hpp>
#include <gsl_server/core/ConditionalMacros.hpp>
#include <vector>

namespace GSL
{
    struct GrGSLData
    {
        std::shared_ptr<rclcpp::Node> node;
        GrGSL_internal::Settings& settings;
        std::vector<GrGSL_internal::Cell>& cells;
        std::vector<Occupancy>& occupancy;
        Grid2DMetadata& gridMetadata;
        Vector2& currentRobotPosition;
        Vector2& positionOfLastHit;

        double probability(Vector2Int p)
        {
            return cells.at(gridMetadata.indexOf(p)).sourceProb;
        }
    };

    class MovingStateGrGSL : public MovingState
    {
        IF_GUI(friend class GrGSL_internal::UI);
        typedef std::unordered_set<Vector2Int> HashSet;
        using WindEstimation = gmrf_msgs::srv::WindEstimation;

    public:
        MovingStateGrGSL(Algorithm* _algorithm, const GrGSLData& data);

        void chooseGoalAndMove() override;

    protected:
        GrGSLData grgsl;

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