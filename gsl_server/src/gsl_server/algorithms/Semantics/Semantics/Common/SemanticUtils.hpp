#pragma once

#include "gsl_server/algorithms/Common/Grid2D.hpp"
#include "gsl_server/core/VectorsImpl/vmath_DDACustomVec.hpp"
#include "gsl_server/core/ros_typedefs.hpp"
#include <unordered_set>
#include "gsl_server/algorithms/Common/Occupancy.hpp"

namespace GSL::Semantics
{

    struct FOV
    {
        float angleRads;
        float minDist;
        float maxDist;
    };

    static std::unordered_set<Vector2Int> getCellsInFOV(Pose robotPose, const Grid2DMetadata& gridMetadata, const FOV& fov, const std::vector<Occupancy>& occupancy);

} // namespace GSL::Semantics