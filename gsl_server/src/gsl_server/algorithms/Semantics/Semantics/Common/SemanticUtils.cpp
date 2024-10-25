#include "SemanticUtils.hpp"
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>
#include "AABB.hpp"

namespace GSL::Semantics
{
    std::unordered_set<Vector2Int> getCellsInFOV(Pose robotPose, const Grid2DMetadata& gridMetadata, const FOV& fov, const std::vector<Occupancy>& occupancy)
    {
        std::unordered_set<Vector2Int> cellsInFOV;

        Vector2Int idxRobot = gridMetadata.coordinatesToIndices(robotPose);
        Vector2Int idxLeftCorner, idxRightCorner;
        {
            Pose leftCornerLocal;
            leftCornerLocal.position.x = fov.maxDist * cos(fov.angleRads);
            leftCornerLocal.position.y = fov.maxDist * sin(fov.angleRads);
            leftCornerLocal.orientation = Utils::createQuaternionMsgFromYaw(0);
            Pose leftCornerWorld = Utils::compose(robotPose, leftCornerLocal);
            idxLeftCorner = gridMetadata.coordinatesToIndices(leftCornerWorld);

            Pose rightCornerLocal;
            rightCornerLocal.position.x = fov.maxDist * cos(fov.angleRads);
            rightCornerLocal.position.y = -fov.maxDist * sin(fov.angleRads);
            rightCornerLocal.orientation = Utils::createQuaternionMsgFromYaw(0);
            Pose rightCornerWorld = Utils::compose(robotPose, rightCornerLocal);
            idxRightCorner = gridMetadata.coordinatesToIndices(rightCornerWorld);
        }

        AABB2DInt aabb(
            Vector2Int(std::min({idxRobot.x, idxLeftCorner.x, idxRightCorner.x}), std::min({idxRobot.y, idxLeftCorner.y, idxRightCorner.y})),
            Vector2Int(std::max({idxRobot.x, idxLeftCorner.x, idxRightCorner.x}), std::max({idxRobot.y, idxLeftCorner.y, idxRightCorner.y})));

        // yaw of the camera in world space
        double cameraYaw = Utils::getYaw(robotPose.orientation);

        Vector2 robotCoords(robotPose.position.x, robotPose.position.y);

        for (Vector2Int indices : aabb)
        {
            if (!gridMetadata.indicesInBounds(indices) || occupancy[gridMetadata.indexOf(indices)] != Occupancy::Free)
                continue;

            Vector2 point = gridMetadata.indicesToCoordinates(indices);
            Vector2 camToPoint = point - robotCoords;
            float distance = vmath::length(camToPoint);

            // yaw of the vector that goes from the camera to the considered point
            double angleWorldSpace = std::atan2(camToPoint.y, camToPoint.x);
            double angleCameraSpace = std::atan2(std::sin(angleWorldSpace - cameraYaw), std::cos(angleWorldSpace - cameraYaw));

            if (distance < fov.maxDist && distance > fov.minDist && std::abs(angleCameraSpace) < fov.angleRads &&
                GridUtils::PathFree(gridMetadata, occupancy, robotCoords, point))
            {
                cellsInFOV.insert(indices);
            }
        }
        return cellsInFOV;
    }
} // namespace GSL::Semantics