#include <gsl_server/algorithms/Common/Utils/Collections.hpp>
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>
#include <gsl_server/algorithms/Semantics/Semantics/ClassMap2D.hpp>
#include <gsl_server/core/ros_typedefs.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <gsl_server/algorithms/Common/Utils/Color.hpp>
#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <gsl_server/algorithms/PMFS/PMFSLib.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

const char* otherClassName = "other";
namespace GSL
{
    ClassMap2D::ClassMap2D(Grid2DMetadata _gridMetadata, std::vector<Occupancy>& occupancy, BufferWrapper& _bufferWrapper,
                           const PoseWithCovarianceStamped& _currentRobotPose)
        : gridMetadata(_gridMetadata), wallsOccupancy(occupancy), bufferWrapper(_bufferWrapper), currentRobotPose(_currentRobotPose)
    {
        node = std::make_shared<rclcpp::Node>("class_map");

        std::string detectionsTopic = Utils::getParam<std::string>(node, "detectionsTopic", "objectDetections");
        cameraSub =
            node->create_subscription<Detection3DArray>(detectionsTopic, 1, std::bind(&ClassMap2D::detectionCallback, this, std::placeholders::_1));

        classMarkers = node->create_publisher<Marker>("class_markers", 1);

        classMap.classDistributions.resize(gridMetadata.height * gridMetadata.width);
        std::string ontologyPath = Utils::getParam<std::string>(node, "ontologyPath", "?");
        std::string targetGas = Utils::getParam<std::string>(node, "targetGas", "smoke");
        classMap.parseOntology(ontologyPath, targetGas);

        fov.maxDist = Utils::getParam<double>(node, "fovMaxDist", 4.0);
        fov.minDist = Utils::getParam<double>(node, "fovMinDist", 0.0);
        fov.angleRads = Utils::getParam<double>(node, "fovAngleRads", 30 * Utils::Deg2Rad);
    }

    void ClassMap2D::OnUpdate()
    {
        rclcpp::spin_some(node);
        visualize();
    }

    std::vector<double> ClassMap2D::GetSourceProbability()
    {
        std::vector<double> probs(gridMetadata.height * gridMetadata.width, 0.0);
        GetSourceProbabilityInPlace(probs);
        return probs;
    }

    void ClassMap2D::GetSourceProbabilityInPlace(std::vector<double>& sourceProb)
    {
        for (int i = 0; i < classMap.classDistributions.size(); i++)
        {
            classMap.computeSourceProbability(classMap.classDistributions[i], sourceProb[i]);
        }

        Grid2D<double> probGrid(sourceProb, wallsOccupancy, gridMetadata);
        Utils::NormalizeDistribution(probGrid);
    }

    double ClassMap2D::GetSourceProbabilityAt(const Vector3& point)
    {
        Vector2Int indices = gridMetadata.coordinatesToIndex(point.x, point.y);
        size_t index = gridMetadata.indexOf(indices);
        double value = 0;
        classMap.computeSourceProbability(classMap.classDistributions[index], value);
        return value;
    }

    void ClassMap2D::detectionCallback(Detection3DArray::ConstSharedPtr msg)
    {
        std::unordered_set<Vector2Int> remainingCellsInFOV = getCellsInFOV(currentRobotPose.pose.pose);
        for (const Detection3D& detection : msg->detections)
        {
            std::vector<std::pair<std::string, float>> scores;
            for (const auto& hyp : detection.results)
            {
                std::string _class = hyp.hypothesis.class_id;
                if (!classMap.sourceProbByClass.contains(_class))
                    _class = otherClassName;
                scores.emplace_back(_class, hyp.hypothesis.score);
            }

            // fill in missing classes with a low score
            for (const auto& [_class, _] : classMap.sourceProbByClass)
            {
                auto predicate = [&](const auto & t)
                {
                    return _class == t.first;
                };
                if (!Utils::containsPred(scores, predicate))
                    scores.emplace_back(_class, 0.1f); // TODO change the value to something meaningful
            }

            AABB2DInt aabb = getAABB(detection);

            for (Vector2Int indices : aabb)
            {
                if (!gridMetadata.indicesInBounds(indices))
                {
                    GSL_WARN("Indices {} out of bounds! Probably a bad mask causing a huge bounding box", indices);
                    break;
                }
                size_t index = gridMetadata.indexOf(indices);
                classMap.updateObjectProbabilities(index, scores);
                remainingCellsInFOV.erase(indices);
            }
        }

        // TODO make this a less arbitrary
        const float true_negative_prob = 1.0 / classMap.sourceProbByClass.size() + 0.05f;
        const float false_negative_prob = (1.0f - true_negative_prob) / (classMap.sourceProbByClass.size() - 1);
        // cells where we didn't see anything
        {
            for (Vector2Int indices : remainingCellsInFOV)
            {
                std::vector<std::pair<std::string, float>> scores;
                for (const auto& kv : classMap.sourceProbByClass)
                {
                    float score;
                    if (kv.first == otherClassName)
                        score = true_negative_prob;
                    else
                        score = false_negative_prob;
                    scores.emplace_back(kv.first, score);
                }
                size_t index = gridMetadata.indexOf(indices);
                classMap.updateObjectProbabilities(index, scores);
            }
        }
    }

    AABB2DInt ClassMap2D::getAABB(const Detection3D& detection)
    {
        Vector3 bbCenter;
        Vector3 bbExtents;
        {
            geometry_msgs::msg::PointStamped center_stamped;
            center_stamped.header = detection.header;
            center_stamped.point = detection.bbox.center.position;
            geometry_msgs::msg::Vector3Stamped size_stamped;
            size_stamped.header = detection.header;
            size_stamped.vector = detection.bbox.size;
            bbCenter = Utils::fromMsg(bufferWrapper.buffer.transform(center_stamped, "map").point);
            bbExtents = Utils::fromMsg(bufferWrapper.buffer.transform(size_stamped, "map").vector);
        }

        Vector3 minBBCoords = bbCenter - bbExtents * 0.5f;
        Vector3 maxBBCoords = bbCenter + bbExtents * 0.5f;

        AABB2DInt aabb;
        aabb.min = gridMetadata.coordinatesToIndex(minBBCoords.x, minBBCoords.y);
        aabb.max = gridMetadata.coordinatesToIndex(maxBBCoords.x, maxBBCoords.y);
        return aabb;
    }

    std::unordered_set<Vector2Int> ClassMap2D::getCellsInFOV(Pose robotPose)
    {
        std::unordered_set<Vector2Int> cellsInFOV;

        Vector2Int idxRobot = gridMetadata.coordinatesToIndex(robotPose);
        Vector2Int idxLeftCorner, idxRightCorner;
        {
            Pose leftCornerLocal;
            leftCornerLocal.position.x = fov.maxDist * cos(fov.angleRads);
            leftCornerLocal.position.y = fov.maxDist * sin(fov.angleRads);
            leftCornerLocal.orientation = Utils::createQuaternionMsgFromYaw(0);
            Pose leftCornerWorld = Utils::compose(robotPose, leftCornerLocal);
            idxLeftCorner = gridMetadata.coordinatesToIndex(leftCornerWorld);

            Pose rightCornerLocal;
            rightCornerLocal.position.x = fov.maxDist * cos(fov.angleRads);
            rightCornerLocal.position.y = -fov.maxDist * sin(fov.angleRads);
            rightCornerLocal.orientation = Utils::createQuaternionMsgFromYaw(0);
            Pose rightCornerWorld = Utils::compose(robotPose, rightCornerLocal);
            idxRightCorner = gridMetadata.coordinatesToIndex(rightCornerWorld);
        }

        AABB2DInt aabb(
            Vector2Int(std::min({idxRobot.x, idxLeftCorner.x, idxRightCorner.x}), std::min({idxRobot.y, idxLeftCorner.y, idxRightCorner.y})),
            Vector2Int(std::max({idxRobot.x, idxLeftCorner.x, idxRightCorner.x}), std::max({idxRobot.y, idxLeftCorner.y, idxRightCorner.y})));

        // yaw of the camera in world space
        double cameraYaw = Utils::getYaw(robotPose.orientation);

        Vector2 robotCoords{robotPose.position.x, robotPose.position.y};

        for (Vector2Int indices : aabb)
        {
            if (!gridMetadata.indicesInBounds(indices) || wallsOccupancy[gridMetadata.indexOf(indices)] != Occupancy::Free)
                continue;

            Vector2 point = gridMetadata.indexToCoordinates(indices);
            Vector2 camToPoint = point - robotCoords;
            float distance = vmath::length(camToPoint);

            // yaw of the vector that goes from the camera to the considered point
            double angleWorldSpace = std::atan2(camToPoint.y, camToPoint.x);
            double angleCameraSpace = std::atan2(std::sin(angleWorldSpace - cameraYaw), std::cos(angleWorldSpace - cameraYaw));

            if (distance < fov.maxDist && distance > fov.minDist && std::abs(angleCameraSpace) < fov.angleRads &&
                    PMFSLib::pathFree(gridMetadata, wallsOccupancy, robotCoords, point))
            {
                cellsInFOV.insert(indices);
            }
        }
        return cellsInFOV;
    }

    void ClassMap2D::visualize()
    {
        using namespace Utils::Colors;
        static std::map<std::string, ColorRGBA> colors;
        if (colors.empty())
        {
            for (const auto& kv : classMap.sourceProbByClass)
            {
                float randomValue = Utils::EquallyDistributed01F() * 360;
                ColorHSV hsv{.H = randomValue, .S = 1, .V = 1};
                colors[kv.first] = HSVtoRGB(hsv.H, hsv.S, hsv.V);
            }
        }

        Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = node->now();
        marker.type = Marker::POINTS;
        marker.scale.x = gridMetadata.cellSize;
        marker.scale.y = gridMetadata.cellSize;
        marker.scale.z = gridMetadata.cellSize;

        for (int i = 0; i < gridMetadata.height; i++)
        {
            for (int j = 0; j < gridMetadata.width; j++)
            {
                int cellIndex = gridMetadata.indexOf({i, j});
                if (wallsOccupancy[cellIndex] != Occupancy::Free)
                    continue;
                Vector2 coords = gridMetadata.indexToCoordinates(i, j);
                Point p;
                p.x = coords.x;
                p.y = coords.y;
                p.z = 0.0;
                marker.points.push_back(p);

                ColorRGBA markerColor = colors[otherClassName];
                for (const auto& [_class, prob] : classMap.classDistributions[cellIndex])
                {
                    markerColor = lerp(markerColor, colors[_class], prob);
                }
                marker.colors.push_back(markerColor);
            }
        }

        classMarkers->publish(marker);
    }

} // namespace GSL