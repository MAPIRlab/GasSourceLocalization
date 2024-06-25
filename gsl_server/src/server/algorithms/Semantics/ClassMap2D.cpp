#include <gsl_server/Utils/RosUtils.hpp>
#include <gsl_server/Utils/Collections.hpp>
#include <gsl_server/algorithms/Semantics/ClassMap2D.hpp>
#include <gsl_server/core/ros_typedefs.hpp>

#include <filesystem>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <gsl_server/Utils/Color.hpp>
#include <gsl_server/Utils/Math.hpp>
#include <gsl_server/algorithms/PMFS/PMFSLib.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>

namespace GSL
{
    ClassMap2D::ClassMap2D(GridMetadata _gridMetadata, std::vector<Occupancy>& occupancy, BufferWrapper& _bufferWrapper,
                           const PoseWithCovarianceStamped& _currentRobotPose)
        : gridMetadata(_gridMetadata), wallsOccupancy(occupancy), bufferWrapper(_bufferWrapper), currentRobotPose(_currentRobotPose)
    {
        node = std::make_shared<rclcpp::Node>("class_map");

        std::string detectionsTopic = Utils::getParam<std::string>(node, "detectionsTopic", "objectDetections");
        cameraSub = node->create_subscription<Detection3DArray>(detectionsTopic, 1, std::bind(&ClassMap2D::detectionCallback, this, std::placeholders::_1));

        classMarkers = node->create_publisher<Marker>("class_markers", 1);

        classDistributions.resize(gridMetadata.height * gridMetadata.width);
        std::string ontologyPath = Utils::getParam<std::string>(node, "ontologyPath", "?");
        parseOntology(ontologyPath);
    }

    std::vector<double> ClassMap2D::GetSourceProbability()
    {
        std::vector<double> probs(classDistributions.size(), 0.0);

        for (int i = 0; i < classDistributions.size(); i++)
        {
            auto& classDistribution = classDistributions[i];
            for (auto& [_class, classProb] : classDistribution)
            {
                probs[i] += classProb * getSourceProbByClass(_class);
            }
        }

        Grid<double> probGrid(probs, wallsOccupancy, gridMetadata);
        Utils::NormalizeDistribution(probGrid);
        return probs;
    }

    void ClassMap2D::OnUpdate()
    {
        rclcpp::spin_some(node);
        publishClassMarkers();
    }

    double ClassMap2D::getSourceProbByClass(const std::string& _class)
    {
        return sourceProbByClass[_class];
    }

    void ClassMap2D::parseOntology(const std::string& path)
    {
        if (!std::filesystem::exists(path))
        {
            GSL_ERROR("Could not open ontology file: {}", path);
            CLOSE_PROGRAM;
        }
        YAML::Node gases = YAML::LoadFile(path)["Gases"];
        for (const YAML::Node& gas : gases)
        {
            if (gas["gas"].as<std::string>() == "sewage")
            {
                YAML::Node probs = gas["probs"];
                for (YAML::const_iterator it = probs.begin(); it != probs.end(); ++it)
                {
                    sourceProbByClass.insert({it->first.as<std::string>(), it->second.as<float>()});
                }
            }
        }

        GSL_INFO("Source probability by semantic class:");
        for (auto& kv : sourceProbByClass)
        {
            GSL_INFO("{}: {}", kv.first, kv.second);
        }
    }

    void ClassMap2D::detectionCallback(Detection3DArray::ConstSharedPtr msg)
    {
        std::unordered_set<Vector2Int> remainingCellsInFOV = getCellsInFOV();
        for (const Detection3D& detection : msg->detections)
        {
            std::vector<std::pair<std::string, float>> scores;
            for (const auto& hyp : detection.results) scores.emplace_back(hyp.hypothesis.class_id, hyp.hypothesis.score);

            AABB2DInt aabb = getAABB(detection);

            for (Vector2Int indices : aabb)
            {
                updateObjectProbabilities(indices, scores);
                remainingCellsInFOV.erase(indices);
            }
        }

        constexpr float true_negative_prob = 0.6;
        constexpr float false_negative_prob = 0.2;
        //cells where we didn't see anything
        {
            for (Vector2Int indices : remainingCellsInFOV)
            {
                std::vector<std::pair<std::string, float>> scores;
                for (const auto& kv : sourceProbByClass)
                {
                    float score;
                    if(kv.first == "other")
                        score = true_negative_prob;
                    else
                        score = false_negative_prob;
                    scores.emplace_back(kv.first, score);
                }
                updateObjectProbabilities(indices, scores);
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

    void ClassMap2D::updateObjectProbabilities(const Vector2Int& indices, const std::vector<std::pair<std::string, float>>& scores)
    {
        size_t index = gridMetadata.indexOf(indices);

        for (const auto& pair : scores)
        {
            std::string _class = pair.first;
            if (!sourceProbByClass.contains(_class))
                _class = "other";
            float score = pair.second;
            classDistributions[index].UpdateProbOf(_class, score);
        }

        classDistributions[index].Normalize();
    }

    std::unordered_set<Vector2Int> ClassMap2D::getCellsInFOV()
    {
        std::unordered_set<Vector2Int> cellsInFOV;

        Vector2Int idxRobot = gridMetadata.coordinatesToIndex(currentRobotPose.pose.pose);
        Vector2Int idxLeftCorner, idxRightCorner;
        {
            Pose leftCornerLocal;
            leftCornerLocal.position.x = fov.maxDist * cos(fov.angleRads);
            leftCornerLocal.position.y = fov.maxDist * sin(fov.angleRads);
            leftCornerLocal.orientation = Utils::createQuaternionMsgFromYaw(0);
            Pose leftCornerWorld = Utils::compose(currentRobotPose.pose.pose, leftCornerLocal);
            idxLeftCorner = gridMetadata.coordinatesToIndex(leftCornerWorld);

            Pose rightCornerLocal;
            rightCornerLocal.position.x = fov.maxDist * cos(fov.angleRads);
            rightCornerLocal.position.y = -fov.maxDist * sin(fov.angleRads);
            rightCornerLocal.orientation = Utils::createQuaternionMsgFromYaw(0);
            Pose rightCornerWorld = Utils::compose(currentRobotPose.pose.pose, rightCornerLocal);
            idxRightCorner = gridMetadata.coordinatesToIndex(rightCornerWorld);
        }

        AABB2DInt aabb(
            Vector2Int(std::min({idxRobot.x, idxLeftCorner.x, idxRightCorner.x}), std::min({idxRobot.y, idxLeftCorner.y, idxRightCorner.y})),
            Vector2Int(std::max({idxRobot.x, idxLeftCorner.x, idxRightCorner.x}), std::max({idxRobot.y, idxLeftCorner.y, idxRightCorner.y})));

        // yaw of the camera in world space
        double cameraYaw = Utils::getYaw(currentRobotPose.pose.pose.orientation);

        Vector2 robotCoords{currentRobotPose.pose.pose.position.x, currentRobotPose.pose.pose.position.y};

        for (Vector2Int indices : aabb)
        {
            if (!gridMetadata.indicesInBounds(indices))
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

    void ClassMap2D::publishClassMarkers()
    {
        static std::vector<std_msgs::msg::ColorRGBA> colors;
        if(colors.empty())
        {
            for(const auto& kv : sourceProbByClass)
            {
                float randomValue = Utils::EquallyDistributed01F();
                Utils::ColorHSV hsv = {randomValue, 1, 1};
                colors.push_back(Utils::HSVtoRGB(hsv.H, hsv.S, hsv.V));
            }
        }

        Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = node->now();
        marker.type = Marker::POINTS;
        marker.scale.x = gridMetadata.cellSize;
        marker.scale.y = gridMetadata.cellSize;
        marker.scale.z = gridMetadata.cellSize;

        for(int i = 0; i<gridMetadata.width; i++)
        {
            for(int j = 0; j<gridMetadata.height; j++)
            {
                int cellIndex = gridMetadata.indexOf({i,j});
                if( wallsOccupancy[cellIndex] != Occupancy::Free)
                    continue;
                Vector2 coords = gridMetadata.indexToCoordinates(i,j);
                Point p;
                p.x = coords.x;
                p.y = coords.y;
                p.z = 0.0;
                marker.points.push_back(p);

                int maxClassIndex = Utils::indexOfMax(classDistributions[cellIndex]);
                marker.colors.push_back(colors[maxClassIndex]);
            }
        }

        classMarkers->publish(marker);
    }


} // namespace GSL