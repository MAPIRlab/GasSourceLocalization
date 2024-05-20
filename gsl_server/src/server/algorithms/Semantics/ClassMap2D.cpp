#include <gsl_server/algorithms/Semantics/ClassMap2D.hpp>
#include <gsl_server/Utils/RosUtils.hpp>
#include <gsl_server/core/ros_typedefs.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <gsl_server/Utils/Math.hpp>
#include <gsl_server/algorithms/PMFS/PMFSLib.hpp>

namespace GSL
{
    ClassMap2D::ClassMap2D(GridMetadata _gridMetadata, std::vector<Occupancy>& occupancy, BufferWrapper& _bufferWrapper,
                       const PoseWithCovarianceStamped& _currentRobotPose)
        : gridMetadata(_gridMetadata), wallsOccupancy(occupancy), bufferWrapper(_bufferWrapper), currentRobotPose(_currentRobotPose)
    {
        node = std::make_shared<rclcpp::Node>("class_map");
        
        std::string detectionsTopic = Utils::getParam<std::string>(node, "detectionsTopic", "objectDetections");
        cameraSub = node->create_subscription<Detection3DArray>(detectionsTopic, 1, std::bind(&ClassMap2D::detectionCallback, this, std::placeholders::_1));


        classDistributions.resize(gridMetadata.height * gridMetadata.width);
        //TODO populate the class distribution vector
    }

    std::vector<double> ClassMap2D::GetSourceProbability()
    {
        std::vector<double> probs(classDistributions.size(), 0.0);

        for(int i = 0; i< classDistributions.size(); i++)
        {
            auto& classDistribution = classDistributions[i];
            for(auto& [_class, classProb] : classDistribution)
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
    }


    double ClassMap2D::getSourceProbByClass(const std::string& _class)
    {
        return sourceProbByClass[_class];
    }

    void ClassMap2D::detectionCallback(Detection3DArray::ConstSharedPtr msg)
    {
        std::unordered_set<Vector2Int> remainingCellsInFOV = GetCellsInFOV();
        for(const Detection3D& detection : msg->detections)
        {
            AABB2DInt aabb = getAABB(detection);

            for(Vector2Int indices : aabb)
            {
                updateObjectProbabilities(indices, detection);
                remainingCellsInFOV.erase(indices);
            }
        }

        for(Vector2Int indices : remainingCellsInFOV)
        {
            //TODO update object probs, saw nothing
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
            bbCenter = Utils::fromMsg( 
                bufferWrapper.buffer.transform(center_stamped, "map").point
            );
            bbExtents = Utils::fromMsg(
                bufferWrapper.buffer.transform(size_stamped,"map").vector
            );
        }

        Vector3 minBBCoords = bbCenter - bbExtents * 0.5f;
        Vector3 maxBBCoords = bbCenter + bbExtents * 0.5f;
        
        AABB2DInt aabb;
        aabb.min = gridMetadata.coordinatesToIndex(minBBCoords.x, minBBCoords.y);
        aabb.max = gridMetadata.coordinatesToIndex(maxBBCoords.x, maxBBCoords.y);
        return aabb;
    }

    void ClassMap2D::updateObjectProbabilities(const Vector2Int& indices, const Detection3D& detection)
    {
        size_t index = gridMetadata.indexOf(indices);

        for(const vision_msgs::msg::ObjectHypothesisWithPose& hypWithPose: detection.results)
        {
            std::string _class = hypWithPose.hypothesis.class_id;
            float score = hypWithPose.hypothesis.score;
            classDistributions[index].UpdateProbOf(_class, score);
        }

        //TODO nothing class?
        
        classDistributions[index].Normalize();
    }

    std::unordered_set<Vector2Int> ClassMap2D::GetCellsInFOV()
    {
        std::unordered_set<Vector2Int> cellsInFOV;

        Vector2Int idxRobot = gridMetadata.coordinatesToIndex(currentRobotPose.pose.pose);
        Vector2Int idxLeftCorner, idxRightCorner;
        {
            Pose leftCornerLocal;
                leftCornerLocal.position.x = fov.maxDist*cos(fov.angleRads);
                leftCornerLocal.position.y = fov.maxDist*sin(fov.angleRads);
                leftCornerLocal.orientation = Utils::createQuaternionMsgFromYaw(0);
            Pose leftCornerWorld = Utils::compose(currentRobotPose.pose.pose, leftCornerLocal);
            idxLeftCorner = gridMetadata.coordinatesToIndex(leftCornerWorld);

            Pose rightCornerLocal;
                rightCornerLocal.position.x = fov.maxDist*cos(fov.angleRads);
                rightCornerLocal.position.y = -fov.maxDist*sin(fov.angleRads);
                rightCornerLocal.orientation = Utils::createQuaternionMsgFromYaw(0);
            Pose rightCornerWorld = Utils::compose(currentRobotPose.pose.pose, rightCornerLocal);
            idxRightCorner = gridMetadata.coordinatesToIndex(rightCornerWorld);
        }
    
        AABB2DInt aabb(
            Vector2Int(std::min({idxRobot.x, idxLeftCorner.x, idxRightCorner.x}), std::min({idxRobot.y, idxLeftCorner.y, idxRightCorner.y})),
            Vector2Int(std::max({idxRobot.x, idxLeftCorner.x, idxRightCorner.x}), std::max({idxRobot.y, idxLeftCorner.y, idxRightCorner.y}))
        );

        //yaw of the camera in world space
        double cameraYaw = Utils::getYaw(currentRobotPose.pose.pose.orientation);

        Vector2 robotCoords{currentRobotPose.pose.pose.position.x, currentRobotPose.pose.pose.position.y};

        for(Vector2Int indices : aabb)
        {
            if (!gridMetadata.indicesInBounds(indices))
                continue;
            
            Vector2 point = gridMetadata.indexToCoordinates(indices);
            Vector2 camToPoint =  point - robotCoords;
            float distance = vmath::length(camToPoint);

            //yaw of the vector that goes from the camera to the considered point
            double angleWorldSpace = std::atan2(camToPoint.y, camToPoint.x);
            double angleCameraSpace = std::atan2(std::sin(angleWorldSpace-cameraYaw), std::cos(angleWorldSpace-cameraYaw)); 

            if(distance < fov.maxDist && distance > fov.minDist
                && std::abs(angleCameraSpace) < fov.angleRads
                && PMFSLib::pathFree(gridMetadata, wallsOccupancy, robotCoords, point))
            {
                cellsInFOV.insert(indices);
            }
        }
        return cellsInFOV;
    }

}