#include <gsl_server/Utils/Math.hpp>
#include <gsl_server/Utils/RosUtils.hpp>
#include <gsl_server/algorithms/Semantics/ClassMap3D.hpp>
#include <voxeland/srv/get_class_distributions.hpp>

namespace GSL
{
    ClassMap3D::ClassMap3D(GridMetadata _gridMetadata, std::vector<Occupancy>& occupancy, BufferWrapper& _bufferWrapper,
                           const PoseWithCovarianceStamped& _currentRobotPose)
        : gridMetadata(_gridMetadata), wallsOccupancy(occupancy), bufferWrapper(_bufferWrapper), currentRobotPose(_currentRobotPose)
    {
        node = std::make_shared<rclcpp::Node>("class_map");

        std::string detectionsTopic = Utils::getParam<std::string>(node, "detectionsTopic", "objectDetections");

        classMarkers = node->create_publisher<Marker>("class_markers", 1);

        zLimits.x = Utils::getParam<float>(node, "zMin", 0.0f);
        zLimits.y = Utils::getParam<float>(node, "zMax", 2.0f);

        classMap.classDistributions.resize(gridMetadata.height * gridMetadata.width * numZLevels());
        std::string ontologyPath = Utils::getParam<std::string>(node, "ontologyPath", "?");
        std::string targetGas = Utils::getParam<std::string>(node, "targetGas", "smoke");
        classMap.parseOntology(ontologyPath, targetGas);
    }

    void ClassMap3D::OnUpdate()
    {
        rclcpp::spin_some(node); // no subscribers for now, but whatever
        getUpdatedMapFromService();
        visualize();
    }

    std::vector<double> ClassMap3D::GetSourceProbability()
    {
        std::vector<double> probs(gridMetadata.height * gridMetadata.width, 0.0);
        GetSourceProbabilityInPlace(probs);
        return probs;
    }

    void ClassMap3D::GetSourceProbabilityInPlace(std::vector<double>& sourceProb)
    {
        for (size_t i = 0; i < sourceProb.size(); i++)
        {
            for (size_t z = 0; z < numZLevels(); z++)
            {
                size_t index = i + z * gridMetadata.width * gridMetadata.height;
                classMap.computeSourceProbability(classMap.classDistributions[index], sourceProb[i]);
            }
        }

        Grid<double> probGrid(sourceProb, wallsOccupancy, gridMetadata);
        Utils::NormalizeDistribution(probGrid);
    }

    double ClassMap3D::GetSourceProbabilityAt(const Vector3& point)
    {
        Vector3Int indices = gridMetadata.coordinatesToIndex(point.x, point.y, point.z);
        size_t index = indexOf(indices);
        double value = 0;
        classMap.computeSourceProbability(classMap.classDistributions[index], value);
        return value;
    }

    void ClassMap3D::getUpdatedMapFromService()
    {
        static std::string serviceName = Utils::getParam<std::string>(node, "semanticsService", "voxeland/get_class_distributions");
        static auto client = node->create_client<voxeland::srv::GetClassDistributions>(serviceName);

        auto request = std::make_shared<voxeland::srv::GetClassDistributions::Request>();
        // request->query_points;

        auto future = client->async_send_request(request);

        auto future_result = rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(1));
        if (future_result == rclcpp::FutureReturnCode::SUCCESS)
        {
            voxeland::srv::GetClassDistributions::Response::SharedPtr response = future.get();
            for (size_t i = 0; i < classMap.classDistributions.size(); i++)
                classMap.classDistributions[i].FromMsg(response->distributions[i].probabilities);
        }
        else
        {
            GSL_ERROR("Could not get an answer from the semantic map service");
        }
    }

    void ClassMap3D::visualize()
    {}

    size_t ClassMap3D::indexOf(const Vector3Int& v) const
    {
        return v.y + v.x * gridMetadata.width + v.z * gridMetadata.width * gridMetadata.height;
    }

    size_t ClassMap3D::numZLevels() const
    {
        return (zLimits.y - zLimits.x) / gridMetadata.cellSize;
    }

} // namespace GSL