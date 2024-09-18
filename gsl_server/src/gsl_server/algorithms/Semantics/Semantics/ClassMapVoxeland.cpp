#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>
#include <gsl_server/algorithms/Semantics/Semantics/ClassMapVoxeland.hpp>
#include <voxeland/srv/get_class_distributions.hpp>

namespace GSL
{
    ClassMapVoxeland::ClassMapVoxeland(Grid2DMetadata _gridMetadata, std::vector<Occupancy>& occupancy, BufferWrapper& _bufferWrapper,
                                       const PoseWithCovarianceStamped& _currentRobotPose)
        : wallsOccupancy(occupancy), bufferWrapper(_bufferWrapper), currentRobotPose(_currentRobotPose)
    {
        node = std::make_shared<rclcpp::Node>("class_map");

        gridMetadata.cellSize = _gridMetadata.cellSize;
        gridMetadata.origin.x = _gridMetadata.origin.x;
        gridMetadata.origin.y = _gridMetadata.origin.y;
        gridMetadata.origin.z = Utils::getParam<float>(node, "zMin", 0.0f);
        gridMetadata.scale = _gridMetadata.scale;
        gridMetadata.dimensions.x = _gridMetadata.dimensions.x;
        gridMetadata.dimensions.y = _gridMetadata.dimensions.y;
        gridMetadata.dimensions.z = (Utils::getParam<float>(node, "zMax", 2.0f) - gridMetadata.origin.z) / gridMetadata.cellSize;

        classMap.classDistributions.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y *  gridMetadata.dimensions.z);
        std::string ontologyPath = Utils::getParam<std::string>(node, "ontologyPath", "?");
        std::string targetGas = Utils::getParam<std::string>(node, "targetGas", "smoke");
        classMap.parseOntology(ontologyPath, targetGas);
    }

    void ClassMapVoxeland::OnUpdate()
    {
        rclcpp::spin_some(node); // no subscribers for now, but whatever
        getUpdatedMapFromService();
        visualize();
    }

    std::vector<double> ClassMapVoxeland::GetSourceProbability()
    {
        std::vector<double> probs(gridMetadata.dimensions.x * gridMetadata.dimensions.y, 0.0);
        GetSourceProbabilityInPlace(probs);
        return probs;
    }

    void ClassMapVoxeland::GetSourceProbabilityInPlace(std::vector<double>& sourceProb)
    {
        for (size_t i = 0; i < sourceProb.size(); i++)
        {
            for (size_t z = 0; z < gridMetadata.dimensions.z; z++)
            {
                size_t index = i + z * gridMetadata.dimensions.x * gridMetadata.dimensions.y;
                classMap.computeSourceProbability(classMap.classDistributions[index], sourceProb[i]);
            }
        }

        Utils::NormalizeDistribution(sourceProb, wallsOccupancy);
    }

    double ClassMapVoxeland::GetSourceProbabilityAt(const Vector3& point)
    {
        Vector3Int indices = gridMetadata.coordinatesToIndex(point.x, point.y, point.z);
        size_t index = gridMetadata.indexOf(indices);
        double value = 0;
        classMap.computeSourceProbability(classMap.classDistributions[index], value);
        return value;
    }

    void ClassMapVoxeland::getUpdatedMapFromService()
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

    void ClassMapVoxeland::visualize()
    {}

} // namespace GSL