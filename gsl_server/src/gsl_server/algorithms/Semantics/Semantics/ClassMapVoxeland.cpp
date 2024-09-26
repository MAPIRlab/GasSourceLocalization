#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>
#include <gsl_server/algorithms/Common/Utils/Time.hpp>
#include <gsl_server/algorithms/Semantics/Semantics/ClassMapVoxeland.hpp>

namespace GSL
{
    ClassMapVoxeland::ClassMapVoxeland(Grid2DMetadata _gridMetadata, std::vector<Occupancy>& occupancy, BufferWrapper& _bufferWrapper,
                                       const PoseWithCovarianceStamped& _currentRobotPose, rclcpp::Node::SharedPtr _node)
        : wallsOccupancy(occupancy), bufferWrapper(_bufferWrapper), currentRobotPose(_currentRobotPose), node(_node)
    {
        // node = std::make_shared<rclcpp::Node>("class_map");

        //Map and metadata
        gridMetadata.cellSize = _gridMetadata.cellSize;
        gridMetadata.origin.x = _gridMetadata.origin.x;
        gridMetadata.origin.y = _gridMetadata.origin.y;
        gridMetadata.origin.z = Utils::getParam<float>(node, "zMin", 0.0f);
        gridMetadata.scale = _gridMetadata.scale;
        gridMetadata.dimensions.x = _gridMetadata.dimensions.x;
        gridMetadata.dimensions.y = _gridMetadata.dimensions.y;
        gridMetadata.dimensions.z = (Utils::getParam<float>(node, "zMax", 2.0f) - gridMetadata.origin.z) / gridMetadata.cellSize;
        classMap.classDistributions.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y * gridMetadata.dimensions.z);

        //Ontology
        std::string ontologyPath = Utils::getParam<std::string>(node, "ontologyPath", "?");
        std::string targetGas = Utils::getParam<std::string>(node, "targetGas", "smoke");
        classMap.parseOntology(ontologyPath, targetGas);

        // Client
        std::string serviceName = Utils::getParam<std::string>(node, "semanticsService", "/voxeland/get_class_distributions");
        rmw_qos_profile_t qos{.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE};
        client = node->create_client<voxeland::srv::GetClassDistributions>(serviceName,qos);
        while (!client->wait_for_service(std::chrono::seconds(2)) && rclcpp::ok())
            GSL_INFO("Waiting for voxeland service: {}", client->get_service_name());
        initializeRequestPoints();
    }

    void ClassMapVoxeland::initializeRequestPoints()
    {
        for (size_t i = 0; i < classMap.classDistributions.size(); i++)
        {
            Vector3 pos = gridMetadata.indicesToCoordinates(i);
            Point point;
            point.x = pos.x;
            point.y = pos.y;
            point.z = pos.z;
            requestPoints.push_back(point);
        }
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
        #pragma omp parallel for collapse (2)
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
        Vector3Int indices = gridMetadata.coordinatesToIndices(point.x, point.y, point.z);
        size_t index = gridMetadata.indexOf(indices);
        double value = 0;
        classMap.computeSourceProbability(classMap.classDistributions[index], value);
        return value;
    }

    void ClassMapVoxeland::getUpdatedMapFromService()
    {
        //only run this stuff every few seconds
        static Utils::Time::Countdown serviceCD;
        if(!serviceCD.isDone())
            return;

        serviceCD.Restart(3);

        auto request = std::make_shared<voxeland::srv::GetClassDistributions::Request>();
        request->query_points = requestPoints; //TODO probably better to avoid copying this

        Utils::Time::Stopwatch watch;
        auto future = client->async_send_request(request);
        GSL_INFO("Sending Vxl Request {}", future.request_id);
        auto future_result = rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(10));
        if (future_result == rclcpp::FutureReturnCode::SUCCESS)
        {
            GSL_INFO("Received response in {}s", watch.ellapsed());
            client->remove_pending_request(future);
            voxeland::srv::GetClassDistributions::Response::SharedPtr response = future.get();
            for (size_t i = 0; i < classMap.classDistributions.size(); i++)
                classMap.FromMsg(i, response->distributions[i].probabilities);
        }
        else
            GSL_ERROR("Could not get an answer from the semantic map service (FutureReturnCode {})", rclcpp::to_string(future_result));
    }

    void ClassMapVoxeland::visualize()
    {}

} // namespace GSL