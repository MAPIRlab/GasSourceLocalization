#include <gsl_server/algorithms/Common/Algorithm.hpp>
#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>
#include <angles/angles.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <fstream>

namespace GSL
{

    Algorithm::Algorithm(std::shared_ptr<rclcpp::Node> _node) : node(_node), tfBuffer(node->get_clock())
    {}

    Algorithm::~Algorithm()
    {}

    void Algorithm::Initialize()
    {
        declareParameters();

        // Subscribers
        //------------
        using namespace std::placeholders;
        localizationSub = node->create_subscription<PoseWithCovarianceStamped>(getParam<std::string>("robot_location_topic", "amcl_pose"), 1,
                          std::bind(&Algorithm::localizationCallback, this, _1));
        rclcpp::Rate rate(1);
        while (resultLogging.robotPosesVector.size() == 0)
        {
            rate.sleep();
            rclcpp::spin_some(node);
            GSL_INFO("Waiting to hear from localization topic: {}", localizationSub->get_topic_name());
        }

        gasSub = node->create_subscription<olfaction_msgs::msg::GasSensor>(getParam<std::string>("enose_topic", "PID/Sensor_reading"), 1,
                 std::bind(&Algorithm::gasCallback, this, _1));

        windSub = node->create_subscription<olfaction_msgs::msg::Anemometer>(
                      getParam<std::string>("anemometer_topic", "Anemometer/WindSensor_reading"), 1, std::bind(&Algorithm::windCallback, this, _1));



        //extra safety net for when the middleware hangs and the node gets stuck in service/action spinning
        static auto exit_timer = node->create_wall_timer(std::chrono::seconds((int)resultLogging.maxSearchTime + 10), //extra time to make sure this only happens if the node is deadlocked
                                 []()
        {
            rclcpp::shutdown();
            GSL_ERROR("GLOBAL TIMEOUT WAS EXCEEDED, BUT NODE IS STILL RUNNING. STOPPING FORCEFULLY.");
            CLOSE_PROGRAM;
        });

        startTime = node->now();
        GSL_INFO_COLOR(fmt::terminal_color::blue, "INITIALIZATON COMPLETED");
    }

    void Algorithm::declareParameters()
    {
        resultLogging.maxSearchTime = getParam<double>("maxSearchTime", 1000.0);
        resultLogging.distanceThreshold = getParam<double>("distanceThreshold", 0.5);
        resultLogging.sourcePositionGT.x = getParam<float>("ground_truth_x", 0.0);
        resultLogging.sourcePositionGT.y = getParam<float>("ground_truth_y", 0.0);
        resultLogging.resultsFile = getParam<std::string>("resultsFile", "");
        resultLogging.navigationPathFile = getParam<std::string>("navigationPathFile", "");

        thresholdGas = getParam<double>("th_gas_present", 0.1);
        thresholdWind = getParam<double>("th_wind_present", 0.1);
    }

    void Algorithm::OnUpdate()
    {
        rclcpp::spin_some(node);
        stateMachine.getCurrentState()->OnUpdate();
    }

    bool Algorithm::HasEnded()
    {
        if ((node->now() - startTime).seconds() > resultLogging.maxSearchTime)
        {
            saveResultsToFile(GSLResult::Failure);
            return true;
        }

        return GetResult() != GSLResult::Running;
    }

    GSLResult Algorithm::GetResult()
    {
        return currentResult;
    }

    void Algorithm::localizationCallback(const PoseWithCovarianceStamped::SharedPtr msg)
    {
        // keep the most recent robot pose
        currentRobotPose = *msg;

        // Keep all poses for later distance estimation
        resultLogging.robotPosesVector.push_back(currentRobotPose);
    }

    void Algorithm::onGetMap(const OccupancyGrid::SharedPtr msg)
    {
        map = *msg;

        {
            GSL_INFO("\n--------------GSL---------------\n"
                     "Occupancy Map dimensions:\n"
                     "x_min:{:.2} x_max:{:.2}   -   y_min:{:.2} y_max:{:.2}\n"
                     "--------------------------------",
                     map.info.origin.position.x, map.info.origin.position.x + map.info.width * map.info.resolution, map.info.origin.position.y,
                     map.info.origin.position.y + map.info.height * map.info.resolution);
        }
    }

    void Algorithm::onGetCostMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        costmap = *msg;
    }

    void Algorithm::OnCompleteNavigation(GSLResult result, State* previousState)
    {
        if (previousState == waitForGasState.get())
            stateMachine.forceResetState(waitForGasState.get());
        else
            stateMachine.forceResetState(stopAndMeasureState.get());
    }

    GSLResult Algorithm::checkSourceFound()
    {
        // 1. Check that working time < max allowed time for search
        rclcpp::Duration time_spent = node->now() - startTime;
        if (time_spent.seconds() > resultLogging.maxSearchTime)
        {
            // Report failure, we were too slow
            GSL_INFO("FAILURE-> Time spent ({} s) > maxSearchTime = {}", time_spent.seconds(), resultLogging.maxSearchTime);
            saveResultsToFile(GSLResult::Failure);
            return GSLResult::Failure;
        }

        // 2. Distance from robot to source
        double Ax = currentRobotPose.pose.pose.position.x - resultLogging.sourcePositionGT.x;
        double Ay = currentRobotPose.pose.pose.position.y - resultLogging.sourcePositionGT.y;
        double dist = sqrt(pow(Ax, 2) + pow(Ay, 2));
        if (dist < resultLogging.distanceThreshold)
        {
            // GSL has finished with success!
            GSL_INFO("SUCCESS -> Time spent ({} s)", time_spent.seconds());
            saveResultsToFile(GSLResult::Success);
            return GSLResult::Success;
        }

        // In other case, we are still searching (keep going)
        return GSLResult::Running;
    }

    void Algorithm::saveResultsToFile(GSLResult result)
    {
        rclcpp::Duration time_spent = node->now() - startTime;
        double search_t = time_spent.seconds();

        // 2. Search distance
        double search_d = 0;

        const auto& robotPosesVector = resultLogging.robotPosesVector;
        for (size_t h = 1; h < robotPosesVector.size(); h++)
        {
            double Ax = robotPosesVector[h - 1].pose.pose.position.x - robotPosesVector[h].pose.pose.position.x;
            double Ay = robotPosesVector[h - 1].pose.pose.position.y - robotPosesVector[h].pose.pose.position.y;
            search_d += sqrt(pow(Ax, 2) + pow(Ay, 2));
        }

        // 3. Navigation distance (from robot to source)
        // Estimate the distances by getting a navigation path from Robot initial pose to Source points in the map
        double nav_d = 0;
        {
            PoseStamped sourcePositionGT;
            sourcePositionGT.header.frame_id = "map";
            sourcePositionGT.header.stamp = node->now();
            sourcePositionGT.pose.position.x = resultLogging.sourcePositionGT.x;
            sourcePositionGT.pose.position.y = resultLogging.sourcePositionGT.y;

            PoseStamped startPose;
            startPose.pose = resultLogging.robotPosesVector[0].pose.pose;
            startPose.header = resultLogging.robotPosesVector[0].header;
            std::optional<nav_msgs::msg::Path> plan = movingState->GetPlan(startPose, sourcePositionGT);
            if (plan)
            {
                const auto& path = plan.value().poses;
                for (int h = 1; h < path.size(); h++)
                {
                    double Ax = path[h - 1].pose.position.x - path[h].pose.position.x;
                    double Ay = path[h - 1].pose.position.y - path[h].pose.position.y;
                    nav_d += sqrt(pow(Ax, 2) + pow(Ay, 2));
                }
            }
            else
                GSL_ERROR("Could not calculate minimal navigation distance for logging results");
        }

        // 4. Nav time
        double nav_t = nav_d / 0.4; // assumming a constant speed of 0.4m/s
        std::string result_string =
            fmt::format("RESULT IS: Success={}, Search_d={:.2}, Nav_d={:.2}, Search_t={:.2}, Nav_t={:.2}\n", (int)result, search_d, nav_d, search_t, nav_t);
        GSL_INFO("{}", result_string);

        std::ofstream output_file(resultLogging.resultsFile, std::ios_base::app);
        if (output_file.is_open())
        {
            updateProximityResults(true);
            output_file << "---------------------------------------------------\n";
            for (const auto& prox : resultLogging.proximityResult)
                output_file << prox.time << " " << prox.distance << "\n";

#if 0
            for (PoseWithCovarianceStamped p : robotPosesVector)
                output_file << p.pose.pose.position.x << ", " << p.pose.pose.position.y << "\n";
#endif
            output_file.close();
        }
        else
            GSL_ERROR("Unable to open Results file at: {}", resultLogging.resultsFile.c_str());
    }

    bool Algorithm::isPointInsideMapBounds(const Vector2& point) const
    {
        const static Vector2 mapStart = {map.info.origin.position.x, map.info.origin.position.y};
        const static Vector2 mapEnd = mapStart + Vector2(map.info.width, map.info.height) * map.info.resolution;

        return point.x >= mapStart.x && point.x < mapEnd.x && point.y >= mapStart.y && point.y < mapEnd.y;
    }

    float Algorithm::ppmFromGasMsg(const olfaction_msgs::msg::GasSensor::SharedPtr msg)
    {
        // extract ppm reading from MOX sensors
        if (msg->raw_units == msg->UNITS_OHM)
        {
            double rs_r0 = msg->raw / 50000.0;
            return std::pow(rs_r0 / msg->calib_a, 1.0 / msg->calib_b);
        }
        else if (msg->raw_units == msg->UNITS_PPM)
        {
            return msg->raw;
        }
        else
        {
            GSL_ERROR("Unknown concentration unit: {}", (int)msg->raw_units);
            return 0;
        }
    }

    float Algorithm::gasCallback(const olfaction_msgs::msg::GasSensor::SharedPtr msg)
    {
        float ppm = ppmFromGasMsg(msg);
        stopAndMeasureState->addGasReading(ppm);
        waitForGasState->addMeasurement(ppm);
        return ppm;
    }

    PoseStamped Algorithm::windCallback(const olfaction_msgs::msg::Anemometer::SharedPtr msg)
    {
        float downWind_direction = angles::normalize_angle(msg->wind_direction);
        // Transform from anemometer ref_system to map ref_system using TF
        PoseStamped anemometer_downWind_pose, map_downWind_pose;
        try
        {
            anemometer_downWind_pose.header.frame_id = msg->header.frame_id;
            anemometer_downWind_pose.pose.position.x = 0.0;
            anemometer_downWind_pose.pose.position.y = 0.0;
            anemometer_downWind_pose.pose.position.z = 0.0;
            anemometer_downWind_pose.pose.orientation = Utils::createQuaternionMsgFromYaw(downWind_direction);

            map_downWind_pose = tfBuffer.buffer.transform(anemometer_downWind_pose, "map");
        }
        catch (tf2::TransformException& ex)
        {
            GSL_ERROR("{} - Error: {}", __FUNCTION__, ex.what());
            return PoseStamped();
        }

        stopAndMeasureState->addWindReading(msg->wind_speed, Utils::getYaw(map_downWind_pose.pose.orientation));
        return map_downWind_pose;
    }

    geometry_msgs::msg::PoseStamped Algorithm::getRandomPoseInMap()
    {
        int idx = 0;
        NavigateToPose::Goal goal;
        geometry_msgs::msg::PoseStamped p;
        p.header.frame_id = "map";
        p.header.stamp = node->now();
        double randomPoseDistance = 0.5;

        constexpr int safetyLimit = 10;
        for (int i = 0; i < safetyLimit; i++)
        {
            p.pose.position.x = Utils::uniformRandom(currentRobotPose.pose.pose.position.x - randomPoseDistance,
                                currentRobotPose.pose.pose.position.x + randomPoseDistance);
            p.pose.position.y = Utils::uniformRandom(currentRobotPose.pose.pose.position.y - randomPoseDistance,
                                currentRobotPose.pose.pose.position.y + randomPoseDistance);
            p.pose.orientation = Utils::createQuaternionMsgFromYaw(0.0);
            if (idx % 5 == 0)
            {
                randomPoseDistance += 0.5;
            }
            idx++;
            goal.pose = p;
            if (movingState->checkGoal(goal))
                return p;
        }

        return p;
    }

    bool Algorithm::isPointFree(const Vector2& point)
    {
        if (!isPointInsideMapBounds(point))
            return false;

        int h = (point.x - map.info.origin.position.x) / map.info.resolution;
        int v = (point.y - map.info.origin.position.y) / map.info.resolution;

        return map.data[v * map.info.width + h] == 0;
    }

    int8_t Algorithm::sampleCostmap(const Vector2& point)
    {
        if (!isPointInsideMapBounds(point))
            return 255;

        int h = (point.x - map.info.origin.position.x) / map.info.resolution;
        int v = (point.y - map.info.origin.position.y) / map.info.resolution;

        return map.data[v * map.info.width + h];
    }

    void Algorithm::updateProximityResults(bool forceUpdate)
    {
        double Ax = currentRobotPose.pose.pose.position.x - resultLogging.sourcePositionGT.x;
        double Ay = currentRobotPose.pose.pose.position.y - resultLogging.sourcePositionGT.y;
        double dist = sqrt(pow(Ax, 2) + pow(Ay, 2));

        constexpr double rewrite_distance = 0.5;
        if (forceUpdate || resultLogging.proximityResult.empty() || dist < (resultLogging.proximityResult.back().distance - rewrite_distance))
            resultLogging.proximityResult.push_back({(node->now() - startTime).seconds(), dist});
    }

} // namespace GSL