#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <gsl_server/core/Logging.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <olfaction_msgs/msg/anemometer.hpp>
#include <olfaction_msgs/msg/gas_sensor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <yaml-cpp/yaml.h>

using olfaction_msgs::msg::Anemometer;
using olfaction_msgs::msg::GasSensor;
using PoseCov = geometry_msgs::msg::PoseWithCovarianceStamped;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<PoseCov, Anemometer, GasSensor>;
using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

struct Measurement
{
    // timestamp?
    PoseCov pose;
    Anemometer wind;
    float gasConcentration;
};

class ObservationRecorder : public rclcpp::Node
{
public:
    ObservationRecorder();
    void Run();

private:
    void SynchronizedCB(PoseCov::ConstSharedPtr pose, Anemometer::ConstSharedPtr wind, GasSensor::ConstSharedPtr gas);

    void Deserialize();
    void PublishAll();
    void Serialize();
    void PublishTF(PoseCov pose);

private:
    std::vector<Measurement> measurements;
    std::filesystem::path filePath;

private:
    message_filters::Subscriber<PoseCov> poseSub;
    message_filters::Subscriber<Anemometer> windSub;
    message_filters::Subscriber<GasSensor> gasSub;
    std::shared_ptr<Synchronizer> sync;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObservationRecorder>();
    node->Run();
}

//--------------------------------------
//--------------------------------------
//--------------------------------------
//--------------------------------------

ObservationRecorder::ObservationRecorder()
    : Node("ObservationRecorder")
{
    std::string pose_topic = declare_parameter("pose_topic", "??");
    poseSub.subscribe(this, pose_topic, rmw_qos_profile_default);

    std::string wind_topic = declare_parameter("wind_topic", "??");
    windSub.subscribe(this, wind_topic, rmw_qos_profile_default);

    std::string gas_topic = declare_parameter("gas_topic", "??");
    gasSub.subscribe(this, gas_topic, rmw_qos_profile_default);

    sync = std::make_shared<Synchronizer>(SyncPolicy(10), poseSub, windSub, gasSub);
    sync->registerCallback(std::bind(&ObservationRecorder::SynchronizedCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    filePath = declare_parameter<std::string>("file_path", "??");

    GSL_INFO("Listening on:\n\tGas:'{}'\n\tWind:'{}'\n\tPose:'{}'", gas_topic, wind_topic, pose_topic);
    GSL_INFO("Writing to '{}'", filePath.c_str());
}

void ObservationRecorder::Run()
{
    rclcpp::spin(shared_from_this());
}

void ObservationRecorder::SynchronizedCB(PoseCov::ConstSharedPtr pose, Anemometer::ConstSharedPtr wind, GasSensor::ConstSharedPtr gas)
{
    GSL_INFO("Received synchronized messages");
    measurements.push_back({.pose = *pose, .wind = *wind, .gasConcentration = (float)gas->raw});
    Serialize();
}

void ObservationRecorder::Serialize()
{
    std::ofstream yamlFile(filePath, std::ios_base::app);
    YAML::Emitter emitter(yamlFile);

    emitter << YAML::Value << YAML::Block << YAML::BeginSeq;
    emitter << YAML::BeginMap;

    Measurement& measurement = measurements.back();

    emitter << YAML::Key << "pose" << YAML::Value << YAML::BeginMap;
    {
        emitter << YAML::Key << "x" << YAML::Value << measurement.pose.pose.pose.position.x;
        emitter << YAML::Key << "y" << YAML::Value << measurement.pose.pose.pose.position.y;
        emitter << YAML::Key << "qx" << YAML::Value << measurement.pose.pose.pose.orientation.x;
        emitter << YAML::Key << "qy" << YAML::Value << measurement.pose.pose.pose.orientation.y;
        emitter << YAML::Key << "qz" << YAML::Value << measurement.pose.pose.pose.orientation.z;
        emitter << YAML::Key << "qw" << YAML::Value << measurement.pose.pose.pose.orientation.w;
    }
    emitter << YAML::EndMap;

    emitter << YAML::Key << "wind" << YAML::Value << YAML::BeginMap;
    {
        emitter << YAML::Key << "direction" << YAML::Value << measurement.wind.wind_direction;
        emitter << YAML::Key << "speed" << YAML::Value << measurement.wind.wind_speed;
        emitter << YAML::Key << "frame" << YAML::Value << measurement.wind.header.frame_id;
    }
    emitter << YAML::EndMap;

    emitter << YAML::Key << "gas" << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "concentration" << YAML::Value << measurement.gasConcentration;
    emitter << YAML::EndMap;

    emitter << YAML::EndMap;
    emitter << YAML::EndSeq;

    yamlFile << "\n";
    yamlFile.close();
}
