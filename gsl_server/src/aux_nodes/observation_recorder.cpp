#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <gsl_server/core/Logging.hpp>
#include <olfaction_msgs/msg/anemometer.hpp>
#include <olfaction_msgs/msg/gas_sensor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <yaml-cpp/yaml.h>

using olfaction_msgs::msg::Anemometer;
using olfaction_msgs::msg::GasSensor;
using PoseCov = geometry_msgs::msg::PoseWithCovarianceStamped;

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
    void WindCB(Anemometer::ConstSharedPtr msg);
    void GasCB(GasSensor::ConstSharedPtr msg);
    void PoseCB(PoseCov::ConstSharedPtr msg);

    void Deserialize();
    void PublishAll();
    void Serialize();
    void PublishTF(PoseCov pose);

private:
    std::optional<PoseCov> currentPose;
    float mostRecentConcentration;
    std::vector<Measurement> measurements;
    std::filesystem::path filePath;

private:
    rclcpp::Subscription<PoseCov>::SharedPtr poseSub;
    rclcpp::Subscription<Anemometer>::SharedPtr windSub;
    rclcpp::Subscription<GasSensor>::SharedPtr gasSub;
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
    poseSub = create_subscription<PoseCov>(pose_topic, 10, std::bind(&ObservationRecorder::PoseCB, this, std::placeholders::_1));

    std::string wind_topic = declare_parameter("wind_topic", "??");
    windSub = create_subscription<Anemometer>(wind_topic, 10, std::bind(&ObservationRecorder::WindCB, this, std::placeholders::_1));

    std::string gas_topic = declare_parameter("gas_topic", "??");
    gasSub = create_subscription<GasSensor>(gas_topic, 10, std::bind(&ObservationRecorder::GasCB, this, std::placeholders::_1));

    filePath = declare_parameter<std::string>("file_path", "??");
}

void ObservationRecorder::Run()
{
    rclcpp::spin(shared_from_this());

    Serialize();
}

void ObservationRecorder::WindCB(Anemometer::ConstSharedPtr msg)
{
    if (!currentPose)
        return;
    GSL_INFO("Received wind message");
    measurements.push_back({.pose = *currentPose, .wind = *msg, .gasConcentration = mostRecentConcentration});
}

void ObservationRecorder::GasCB(GasSensor::ConstSharedPtr msg)
{
    if (!currentPose)
        return;
    GSL_INFO("Received gas message");
    mostRecentConcentration = msg->raw;
}

void ObservationRecorder::PoseCB(PoseCov::ConstSharedPtr msg)
{
    // GSL_INFO("Received pose message");
    currentPose = *msg;
}

void ObservationRecorder::Serialize()
{
    if (measurements.size() == 0)
        return;
    std::ofstream yamlFile(filePath, std::ios_base::app);
    GSL_INFO("Writing messages to '{}'", filePath.c_str());
    YAML::Emitter emitter(yamlFile);

    // WIND
    emitter << YAML::Value << YAML::Block << YAML::BeginSeq;
    for (size_t i = 0; i < measurements.size(); i++)
    {
        emitter << YAML::BeginMap;

        emitter << YAML::Key << "pose" << YAML::Value << YAML::BeginMap;
        {
            emitter << YAML::Key << "x" << YAML::Value << measurements.at(i).pose.pose.pose.position.x;
            emitter << YAML::Key << "y" << YAML::Value << measurements.at(i).pose.pose.pose.position.y;
            emitter << YAML::Key << "qx" << YAML::Value << measurements.at(i).pose.pose.pose.orientation.x;
            emitter << YAML::Key << "qy" << YAML::Value << measurements.at(i).pose.pose.pose.orientation.y;
            emitter << YAML::Key << "qz" << YAML::Value << measurements.at(i).pose.pose.pose.orientation.z;
            emitter << YAML::Key << "qw" << YAML::Value << measurements.at(i).pose.pose.pose.orientation.w;
        }
        emitter << YAML::EndMap;

        emitter << YAML::Key << "wind" << YAML::Value << YAML::BeginMap;
        {
            emitter << YAML::Key << "direction" << YAML::Value << measurements.at(i).wind.wind_direction;
            emitter << YAML::Key << "speed" << YAML::Value << measurements.at(i).wind.wind_speed;
            emitter << YAML::Key << "frame" << YAML::Value << measurements.at(i).wind.header.frame_id;
        }
        emitter << YAML::EndMap;

        emitter << YAML::Key << "gas" << YAML::Value << YAML::BeginMap;
        emitter << YAML::Key << "concentration" << YAML::Value << measurements.at(i).gasConcentration;
        emitter << YAML::EndMap;

        emitter << YAML::EndMap;
    }
    emitter << YAML::EndSeq;

    yamlFile << "\n";
    yamlFile.close();
}
