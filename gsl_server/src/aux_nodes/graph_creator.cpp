#include "geometry_msgs/msg/point_stamped.hpp"
#include "gsl_server/algorithms/GraphGSL/Graph.hpp"
#include "gsl_server/algorithms/Semantics/Semantics/Common/AABB.hpp"
#include <filesystem>
#include <fstream>
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>
#include <gsl_server/core/Logging.hpp>
#include <imgui_gl/imgui_gl.h>
#include <imgui_gl/utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

using namespace GSL;
using namespace std::placeholders;
using geometry_msgs::msg::PointStamped;

enum class CreateType : int
{
    RealNode = 0,
    EmptyNode,
    Link
};

enum class SelectionMode : int
{
    None = -1,
    New,
    Min,
    Max
};

class GraphCreator : public rclcpp::Node
{
public:
    GraphCreator();
    ~GraphCreator();

private:
    void OnGraphUpdated();
    void Render();
    void CreateNodeWindow();
    void CreateEmptyNodeWindow();
    void CreateLinkWindow();
    void AABBTable();
    void DrawAABB(std_msgs::msg::ColorRGBA color);
    void DrawPoint(Vector2 point, std_msgs::msg::ColorRGBA color);

    Graph graph;
    Map2D completeMap;
    CreateType currentType = CreateType::RealNode;
    SelectionMode selectionMode = SelectionMode::New;

    std::string rootDirectory;
    AABB2D currentAABB;

    // node creation
    uint id_number = 0;
    std::string node_id = "";

    // empty creation
    uint empty_id_number = 0;
    Vector2 emptyPosition;

private:
    void OnClick(const PointStamped::SharedPtr msg);

    rclcpp::TimerBase::SharedPtr renderTimer;
    rclcpp::Subscription<PointStamped>::SharedPtr clickSub;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GraphCreator>();
    rclcpp::spin(node);
}

//--------------------------------------
//--------------------------------------
//--------------------------------------

GraphCreator::GraphCreator()
    : Node("graph_creator")
{
    ImguiGL::Setup(
        nullptr,
        "Graph Creator Tool",
        500,
        400,
        ImguiGL::FlagsFixedLayout());

    renderTimer = create_timer(std::chrono::milliseconds(30), std::bind(&GraphCreator::Render, this));
    clickSub = create_subscription<PointStamped>("/clicked_point", 1, std::bind(&GraphCreator::OnClick, this, _1));
    rootDirectory = declare_parameter<std::string>("root_directory", fmt::format("{}/my_graph", std::filesystem::current_path().c_str()));

    // load the map from a file and publish it to rviz
    auto yamlPath = declare_parameter<std::string>("map_yaml", "?");
    completeMap = Utils::parseMapData(yamlPath, std::nullopt);
    OccupancyGrid msg = Utils::toOccupancyGrid(completeMap.AsGrid());

    static auto mapPub = create_publisher<OccupancyGrid>("map", rclcpp::QoS(1).transient_local());
    mapPub->publish(msg);

    OnGraphUpdated();
}

GraphCreator::~GraphCreator()
{
    ImguiGL::Close();
}

void GraphCreator::OnGraphUpdated()
{
    static auto pub = create_publisher<MarkerArray>("/gsl_graph", rclcpp::QoS(1).transient_local());
    graph = Graph::ReadFromDisk(rootDirectory, 0.1, {});
    MarkerArray marker = graph.VisualizeGraph();
    pub->publish(marker);
}

void GraphCreator::Render()
{
    ImguiGL::StartFrame();
    ImguiGL::SetNextWindowFullscreen();
    ImGui::Begin("Main");
    {
        ImGui::RadioButton("Create Normal Node", (int*)&currentType, (int)CreateType::RealNode);
        ImGui::SameLine();
        ImGui::RadioButton("Create Empty Node", (int*)&currentType, (int)CreateType::EmptyNode);
        ImGui::SameLine();
        ImGui::RadioButton("Create Link", (int*)&currentType, (int)CreateType::Link);

        if (currentType == CreateType::RealNode)
            CreateNodeWindow();
        else if (currentType == CreateType::EmptyNode)
            CreateEmptyNodeWindow();
        else if (currentType == CreateType::Link)
            CreateLinkWindow();
    }
    ImGui::End();
    ImguiGL::Render();
}

void GraphCreator::CreateNodeWindow()
{
    if (selectionMode != SelectionMode::New && selectionMode != SelectionMode::Min && selectionMode != SelectionMode::Max)
        selectionMode = SelectionMode::New;

    ImGui::BeginChild("Node Window", ImVec2(0, 0), ImGuiChildFlags_Border, ImGuiWindowFlags_None);
    ImGui::Text("Node creation");
    ImGui::Separator();
    AABBTable();

    ImGui::InputText("Root directory", &rootDirectory);
    ImGui::InputText("Node id", &node_id);
    ImGui::SameLine();
    if (ImGui::Button("Save"))
    {
        if (!std::filesystem::exists(rootDirectory) && !std::filesystem::create_directories(rootDirectory))
            GSL_ERROR("Folder '{}' does not exist and could not be created! Not saving the node.", rootDirectory.c_str());
        else
        {
            // ensure the file structure is correct
            std::filesystem::path rootPath(rootDirectory);
            std::filesystem::create_directories(rootPath / node_id);
            std::filesystem::create_directories(rootPath / node_id / "links");

            // create the occupancy files

            // YAML
            {
                std::ofstream yamlFile(rootPath / node_id / "occupancy.yaml");
                YAML::Emitter emitter(yamlFile);
                emitter << YAML::BeginMap;
                emitter << YAML::Key << "image" << YAML::Value << "occupancy.pgm";
                emitter << YAML::Key << "resolution" << YAML::Value << completeMap.metadata.cellSize;
                emitter << YAML::Key << "origin" << YAML::Value << YAML::Flow << YAML::BeginSeq << currentAABB.min.x << currentAABB.min.y << 0 << YAML::EndSeq;
                emitter << YAML::Key << "occupied_thresh" << YAML::Value << 0.9;
                emitter << YAML::Key << "free_thresh" << YAML::Value << 0.1;
                emitter << YAML::Key << "negate" << YAML::Value << 0;
                emitter << YAML::EndMap;
                yamlFile.close();
            }

            // PGM
            {
                Map2D cropped = GridUtils::CropMap(completeMap.AsGrid(), currentAABB);
                std::ofstream file(rootPath / node_id / "occupancy.pgm");
                file << "P2\n"
                     << cropped.metadata.dimensions.x << " " << cropped.metadata.dimensions.y << "\n"
                     << "1\n";

                for (int row = cropped.metadata.dimensions.y - 1; row >= 0; row--)
                {
                    for (int col = 0; col < cropped.metadata.dimensions.x; col++)
                        file << (cropped.AsGrid().occupancyAt(col, row) ? 1 : 0) << " ";
                    file << "\n";
                }
                file.close();
            }

            GSL_INFO("Created node '{}' at '{}'", node_id, std::filesystem::canonical(rootPath).c_str());
            OnGraphUpdated();
        }
    }

    DrawAABB(Utils::create_color(0, 1, 0, 0.3));
    ImGui::EndChild();
}

void GraphCreator::CreateEmptyNodeWindow()
{
    ImGui::BeginChild("Empty Node Window", ImVec2(0, 0), ImGuiChildFlags_Border, ImGuiWindowFlags_None);
    ImGui::Text("Empty node creation");
    ImGui::Separator();
    ImGui::DragFloat2("Position", &emptyPosition.x, 0.02f);

    DrawPoint(emptyPosition, Utils::create_color(0, 1, 0, 0.3));

    ImGui::InputText("Root directory", &rootDirectory);
    ImGui::InputText("Node id", &node_id);
    if (ImGui::Button("Save"))
    {
        if (!std::filesystem::exists(rootDirectory) && !std::filesystem::create_directories(rootDirectory))
            GSL_ERROR("Folder '{}' does not exist and could not be created! Not saving the node.", rootDirectory.c_str());
        else
        {
            // ensure the file structure is correct
            std::filesystem::path rootPath(rootDirectory);
            std::filesystem::create_directories(rootPath / node_id);
            std::filesystem::create_directories(rootPath / node_id / "links");

            // YAML
            {
                std::ofstream yamlFile(rootPath / node_id / "out.yaml");
                YAML::Emitter emitter(yamlFile);
                emitter << YAML::BeginMap;
                emitter << YAML::Key << "pos_x" << YAML::Value << emptyPosition.x;
                emitter << YAML::Key << "pos_y" << YAML::Value << emptyPosition.y;
                emitter << YAML::EndMap;
                yamlFile.close();
            }
            GSL_INFO("Created empty node '{}' at '{}'", node_id, std::filesystem::canonical(rootPath).c_str());
            OnGraphUpdated();
        }
    }
    ImGui::EndChild();
}

void GraphCreator::CreateLinkWindow()
{
    ImGui::BeginChild("Link Window", ImVec2(0, 0), ImGuiChildFlags_Border, ImGuiWindowFlags_None);
    ImGui::Text("Link creation");
    ImGui::Separator();
    AABBTable();
    ImGui::InputText("Root directory", &rootDirectory);

    if (!std::filesystem::exists(rootDirectory))
    {
        ImGui::Text("Root directory does not exist! You should create it.\nIf you save a node, the directory will be created automatically.");
        ImGui::EndChild();
        return;
    }

    // create two lists of nodes
    std::set<std::filesystem::path> sortedSet;
    for (std::filesystem::path path : std::filesystem::directory_iterator(rootDirectory))
        if (std::filesystem::is_directory(path))
            sortedSet.insert(path.stem());

    std::vector<std::string> node_names;
    for (const auto& path : sortedSet)
        node_names.push_back(path.c_str());

    if (node_names.size() == 0)
    {
        ImGui::Text("No nodes in the graph yet, there's nothing to link");
        ImGui::EndChild();
        return;
    }

    auto selectNode = [&](size_t& idx, const char* ID)
    {
        ImGui::PushID(ID);
        if (ImGui::BeginCombo("Selected Instance", node_names.at(idx).c_str()))
        {
            for (size_t i = 0; i < node_names.size(); i++)
                if (ImGui::Selectable(node_names.at(i).c_str()))
                    idx = i;

            ImGui::EndCombo();
        }
        ImGui::PopID();
    };

    static size_t firstNode = 0;
    static size_t secondNode = 0;
    size_t old_firstNode = firstNode;
    size_t old_secondNode = secondNode;

    if (ImGui::BeginTable("table1", 2))
    {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        selectNode(firstNode, "firstNode");
        ImGui::TableSetColumnIndex(1);
        selectNode(secondNode, "secondNode");

        ImGui::EndTable();
    }

    std::string firstID = node_names.at(firstNode);
    std::string secondID = node_names.at(secondNode);

    static std::string linkName = "";
    if (old_firstNode != firstNode || old_secondNode != secondNode)
    {
        std::array<std::string, 2> namesArray{firstID, secondID};
        std::sort(namesArray.begin(), namesArray.end());
        linkName = namesArray.at(0) + "-" + namesArray.at(1);
    }

    ImGui::InputText("Link name", &linkName);

    if (ImGui::Button("Save"))
    {
        // ensure the file structure is correct
        std::filesystem::path rootPath(rootDirectory);

        // first ->second
        {
            std::filesystem::create_directories(rootPath / firstID / "links");

            std::ofstream yamlFile(rootPath / firstID / "links" / fmt::format("{}.yaml", linkName));
            YAML::Emitter emitter(yamlFile);
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "to" << YAML::Value << secondID;
            emitter << YAML::Key << "min_x" << YAML::Value << currentAABB.min.x;
            emitter << YAML::Key << "min_y" << YAML::Value << currentAABB.min.y;
            emitter << YAML::Key << "max_x" << YAML::Value << currentAABB.max.x;
            emitter << YAML::Key << "max_y" << YAML::Value << currentAABB.max.y;

            emitter << YAML::EndMap;
            yamlFile.close();
        }

        // second -> first
        {
            std::filesystem::create_directories(rootPath / secondID / "links");

            std::ofstream yamlFile(rootPath / secondID / "links" / fmt::format("{}.yaml", linkName));
            YAML::Emitter emitter(yamlFile);
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "to" << YAML::Value << firstID;
            emitter << YAML::Key << "min_x" << YAML::Value << currentAABB.min.x;
            emitter << YAML::Key << "min_y" << YAML::Value << currentAABB.min.y;
            emitter << YAML::Key << "max_x" << YAML::Value << currentAABB.max.x;
            emitter << YAML::Key << "max_y" << YAML::Value << currentAABB.max.y;

            emitter << YAML::EndMap;
            yamlFile.close();
        }
        GSL_INFO("Created link beween nodes '{}' and '{}' at '{}'", firstID, secondID, std::filesystem::canonical(rootPath).c_str());
        OnGraphUpdated();
    }

    DrawAABB(Utils::create_color(1, 0, 0, 0.3));

    ImGui::EndChild();
}

void GraphCreator::AABBTable()
{
    if (ImGui::BeginTable("table1", 2))
    {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::DragFloat2("Min", &currentAABB.min.x, 0.02f);
        ImGui::TableSetColumnIndex(1);
        ImGui::RadioButton("##Minradio", (int*)&selectionMode, (int)SelectionMode::Min);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::DragFloat2("Max", &currentAABB.max.x, 0.02f);
        ImGui::TableSetColumnIndex(1);
        ImGui::RadioButton("##Maxradio", (int*)&selectionMode, (int)SelectionMode::Max);

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(1);
        ImGui::RadioButton("New", (int*)&selectionMode, (int)SelectionMode::New);

        ImGui::EndTable();
    }
}

void GraphCreator::DrawAABB(std_msgs::msg::ColorRGBA color)
{
    static auto pub = create_publisher<Marker>("/currentAABB", 1);
    Marker marker;
    marker.header.frame_id = "map";
    marker.type = Marker::CUBE;
    marker.color = color;

    Vector2 pos = currentAABB.center();
    marker.pose.position.x = pos.x;
    marker.pose.position.y = pos.y;

    Vector2 size = currentAABB.size();
    marker.scale.x = size.x;
    marker.scale.y = size.y;
    marker.scale.z = 0.2;

    pub->publish(marker);
}

void GraphCreator::DrawPoint(Vector2 point, std_msgs::msg::ColorRGBA color)
{
    static auto pub = create_publisher<Marker>("/spawnPoint", 1);
    Marker marker;
    marker.header.frame_id = "map";
    marker.type = Marker::SPHERE;
    marker.color = color;

    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    pub->publish(marker);
}

void GraphCreator::OnClick(const PointStamped::SharedPtr msg)
{
    if (currentType == CreateType::EmptyNode)
    {
        emptyPosition.x = msg->point.x;
        emptyPosition.y = msg->point.y;
        empty_id_number++;
        node_id = fmt::format("out_{}", empty_id_number);
        return;
    }

    if (selectionMode == SelectionMode::New)
    {
        currentAABB.min.x = msg->point.x;
        currentAABB.min.y = msg->point.y;
        currentAABB.max.x = msg->point.x;
        currentAABB.max.y = msg->point.y;
        selectionMode = SelectionMode::Max;
        id_number++;
        node_id = fmt::format("room_{}", id_number);
    }
    else if (selectionMode == SelectionMode::Min)
    {
        currentAABB.min.x = msg->point.x;
        currentAABB.min.y = msg->point.y;
        selectionMode = SelectionMode::Max;
    }
    else if (selectionMode == SelectionMode::Max)
    {
        currentAABB.max.x = msg->point.x;
        currentAABB.max.y = msg->point.y;
        selectionMode = SelectionMode::New;
    }

    // make sure we don't have an inverted box
    if (currentAABB.min.x > currentAABB.max.x || currentAABB.min.y > currentAABB.max.y)
    {
        Vector2 temp = currentAABB.max;
        currentAABB.max = currentAABB.min;
        currentAABB.min = temp;
    }
}
