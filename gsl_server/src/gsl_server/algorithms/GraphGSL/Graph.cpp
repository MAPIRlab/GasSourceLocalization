#include "Graph.hpp"
#include "Node.hpp"
#include "gsl_server/algorithms/Common/Utils/Pointers.hpp"
#include "gsl_server/core/Macros.hpp"
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>
#include <yaml-cpp/yaml.h>

namespace GSL
{
    Graph Graph::ReadFromDisk(const std::filesystem::path& folder, float cellSize, float nodeSeparationMultiplier, gmrfw::CGMRF_map::Parameters gmrfParams)
    {
        if (!std::filesystem::exists(folder))
        {
            GSL_ERROR("Graph folder '{}' does not exist!", folder.c_str());
            CLOSE_PROGRAM;
        }

        Graph graph;
        std::map<std::string, std::weak_ptr<Node>> byName;

        // create the nodes
        for (std::filesystem::path subfolder : std::filesystem::directory_iterator(folder))
        {
            if (!std::filesystem::is_directory(subfolder))
                continue;

            std::shared_ptr<Node> node;
            if (std::filesystem::exists(subfolder / "out.yaml"))
            {
                const YAML::Node yaml = YAML::LoadFile(subfolder / "out.yaml");
                Vector2 position;
                position.x = yaml["pos_x"].as<float>();
                position.y = yaml["pos_y"].as<float>();
                position = position * nodeSeparationMultiplier;
                node = std::make_shared<EmptyNode>(position);
            }
            else
            {
                Map2D map = Utils::parseMapData(subfolder / "occupancy.yaml", cellSize);
                map.metadata.origin = map.metadata.origin * nodeSeparationMultiplier;
                node = std::make_shared<RealNode>(map.AsGrid(), gmrfParams);
            }

            graph.nodes.push_back(node);
            std::string name = subfolder.stem();
            byName[name] = node;
            node->id = name;
        }

        // connect them to each other
        for (std::filesystem::path subfolder : std::filesystem::directory_iterator(folder))
        {
            if (!std::filesystem::is_directory(subfolder))
                continue;
            std::shared_ptr<Node> thisNode = byName.at(subfolder.stem()).lock();

            std::filesystem::path linksFolder = subfolder / "links";
            for (std::filesystem::path linkFile : std::filesystem::directory_iterator(linksFolder))
            {
                const YAML::Node yaml = YAML::LoadFile(linkFile);
                AABB2D aabb;
                aabb.min.x = yaml["min_x"].as<float>();
                aabb.min.y = yaml["min_y"].as<float>();
                aabb.max.x = yaml["max_x"].as<float>();
                aabb.max.y = yaml["max_y"].as<float>();

                Vector2 spawnPoint;
                spawnPoint.x = yaml["spawn_point_x"].as<float>();
                spawnPoint.y = yaml["spawn_point_y"].as<float>();

                std::string name = linkFile.stem();
                std::weak_ptr<Node> otherNode;
                if (!byName.contains(name))
                {
                    GSL_ERROR("Tried to create link between {} and {}, but {} does not exist!", subfolder.stem().c_str(), name, name);
                    CLOSE_PROGRAM;
                }
                otherNode = byName.at(name);

                thisNode->arcs.push_back(Arc{
                    .to = otherNode,
                    .weight = 1,
                    .aabb = aabb,
                    .spawnPoint = spawnPoint});
            }
        }

        return graph;
    }

    void Graph::AddObservation(Vector2 position, Vector2 wind, float gasConcentration)
    {
        bool accepted = false;
        for (auto node : nodes)
        {
            if (node->AddObservation(position, wind))
            {
                GSL_INFO("Observation accepted into node {}", node->id);
                accepted = true;
            }
            node->AddObservation(position, gasConcentration);
        }
        if (!accepted)
            GSL_INFO("Observation not accepted by any nodes!");
    }

    MarkerArray Graph::VisualizeGraph()
    {
        MarkerArray array;
        size_t id = 0;
        for (auto node : nodes)
        {
            Vector2 position = node->GetPosition();
            ColorRGBA color;
            if (Is<RealNode>(node))
                color = Utils::create_color(0, 1, 0);
            else
                color = Utils::create_color(1, 0, 0);

            Marker marker;
            marker.header.frame_id = "map";
            marker.type = Marker::SPHERE;
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.3;
            marker.color = color;
            marker.pose.position.x = position.x;
            marker.pose.position.y = position.y;
            marker.id = id;
            id++;
            array.markers.push_back(marker);

            // draw the arcs
            for (size_t i = 0; i < node->arcs.size(); i++)
            {
                Vector2 otherPos = node->arcs.at(i).to.lock()->GetPosition();
                Marker marker;
                marker.header.frame_id = "map";
                marker.type = Marker::ARROW;
                marker.scale.x = 0.02; // shaft diameter
                marker.scale.y = 0.05; // head diameter
                marker.color = Utils::create_color(0, 0, 1);
                marker.points.push_back(Point{}.set__x(position.x).set__y(position.y));
                marker.points.push_back(Point{}.set__x(otherPos.x).set__y(otherPos.y));
                marker.id = id;
                id++;
                array.markers.push_back(marker);
            }
        }
        return array;
    }

} // namespace GSL