#include "Graph.hpp"
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>
#include <yaml-cpp/yaml.h>
#include "Node.hpp"

namespace GSL
{
    Graph Graph::ReadFromDisk(const std::filesystem::path& folder)
    {
        Graph graph;
        gmrfw::CGMRF_map::Parameters gmrfParams{}; // TODO
        std::map<std::string, std::weak_ptr<Node>> byName;

        // create the nodes
        for (std::filesystem::path subfolder : folder)
        {
            if (!std::filesystem::is_directory(subfolder))
                continue;

            Grid2DMetadata gridMetadata;
            std::vector<Occupancy> occupancy;
            Utils::parseMapData(subfolder / "occupancy.yaml", gridMetadata, occupancy);

            std::shared_ptr<Node> node = std::make_shared<Node>(gridMetadata, occupancy, gmrfParams);

            graph.nodes.push_back(node);
            byName[subfolder.stem()] = node;
        }

        // connect them to each other
        for (std::filesystem::path subfolder : folder)
        {
            if (!std::filesystem::is_directory(subfolder))
                continue;
            std::weak_ptr<Node> thisNode = byName.at(subfolder.stem());

            std::filesystem::path linksFolder = subfolder / "links";
            for (std::filesystem::path linkFile : linksFolder)
            {
                std::string name = linkFile.stem();
                std::weak_ptr<Node> otherNode = byName.at(name);

                const YAML::Node yaml = YAML::LoadFile(linkFile);
                AABB2D aabb;
                aabb.min.x = yaml["min_x"].as<float>();
                aabb.min.y = yaml["min_y"].as<float>();
                aabb.max.x = yaml["max_x"].as<float>();
                aabb.max.y = yaml["max_y"].as<float>();

                Vector2 spawnPoint;
                spawnPoint.x = yaml["spawn_point_x"].as<float>();
                spawnPoint.y = yaml["spawn_point_y"].as<float>();

                thisNode.lock()->arcs.push_back(Arc{
                    .to = otherNode,
                    .weight = 1,
                    .aabb = aabb,
                    .spawnPoint= spawnPoint
                });
            }
        }

        return graph;
    }

} // namespace GSL