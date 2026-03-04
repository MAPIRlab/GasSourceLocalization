#include "Graph.hpp"
#include "Node.hpp"
#include "gsl_server/core/Macros.hpp"
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>
#include <yaml-cpp/yaml.h>

namespace GSL
{
    Graph Graph::ReadFromDisk(const std::filesystem::path& folder, float cellSize, gmrfw::CGMRF_map::Parameters gmrfParams)
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
                node = std::make_shared<EmptyNode>(position);
            }
            else
            {
                Map2D map = Utils::parseMapData(subfolder / "occupancy.yaml", cellSize);
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

} // namespace GSL