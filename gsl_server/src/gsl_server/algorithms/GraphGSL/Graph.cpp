#include "Graph.hpp"
#include "Node.hpp"
#include "gsl_server/algorithms/Common/Utils/Pointers.hpp"
#include "gsl_server/core/Macros.hpp"
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>
#include <yaml-cpp/yaml.h>

namespace GSL
{
    Graph Graph::ReadFromDisk(const std::filesystem::path& folder)
    {
        Graph graph;
        gmrfw::CGMRF_map::Parameters gmrfParams{}; // TODO
        std::map<std::string, std::weak_ptr<Node>> byName;

        // create the nodes
        for (std::filesystem::path subfolder : std::filesystem::directory_iterator(folder))
        {
            if (!std::filesystem::is_directory(subfolder))
                continue;

            Grid2DMetadata gridMetadata;
            std::vector<Occupancy> occupancy;
            Utils::parseMapData(subfolder / "occupancy.yaml", gridMetadata, occupancy);

            std::shared_ptr<Node> node = std::make_shared<RealNode>(gridMetadata, occupancy, gmrfParams);

            graph.nodes.push_back(node);
            byName[subfolder.stem()] = node;
        }

        // connect them to each other
        for (std::filesystem::path subfolder : std::filesystem::directory_iterator(folder))
        {
            if (!std::filesystem::is_directory(subfolder))
                continue;
            auto thisNode = As<RealNode>(byName.at(subfolder.stem()).lock());

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
                if (name == "out")
                {
                    auto outNode = std::make_shared<EmptyNode>(spawnPoint);
                    graph.nodes.push_back(outNode);
                    otherNode = outNode;
                }
                else
                {
                    if (!byName.contains(name))
                    {
                        GSL_ERROR("Tried to create link between {} and {}, but {} does not exist!", subfolder.stem().c_str(), name, name);
                        CLOSE_PROGRAM;
                    }
                    otherNode = byName.at(name);
                }

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