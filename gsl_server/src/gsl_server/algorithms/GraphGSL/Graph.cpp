#include "Graph.hpp"
#include "Node.hpp"
#include "gsl_server/algorithms/Common/Utils/Pointers.hpp"
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

    void Graph::UpdateAllWindMaps()
    {
        // to make sure that the observations in one of the nodes also affect the rest of the maps, we can add "virtual" observations to all the neighbouring nodes
        // these observations will have the value of whatever wind vector was predicted by GMRF at the connecting doorway

        std::set<std::string> closedNodes;
        // would it make a difference to make this a priority queue so that we update nodes in order, based on how close they are to the measurements?
        std::queue<std::shared_ptr<RealNode>> dirtyNodes;

        for (auto node : nodes)
        {
            if (!Is<RealNode>(node))
                continue;

            auto realNode = As<RealNode>(node);
            if (realNode->isDirty())
            {
                dirtyNodes.push(realNode);
                // closedNodes.insert(realNode->id); //allow virtual measurements for nodes that also contain real ones?
            }
        }

        while (!dirtyNodes.empty())
        {
            auto realNode = dirtyNodes.front();
            dirtyNodes.pop();

            Grid2D<Vector2> windMap = realNode->GetWindMap();

            // add a virtual observation at spawnPoint which is equal to the average wind vector inside the doorway area
            for (auto arc : realNode->arcs)
            {
                if (closedNodes.contains(arc.to.lock()->id) || !Is<RealNode>(arc.to))
                    continue;

                Grid2DMetadata metadata = realNode->GetOccupancy().metadata;
                AABB2DInt aabbIdx{
                    metadata.coordinatesToIndices(arc.aabb.min),
                    metadata.coordinatesToIndices(arc.aabb.max)};

                Vector2 windVec;
                size_t count = 0;
                for (Vector2Int indices : aabbIdx)
                    if (metadata.indicesInBounds(indices) && realNode->GetOccupancy().freeAt(indices))
                    {
                        windVec += windMap.dataAt(indices);
                        count++;
                    }

                if (count > 0)
                {
                    windVec = windVec / count;
                    auto otherNode = As<RealNode>(arc.to.lock());
                    otherNode->AddObservation(arc.spawnPoint, windVec); // TODO lower confidence for these virtual measurements?
                    dirtyNodes.push(otherNode);
                }
                else
                    GSL_WARN("0 free cells in the connection between {} and {}! Probably not right!", realNode->id, arc.to.lock()->id);
            }

            closedNodes.insert(realNode->id);
        }

        // second round
        for (auto node : nodes)
        {
            if (!Is<RealNode>(node))
                continue;

            auto realNode = As<RealNode>(node);
            if (realNode->isDirty())
                realNode->GetWindMap();
        }
    }

    MarkerArray Graph::VisualizeGraph()
    {
        MarkerArray array;
        size_t id = 0;
        for (auto node : nodes)
        {
            Vector2 position = node->GetPosition() * nodeSeparationViz;
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
                Vector2 otherPos = node->arcs.at(i).to.lock()->GetPosition() * nodeSeparationViz;
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

    MarkerArray Graph::VisualizeOccupancy()
    {
        MarkerArray occArray;
        MarkerArray windArray;
        size_t occID = 0;
        for (auto node : nodes)
        {
            if (!Is<RealNode>(node) || !selectedForVisualization.contains(node->id) || !selectedForVisualization.at(node->id))
                continue;

            auto realNode = As<RealNode>(node);
            Grid2D<Occupancy> occupancy = realNode->GetOccupancy();

            Grid2DMetadata vizMetadata = occupancy.metadata;
            vizMetadata.origin = vizMetadata.origin * nodeSeparationViz;
            Marker occMarker = Utils::createPointsOccupancyMarker(Grid2D<Occupancy>(occupancy.occupancy, occupancy.occupancy, vizMetadata));
            occMarker.id = occID;
            occID++;

            occArray.markers.push_back(occMarker);
        }
        return occArray;
    }

    MarkerArray Graph::VisualizeWind()
    {
        MarkerArray windArray;
        for (auto node : nodes)
        {
            if (!Is<RealNode>(node) || !selectedForVisualization.contains(node->id) || !selectedForVisualization.at(node->id))
                continue;

            auto realNode = As<RealNode>(node);

            Grid2D<Vector2> windMap = realNode->GetWindMap();
            Grid2DMetadata vizMetadata = windMap.metadata;
            vizMetadata.origin = vizMetadata.origin * nodeSeparationViz;

            MarkerArray windMarker = Utils::createArrowsMarkers(Grid2D<Vector2>(windMap.data, windMap.occupancy, vizMetadata), 0, 0.5);
            MergeWindMarkers(windArray, windMarker);
        }
        return windArray;
    }

    void Graph::MergeWindMarkers(MarkerArray& all, const MarkerArray& _new)
    {
        size_t startingID = all.markers.size() > 0 ? all.markers.back().id + 1 : 0;
        for (Marker marker : _new.markers)
        {
            marker.id += startingID;
            all.markers.push_back(marker);
        }
    }
} // namespace GSL