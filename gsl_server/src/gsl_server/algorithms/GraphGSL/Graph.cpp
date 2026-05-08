#include "Graph.hpp"
#include "Node.hpp"
#include "gsl_server/algorithms/Common/Utils/Pointers.hpp"
#include "gsl_server/core/Macros.hpp"
#include "gsl_server/core/Profiling.hpp"
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
        std::map<std::string, std::weak_ptr<PlaceNode>> nodesByName;

        // create the nodes
        std::set<std::filesystem::path> orderedPaths;
        for (std::filesystem::path subfolder : std::filesystem::directory_iterator(folder))
            orderedPaths.insert(subfolder);

        for (const auto& subfolder : orderedPaths)
        {
            if (!std::filesystem::is_directory(subfolder))
                continue;

            std::shared_ptr<PlaceNode> node;
            if (std::filesystem::exists(subfolder / "out.yaml"))
            {
                const YAML::Node yaml = YAML::LoadFile(subfolder / "out.yaml");
                Vector2 position;
                position.x = yaml["pos_x"].as<float>();
                position.y = yaml["pos_y"].as<float>();
                node = std::make_shared<OutsideNode>(position);
            }
            else
            {
                Map2D map = Utils::parseMapData(subfolder / "occupancy.yaml", cellSize);
                node = std::make_shared<RoomNode>(map.AsGrid());
            }

            graph.nodes.push_back(node);
            std::string name = subfolder.stem();
            nodesByName[name] = node;
            node->id = name;
        }

        // connect them to each other
        for (std::filesystem::path subfolder : std::filesystem::directory_iterator(folder))
        {
            if (!std::filesystem::is_directory(subfolder))
                continue;
            std::string nameThisPlace = subfolder.stem();
            std::shared_ptr<PlaceNode> thisNode = nodesByName.at(nameThisPlace).lock();

            std::filesystem::path linksFolder = subfolder / "links";
            if (!std::filesystem::exists(linksFolder))
            {
                GSL_WARN("Node {} has no links folder!", nameThisPlace);
                continue;
            }

            for (std::filesystem::path linkFile : std::filesystem::directory_iterator(linksFolder))
            {
                const YAML::Node yaml = YAML::LoadFile(linkFile);
                AABB2D aabb;
                aabb.min.x = yaml["min_x"].as<float>();
                aabb.min.y = yaml["min_y"].as<float>();
                aabb.max.x = yaml["max_x"].as<float>();
                aabb.max.y = yaml["max_y"].as<float>();

                std::string nameOtherPlace = yaml["to"].as<std::string>();

                std::weak_ptr<PlaceNode> otherNode;
                if (!nodesByName.contains(nameOtherPlace))
                {
                    GSL_ERROR("Tried to create link between {} and {}, but {} does not exist!", subfolder.stem().c_str(), nameThisPlace, nameOtherPlace);
                    CLOSE_PROGRAM;
                }
                otherNode = nodesByName.at(nameOtherPlace);

                std::string nameDoorway = linkFile.stem();

                DoorwayNode doorway(nameDoorway);
                doorway.from = thisNode;
                doorway.to = otherNode;
                doorway.aabb = aabb;

                thisNode->doorways.push_back(doorway);
            }

            thisNode->UpdateDoorwayMask();
        }

        graph.gmrf_parameters = gmrfParams;
        graph.completeMap = Utils::parseMapData(folder / "occupancy.yaml", cellSize);
        graph.gmrf = std::make_shared<gmrfw::CGMRF_map>(ToGMRFOcc(graph.completeMap.AsGrid()), graph.gmrf_parameters, false, false);

        return graph;
    }

    size_t Graph::GetCorrespondingNodeIdx(Vector2 position)
    {
        for (size_t i = 0; i < nodes.size(); i++)
        {
            if (nodes.at(i)->IsValidPoint(position))
                return i;
        }
        return 0;
    }

    void Graph::AddObservation(Vector2 position, Vector2 wind, float gasConcentration)
    {
        // wind
        {
            constexpr float sigma = 0.01;
            float speed = vmath::length(wind);
            float direction = std::atan2(wind.y, wind.x);
            bool accepted = gmrf->insertObservation_GMRF(
                speed,
                direction,
                sigma, sigma,
                position.x, position.y);

            if (!accepted)
                GSL_WARN("Wind GMRF did not accept observation at {}", position);
        }

        // gas
        bool accepted = false;
        for (auto node : nodes)
        {
            if (node->AddObservation(position, gasConcentration))
            {
                GSL_INFO("Observation accepted into node {}", node->id);
                accepted = true;
            }
        }

        if (!accepted)
            GSL_WARN("Gas observation at {} not accepted by any nodes!", position);
    }

    void Graph::UpdateAllWindMaps()
    {
        ScopedStopwatch watch("Updating wind maps");

        gmrf->MAP_estimation_GMRF(10);
        for (auto node : nodes)
        {
            if (!Is<RoomNode>(node))
                continue;

            auto roomNode = As<RoomNode>(node);
            roomNode->UpdateWindMap(gmrf);
        }
    }

    gmrfw::TOccupancyMap Graph::ToGMRFOcc(const Grid2D<Occupancy> occupancy)
    {
        gmrfw::TOccupancyMap occMap;

        std::transform(occupancy.data.begin(), occupancy.data.end(), std::back_inserter(occMap.data), [](const Occupancy value) -> int8_t
                       { return static_cast<int8_t>(value); });

        occMap.width = occupancy.metadata.dimensions.x;
        occMap.height = occupancy.metadata.dimensions.y;
        occMap.resolution = occupancy.metadata.cellSize;
        occMap.origin_x = occupancy.metadata.origin.x;
        occMap.origin_y = occupancy.metadata.origin.y;

        return occMap;
    }

    MarkerArray Graph::VisualizeGraph()
    {
        MarkerArray array;
        size_t id = 0;
        for (auto node : nodes)
        {
            Vector2 position = node->GetPosition();
            if (Is<RoomNode>(node))
                position += As<RoomNode>(node)->GetOccupancy().metadata.origin * (nodeSeparationViz - 1);
            else
            {
                for (const DoorwayNode& doorway : node->doorways)
                {
                    auto otherNode = doorway.to.lock();
                    position += As<RoomNode>(otherNode)->GetOccupancy().metadata.origin * (nodeSeparationViz - 1) * (1. / node->doorways.size());
                }
            }

            ColorRGBA color;
            if (Is<RoomNode>(node))
                color = Utils::create_color(0, 1, 0);
            else
                color = Utils::create_color(1, 0, 0);

            // node marker
            {
                Marker marker;
                marker.header.frame_id = "map";
                marker.type = Marker::SPHERE;
                marker.scale.x = 0.3;
                marker.scale.y = 0.3;
                marker.scale.z = 0.3;
                marker.color = color;
                marker.pose.position.x = position.x;
                marker.pose.position.y = position.y;
                marker.pose.position.z = 0.5f;
                marker.id = id;
                id++;
                array.markers.push_back(marker);
            }

            // text
            {
                visualization_msgs::msg::Marker textMarker;
                textMarker.header.frame_id = "map";
                textMarker.id = id++;
                textMarker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                textMarker.scale.z = 0.2;
                textMarker.text = node->id;
                textMarker.pose.position.x = position.x + 0.5f;
                textMarker.pose.position.y = position.y + 0.5f;
                textMarker.pose.position.z = 1.f;

                textMarker.color.r = 0;
                textMarker.color.g = 0;
                textMarker.color.b = 0;
                textMarker.color.a = 1;
                array.markers.push_back(textMarker);
            }
            // draw the arcs
            for (size_t i = 0; i < node->doorways.size(); i++)
            {
                Vector2 otherPos = node->doorways.at(i).aabb.center();
                auto otherNode = node->doorways.at(i).to.lock();

                // if both are real, move the doorway node the average of the two
                // otherwise, just copy the movement of the real one
                if (Is<RoomNode>(node) && Is<RoomNode>(otherNode))
                {
                    otherPos += As<RoomNode>(node)->GetOccupancy().metadata.origin * (nodeSeparationViz - 1) * 0.5;
                    otherPos += As<RoomNode>(otherNode)->GetOccupancy().metadata.origin * (nodeSeparationViz - 1) * 0.5;
                }
                else if (Is<RoomNode>(node))
                    otherPos += As<RoomNode>(node)->GetOccupancy().metadata.origin * (nodeSeparationViz - 1);
                else
                    otherPos += As<RoomNode>(otherNode)->GetOccupancy().metadata.origin * (nodeSeparationViz - 1);

                // doorway Marker
                {
                    Marker marker;
                    marker.header.frame_id = "map";
                    marker.type = Marker::CUBE;
                    marker.scale.x = 0.3;
                    marker.scale.y = 0.3;
                    marker.scale.z = 0.3;
                    marker.color = Utils::create_color(0, 0, 1);
                    marker.pose.position.x = otherPos.x;
                    marker.pose.position.y = otherPos.y;
                    marker.pose.position.z = 0.5f;
                    marker.id = id;
                    id++;
                    array.markers.push_back(marker);
                }

                // arrow marker
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
            if (!Is<RoomNode>(node) || !selectedForVisualization.contains(node->id) || !selectedForVisualization.at(node->id))
                continue;

            auto roomNode = As<RoomNode>(node);
            Grid2D<Occupancy> occupancy = roomNode->GetOccupancy();

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
            if (!Is<RoomNode>(node) || !selectedForVisualization.contains(node->id) || !selectedForVisualization.at(node->id))
                continue;

            auto roomNode = As<RoomNode>(node);

            Grid2D<Vector2> windMap = roomNode->GetWindMap();
            Grid2DMetadata vizMetadata = windMap.metadata;
            vizMetadata.origin = vizMetadata.origin * nodeSeparationViz;

            MarkerArray windMarker = Utils::createArrowsMarkers(Grid2D<Vector2>(windMap.data, windMap.occupancy, vizMetadata), 0.7, 0.05, 0.5);
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