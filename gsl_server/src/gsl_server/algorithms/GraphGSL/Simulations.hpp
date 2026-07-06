#pragma once
#include "Node.hpp"
#include "gsl_server/algorithms/Common/Simulation.hpp"
#include "gsl_server/algorithms/Common/Utils/Pointers.hpp"
#include "gsl_server/algorithms/Common/Utils/RosUtils.hpp"
#include "gsl_server/core/ros_typedefs.hpp"

namespace GSL::Graph_internal
{
    struct Options
    {
        bool cummulativeMap = true;
        float filamentsPerSecond = 5.0;
        float deltaTime = 0.1;
        float blurSigma = 1.5;
        float noiseSTDev = 0.25;
        float warmupTimeAcc = 4.0;
        size_t iterationLimit = 200;
        size_t minWarmupIterations = 1000;
        size_t maxWarmupIterations = 2000;
        float normalizationPower = 0.5;
    };

    struct SimWithResult
    {
        std::shared_ptr<Simulation> simulation;
        std::shared_ptr<std::vector<float>> hitMap;

        float ProportionInDoorway(size_t index, const std::vector<float>* map = nullptr) const;
    };

    struct Source
    {
        virtual Vector2 GetPoint() = 0;
    };
    struct PointSource : public Source
    {
        explicit PointSource(Vector2 point) : point(point) {}
        Vector2 point;
        Vector2 GetPoint() override
        {
            return point;
        }
    };
    struct DoorwaySource : public Source
    {
        explicit DoorwaySource(std::shared_ptr<DoorwayNode> doorway) : doorway(doorway) {}
        std::shared_ptr<DoorwayNode> doorway;
        Vector2 GetPoint() override
        {
            return doorway->aabb.center();
        }
    };

    // gas maps expected in each room, assuming a specific source location
    struct CompleteMap
    {
        std::shared_ptr<Source> source;
        std::map<std::shared_ptr<RoomNode>, std::vector<float>> gasMaps;
    };

    inline MarkerArray VisualizeCompleteMap(const CompleteMap& map, const std::vector<std::shared_ptr<PlaceNode>>& nodes, float nodeSeparationViz, float height)
    {
        MarkerArray array;

        // create a marker for the source location
        {
            Marker sourceMarker;
            sourceMarker.header.frame_id = "map";
            sourceMarker.type = Marker::SPHERE;
            sourceMarker.scale.set__x(0.2).set__y(0.2).set__z(0.2);
            sourceMarker.pose.position.set__x(map.source->GetPoint().x).set__y(map.source->GetPoint().y).set__z(0.3);
            sourceMarker.id = 0;
            sourceMarker.color = Utils::create_color(1, 1, 1);
            array.markers.push_back(sourceMarker);
        }

        size_t i = 1;
        for (const auto& node : nodes)
        {
            const auto room = As<RoomNode>(node);
            if (!room)
                continue;

            Grid2D<Occupancy> occupancy = room->GetOccupancy();
            std::vector<ColorRGBA> colors(occupancy.data.size());
            if (map.gasMaps.contains(room))
            {
                const auto& result = map.gasMaps.at(room);
                for (size_t i = 0; i < result.size(); i++)
                    colors.at(i) = Utils::valueToColor(result.at(i), 0, 1, Utils::ValueColorMode::Linear, Utils::Colors::ColorMaps::Viridis);
            }
            else
                for (size_t i = 0; i < colors.size(); i++)
                    colors.at(i) = Utils::valueToColor(0, 0, 1, Utils::ValueColorMode::Linear, Utils::Colors::ColorMaps::Viridis);

            Grid2DMetadata vizMetadata = occupancy.metadata;
            vizMetadata.origin = vizMetadata.origin * nodeSeparationViz;

            Marker marker = Utils::createPointsMarker(Grid2D<ColorRGBA>(colors, occupancy.occupancy, vizMetadata), height);
            marker.id = i++;
            array.markers.push_back(marker);
        }
        return array;
    }

    inline float SimWithResult::ProportionInDoorway(size_t index, const std::vector<float>* map) const
    {
        const auto& mask = simulation->outlets->mask;
        const auto& localHitMap = map ? *map : *hitMap;

        float sum = 0;
        for (size_t i = 0; i < mask.data.size(); i++)
            if (mask.occupancy.at(i) && mask.data.at(i) == index)
                sum += localHitMap.at(i);

        return sum / simulation->outlets->numCellsOutlet.at(index);
    }
} // namespace GSL::Graph_internal