#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <gsl_server/algorithms/PMFS/PMFS.hpp>
#include <gsl_server/algorithms/PMFS/PMFSViz.hpp>

namespace GSL
{
    namespace NQA = Utils::NQA;
    void PMFSViz::ShowHitProb(Grid2D<PMFS_internal::HitProbability> grid, const PMFS_internal::VisualizationSettings& settings,
                              const PMFS_internal::PublishersAndSubscribers& pubs)
    {
        float markerSize = grid.metadata.cellSize * 0.95;
        Marker gasProbMarker = Utils::emptyMarker({markerSize, markerSize}, pubs.clock);

        Marker confidenceMarker = gasProbMarker;

        for (int b = 0; b < grid.metadata.dimensions.y; b++)
        {
            for (int a = 0; a < grid.metadata.dimensions.x; a++)
            {
                if (grid.freeAt(a, b))
                {
                    auto coords = grid.metadata.indicesToCoordinates(a, b);
                    Point p;
                    p.x = coords.x;
                    p.y = coords.y;
                    p.z = settings.markers_height;

                    // HIT
                    std_msgs::msg::ColorRGBA col_hit =
                        valueToColor(Utils::logOddsToProbability(grid.dataAt(a, b).logOdds), settings.hitLimits.x,
                                     settings.hitLimits.y, settings.hitMode);

                    p.z = settings.markers_height - 0.1;
                    gasProbMarker.points.push_back(p);
                    gasProbMarker.colors.push_back(col_hit);

                    // CONFIDENCE
                    std_msgs::msg::ColorRGBA colorConfidence =
                        valueToColor(grid.dataAt(a, b).confidence, 0, 1, Utils::ValueColorMode::Linear);

                    p.z = settings.markers_height - 0.1;
                    confidenceMarker.points.push_back(p);
                    confidenceMarker.colors.push_back(colorConfidence);
                }
            }
        }

        pubs.markers.hitProbabilityMarkers->publish(gasProbMarker);
        pubs.markers.confidenceMarkers->publish(confidenceMarker);
    }

    void PMFSViz::ShowSourceProb(Grid2D<double> grid, const PMFS_internal::VisualizationSettings& settings,
                                 const PMFS_internal::PublishersAndSubscribers& pubs)
    {
        float markerSize = grid.metadata.cellSize * 0.95;
        Marker sourceProbMarker = Utils::emptyMarker({markerSize, markerSize}, pubs.clock);
        for (int b = 0; b < grid.metadata.dimensions.y; b++)
        {
            for (int a = 0; a < grid.metadata.dimensions.x; a++)
            {
                if (grid.freeAt(a, b))
                {
                    auto coords = grid.metadata.indicesToCoordinates(a, b);
                    Point p;
                    p.x = coords.x;
                    p.y = coords.y;
                    p.z = settings.markers_height;

                    // SOURCE PROB
                    double prob_s = grid.dataAt(a, b);
                    std_msgs::msg::ColorRGBA color_source = Utils::valueToColor(
                        prob_s, settings.sourceLimits.x, settings.sourceLimits.y, settings.sourceMode);
                    sourceProbMarker.points.push_back(p);
                    sourceProbMarker.colors.push_back(color_source);
                }
            }
        }

        pubs.markers.sourceProbabilityMarkers->publish(sourceProbMarker);
    }

    void PMFSViz::DebugMapSegmentation(const std::vector<Utils::NQA::Node>& QTleaves,
                                       const PMFS_internal::PublishersAndSubscribers& pubs, const Grid2DMetadata& gridMetadata)
    {
        {
            MarkerArray clearArray;
            Marker clear;
            clear.action = Marker::DELETEALL;
            clearArray.markers.push_back(clear);
            pubs.markers.quadtreePublisher->publish(clearArray);
        }

        MarkerArray segmentMarker;
        for (int i = 0; i < QTleaves.size(); i++)
        {
            const NQA::Node* leaf = &QTleaves[i];
            Marker mark = Utils::emptyMarker({0, 0}, pubs.clock);
            mark.type = Marker::CUBE;
            Vector2 worldSpaceScale = (Vector2(leaf->size.x, leaf->size.y))*gridMetadata.cellSize;

            auto coords = gridMetadata.indicesToCoordinates(leaf->origin.x, leaf->origin.y, false) + (worldSpaceScale * 0.5f);

            Point p;
            p.x = coords.x;
            p.y = coords.y;
            p.z = 0;
            // mark.points.push_back(p);
            mark.pose.position = p;
            mark.id = i;
            mark.scale.x = worldSpaceScale.x - gridMetadata.cellSize * 0.2;
            mark.scale.y = worldSpaceScale.y - gridMetadata.cellSize * 0.2;
            mark.scale.z = 0.01;

            if (leaf->value == 0)
                mark.color = Utils::create_color(0, 0, 0, 1);
            else
                mark.color = Utils::create_color(1, 1, 1, 1);
            segmentMarker.markers.push_back(mark);
        }
        pubs.markers.quadtreePublisher->publish(segmentMarker);
    }

    void PMFSViz::VisualizeCoarseToFine(const std::vector<Utils::NQA::Node*>& activeNodes,
                                        const std::vector<Utils::NQA::Node*>& oldNodes,
                                        std::string_view topic,
                                        const Grid2DMetadata& gridMetadata)
    {
        static rclcpp::Node::SharedPtr debugNode = std::make_shared<rclcpp::Node>("ctf");
        static auto pub = debugNode->create_publisher<MarkerArray>(topic.data(), 1);

        GSL_INFO("Publishing coarseToFine visualization");
        {
            MarkerArray clearArray;
            Marker clear;
            clear.action = Marker::DELETEALL;
            clearArray.markers.push_back(clear);
            pub->publish(clearArray);
        }

        auto createMarker = [&](const NQA::Node* leaf, int id)
        {
            Marker mark = Utils::emptyMarker({0, 0}, debugNode->get_clock());
            mark.type = Marker::CUBE;
            Vector2 worldSpaceScale = (Vector2(leaf->size.x, leaf->size.y))*gridMetadata.cellSize;

            auto coords = gridMetadata.indicesToCoordinates(leaf->origin.x, leaf->origin.y, false) + (worldSpaceScale * 0.5f);

            Point p;
            p.x = coords.x;
            p.y = coords.y;
            p.z = 0;
            // mark.points.push_back(p);
            mark.pose.position = p;
            mark.id = id;
            mark.scale.x = worldSpaceScale.x - gridMetadata.cellSize * 0.2;
            mark.scale.y = worldSpaceScale.y - gridMetadata.cellSize * 0.2;
            mark.scale.z = 0.01;

            return mark;
        };

        MarkerArray segmentMarker;
        int id = 0;
        for (int i = 0; i < activeNodes.size(); i++)
        {
            const NQA::Node* leaf = activeNodes[i];
            Marker mark = createMarker(leaf, id);
            mark.color = Utils::create_color(1, 1, 1, 1);
            segmentMarker.markers.push_back(mark);
            id++;
        }

        for (int i = 0; i < oldNodes.size(); i++)
        {
            const NQA::Node* leaf = oldNodes[i];
            Marker mark = createMarker(leaf, id);
            mark.color = Utils::create_color(0.5, 0.5, 0.5, 1);
            segmentMarker.markers.push_back(mark);
            id++;
        }

        pub->publish(segmentMarker);
    }

    void PMFSViz::PlotWindVectors(Grid2D<Vector2> estimatedWindVectors, const PMFS_internal::VisualizationSettings& settings,
                                  const PMFS_internal::PublishersAndSubscribers& pubs)
    {
        MarkerArray arrow_array = Utils::createArrowsMarkers(estimatedWindVectors, settings.markers_height);
        pubs.markers.windArrowMarkers->publish(arrow_array);
    }

} // namespace GSL