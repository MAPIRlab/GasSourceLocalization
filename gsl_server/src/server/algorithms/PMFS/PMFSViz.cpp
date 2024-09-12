#include <gsl_server/algorithms/PMFS/PMFSViz.hpp>
#include <gsl_server/algorithms/PMFS/PMFS.hpp>
#include <gsl_server/Utils/Math.hpp>

namespace GSL
{
    namespace NQA = Utils::NQA;
    void PMFSViz::ShowHitProb(Grid<PMFS_internal::HitProbability> grid, const PMFS_internal::VisualizationSettings& settings,
                              const PMFS_internal::PublishersAndSubscribers& pubs)
    {
        Marker gasProbMarker = Utils::emptyMarker({0.2, 0.2}, pubs.clock);

        Marker confidenceMarker = gasProbMarker;

        for (int a = 0; a < grid.metadata.height; a++)
        {
            for (int b = 0; b < grid.metadata.width; b++)
            {
                if (grid.freeAt(a, b))
                {
                    auto coords = grid.metadata.indexToCoordinates(a, b);
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
                        valueToColor(grid.dataAt(a, b).confidence, 0, 1, Utils::valueColorMode::Linear);

                    p.z = settings.markers_height - 0.1;
                    confidenceMarker.points.push_back(p);
                    confidenceMarker.colors.push_back(colorConfidence);
                }
            }
        }

        pubs.markers.hitProbabilityMarkers->publish(gasProbMarker);
        pubs.markers.confidenceMarkers->publish(confidenceMarker);
    }

    void PMFSViz::ShowSourceProb(Grid<double> grid, const PMFS_internal::VisualizationSettings& settings,
                                 const PMFS_internal::PublishersAndSubscribers& pubs)
    {
        Marker sourceProbMarker = Utils::emptyMarker({0.2, 0.2}, pubs.clock);
        for (int a = 0; a < grid.metadata.height; a++)
        {
            for (int b = 0; b < grid.metadata.width; b++)
            {
                if (grid.freeAt(a, b))
                {
                    auto coords = grid.metadata.indexToCoordinates(a, b);
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
                                       const PMFS_internal::PublishersAndSubscribers& pubs, const GridMetadata& gridMetadata)
    {
        MarkerArray segmentMarker;
        for (int i = 0; i < QTleaves.size(); i++)
        {
            const NQA::Node* leaf = &QTleaves[i];
            Marker mark = Utils::emptyMarker({0.2, 0.2}, pubs.clock);
            mark.type = Marker::CUBE;
            Vector2 worldSpaceScale = (Vector2(leaf->size.y, leaf->size.x)) * gridMetadata.cellSize;

            auto coords = gridMetadata.indexToCoordinates(leaf->origin.x, leaf->origin.y, false) + (worldSpaceScale * 0.5f);

            Point p;
            p.x = coords.x;
            p.y = coords.y;
            p.z = 0;
            // mark.points.push_back(p);
            mark.pose.position = p;
            mark.id = i;
            mark.scale.x = worldSpaceScale.x - 0.05;
            mark.scale.y = worldSpaceScale.y - 0.05;
            mark.scale.z = 0.01;

            if (leaf->value <= 0.5)
                mark.color = Utils::create_color(0, 0, 0, 1);
            else
                mark.color = Utils::create_color(1, 1, 1, 1);
            segmentMarker.markers.push_back(mark);

            pubs.markers.quadtreePublisher->publish(segmentMarker);

            if (!rclcpp::ok())
                break;
        }
    }

    void PMFSViz::PlotWindVectors(Grid<Vector2> estimatedWindVectors, const PMFS_internal::VisualizationSettings& settings,
                                  const PMFS_internal::PublishersAndSubscribers& pubs)
    {
        MarkerArray arrow_array;
        // Add an ARROW marker for each node
        Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = pubs.clock->now();
        marker.ns = "WindVector";
        marker.type = Marker::ARROW;
        marker.action = Marker::ADD;

        // Get max wind vector in the map (to normalize the plot)
        double max_module = 0.0;
        for (size_t i = 0; i < estimatedWindVectors.data.size(); i++)
        {
            if (vmath::length(estimatedWindVectors.data[i]) > max_module)
                max_module = vmath::length(estimatedWindVectors.data[i]);
        }

        for (size_t i = 0; i < estimatedWindVectors.data.size(); i++)
        {
            if (estimatedWindVectors.occupancy[i] == Occupancy::Free)
            {
                double module = vmath::length(estimatedWindVectors.data[i]);
                double angle = std::atan2(estimatedWindVectors.data[i].y, estimatedWindVectors.data[i].x);
                if (module > 0.001)
                {
                    marker.id = i;
                    // Set the pose of the marker.
                    Vector2Int indices2D = estimatedWindVectors.metadata.indices2D(i);
                    Vector2 coords = estimatedWindVectors.metadata.indexToCoordinates(indices2D.x, indices2D.y);
                    marker.pose.position.x = coords.x;
                    marker.pose.position.y = coords.y;
                    marker.pose.position.z = settings.markers_height;
                    marker.pose.orientation = Utils::createQuaternionMsgFromYaw(angle);
                    // shape
                    marker.scale.x = estimatedWindVectors.metadata.cellSize * (module / max_module); // arrow length,
                    marker.scale.y = 0.03; // arrow width
                    marker.scale.z = 0.05; // arrow height
                    // color -> must normalize to [0-199]
                    size_t idx_color = 199 * (module / max_module);
                    marker.color.r = 1;
                    marker.color.g = 0;
                    marker.color.b = 0;
                    marker.color.a = 1.0;

                    // Push Arrow to array
                    arrow_array.markers.push_back(marker);
                }
            }
        }
        pubs.markers.windArrowMarkers->publish(arrow_array);
    }

} // namespace GSL