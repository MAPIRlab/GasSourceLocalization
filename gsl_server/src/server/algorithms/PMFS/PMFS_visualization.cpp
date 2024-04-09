#include <gsl_server/algorithms/PMFS/PMFS.hpp>
#include <gsl_server/Utils/Math.hpp>

namespace GSL
{
    namespace NQA = Utils::NQA;
    void PMFS::showWeights()
    {
        Marker sourceProbMarker = Utils::emptyMarker({0.2, 0.2}, node->get_clock());

        Marker gasProbMarker = Utils::emptyMarker({0.2, 0.2}, node->get_clock());

        Marker confidenceMarker = gasProbMarker;

        Grid<HitProbability> hitProbGrid(hitProbability, simulationOccupancy, gridMetadata);
        Grid<double> sourceGrid(sourceProbability, simulationOccupancy, gridMetadata);


        for (int a = 0; a < gridMetadata.height; a++)
        {
            for (int b = 0; b < gridMetadata.width; b++)
            {
                if (sourceGrid.freeAt(a,b))
                {
                    auto coords = gridMetadata.indexToCoordinates(a, b);
                    Point p;
                    p.x = coords.x;
                    p.y = coords.y;
                    p.z = settings.visualization.markers_height;

                    // SOURCE PROB
                    double prob_s = sourceGrid.dataAt(a, b);
                    std_msgs::msg::ColorRGBA color_source = Utils::valueToColor(
                        prob_s, settings.visualization.sourceLimits.x, settings.visualization.sourceLimits.y, settings.visualization.sourceMode);
                    sourceProbMarker.points.push_back(p);
                    sourceProbMarker.colors.push_back(color_source);

                    // HIT
                    std_msgs::msg::ColorRGBA col_hit =
                        valueToColor(Utils::logOddsToProbability(hitProbGrid.dataAt(a,b).logOdds), settings.visualization.hitLimits.x,
                                     settings.visualization.hitLimits.y, settings.visualization.hitMode);

                    p.z = settings.visualization.markers_height - 0.1;
                    gasProbMarker.points.push_back(p);
                    gasProbMarker.colors.push_back(col_hit);

                    // CONFIDENCE
                    std_msgs::msg::ColorRGBA colorConfidence =
                        valueToColor(hitProbGrid.dataAt(a,b).confidence, 0, 1, Utils::valueColorMode::Linear);

                    p.z = settings.visualization.markers_height - 0.1;
                    confidenceMarker.points.push_back(p);
                    confidenceMarker.colors.push_back(colorConfidence);
                }
            }
        }

        pubs.markers.source_probability_markers->publish(sourceProbMarker);
        pubs.markers.hitProbabilityMarkers->publish(gasProbMarker);
        pubs.markers.confidenceMarkers->publish(confidenceMarker);
    }

    void PMFS::debugMapSegmentation()
    {
        MarkerArray segmentMarker;
        for (int i = 0; i < simulations.QTleaves.size(); i++)
        {
            NQA::Node* leaf = &simulations.QTleaves[i];
            Marker mark = Utils::emptyMarker({0.2, 0.2}, node->get_clock());
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

    void PMFS::plotWindVectors()
    {
        MarkerArray arrow_array;
        // Add an ARROW marker for each node
        Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = node->now();
        marker.ns = "WindVector";
        marker.type = Marker::ARROW;
        marker.action = Marker::ADD;

        // Get max wind vector in the map (to normalize the plot)
        double max_module = 0.0;
        for (size_t i = 0; i < estimatedWindVectors.size(); i++)
        {
            if (glm::length(estimatedWindVectors[i]) > max_module)
                max_module = glm::length(estimatedWindVectors[i]);
        }

        for (size_t i = 0; i < estimatedWindVectors.size(); i++)
        {
            if (simulationOccupancy[i] == Occupancy::Free)
            {
                double module = glm::length(estimatedWindVectors[i]);
                double angle = std::atan2(estimatedWindVectors[i].y, estimatedWindVectors[i].x);
                if (module > 0.001)
                {
                    marker.id = i;
                    // Set the pose of the marker.
                    Vector2Int indices2D = gridMetadata.indices2D(i);
                    Vector2 coords = gridMetadata.indexToCoordinates(indices2D.x, indices2D.y);
                    marker.pose.position.x = coords.x;
                    marker.pose.position.y = coords.y;
                    marker.pose.position.z = settings.visualization.markers_height;
                    marker.pose.orientation = Utils::createQuaternionMsgFromYaw(angle);
                    // shape
                    marker.scale.x = gridMetadata.cellSize * (module / max_module); // arrow length,
                    marker.scale.y = 0.03;                                      // arrow width
                    marker.scale.z = 0.05;                                      // arrow height
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