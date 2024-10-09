#include "GrGSLLib.hpp"
#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>
#include <gsl_server/algorithms/Common/Utils/Collections.hpp>
#include <angles/angles.h>

namespace GSL
{
    using namespace GrGSL_internal;
    void GrGSLLib::initMetadata(Grid2DMetadata& metadata, const OccupancyGrid& map, int scale)
    {
        metadata.cellSize = map.info.resolution * scale;
        metadata.origin.x = map.info.origin.position.x;
        metadata.origin.y = map.info.origin.position.y;
        metadata.numFreeCells = 0;
        metadata.dimensions.x = floor((float)map.info.width / scale);
        metadata.dimensions.y = floor((float)map.info.height / scale);
        metadata.scale = scale;
    }

    void GrGSLLib::initializeMap(Algorithm& algorithm, Grid2D<Cell> grid)
    {        
        // remove "free" cells that are actually unreachable
        {
            HashSet openPropagationSet;
            HashSet activePropagationSet;
            HashSet closedPropagationSet;

            Vector2Int currentIndices = grid.metadata.coordinatesToIndices(algorithm.currentRobotPose.pose.pose);
            grid.dataAt(currentIndices).auxWeight = 1;
            activePropagationSet.insert(currentIndices);
            propagateProbabilities(grid, openPropagationSet, closedPropagationSet, activePropagationSet);
            for (int i = 0; i < grid.data.size(); i++)
            {
                if (grid.data[i].auxWeight == -1)
                {
                    grid.data[i].sourceProb = 0;
                    grid.metadata.numFreeCells--;
                    grid.occupancy[i] = Occupancy::Obstacle;
                }
                else
                    grid.data[i].sourceProb = 1;
            }
        }
        Normalize(grid);
    }

    void GrGSLLib::GetSettings(rclcpp::Node::SharedPtr node, GrGSL_internal::Settings& settings, GrGSL_internal::Markers& markers)
    {
        settings.useDiffusionTerm = Utils::getParam<bool>(node, "useDiffusionTerm", false); //experimental way to extract useful info from low-wind measurements, not very reliable
        settings.stdevHit = Utils::getParam<double>(node, "stdevHit", 1.0);
        settings.stdevMiss = Utils::getParam<double>(node, "stdevMiss", 2.0);
        settings.infoTaxis = Utils::getParam<bool>(node, "infoTaxis", false);
        settings.allowMovementRepetition = Utils::getParam<bool>(node, "allowMovementRepetition", true);
        settings.convergence_thr = Utils::getParam<double>(node, "convergence_thr", 0.5); // threshold for source declaration
        markers.markersHeight = Utils::getParam<float>(node, "markers_height", 0);
    }

    void GrGSLLib::estimateProbabilitiesfromGasAndWind(Grid2D<Cell> grid, const GrGSL_internal::Settings& settings,
            bool hit, bool advection, double windDirection, Vector2 positionOfLastHit, Vector2Int robotPosition)
    {
        if (!advection && !(settings.useDiffusionTerm && hit))
            return;

        // ADVECTION
        // this part is always done, to calculate the distance field to the current robot position
        // the actual advection-based source probabilities are discarded if the wind speed is too low for the direction to be reliable
        HashSet openPropagationSet;
        HashSet activePropagationSet;
        HashSet closedPropagationSet;

        // estimate the probabilities for the neighbour cells
        //-------------------------------
        {
            Vector2 coordR = grid.metadata.indicesToCoordinates(robotPosition);
            double idealPropagationDirection = hit ?
                                               angles::normalize_angle(windDirection + M_PI) //upwind direction
                                               : std::atan2((positionOfLastHit.y - coordR.y), (positionOfLastHit.x - coordR.x)) + M_PI; //direction that we have moved since the last hit

            //loop limits
            size_t startC = std::max(0, robotPosition.x - 2);
            size_t endC = std::min(grid.metadata.dimensions.x - 1, robotPosition.x + 2);
            size_t startR = std::max(0, robotPosition.y - 2);
            size_t endR = std::min(grid.metadata.dimensions.y - 1, robotPosition.y + 2);

            for (int row = startR; row <= endR; row++)
            {
                for (int col = startC; col <= endC; col++)
                {
                    Vector2Int colRow {col, row};
                    if (!grid.freeAt(colRow))
                        continue;

                    //the cells that the robot is in always gets the same probability as the best direction
                    if (colRow == robotPosition)
                    {
                        grid.dataAt(colRow).auxWeight = Utils::evaluate1DGaussian(
                                                            (hit ? 0 : M_PI),
                                                            (hit ? settings.stdevHit : settings.stdevMiss));
                        grid.dataAt(colRow).distance = 0;
                        closedPropagationSet.insert(colRow);
                    }
                    //for the rest of the cells, we compare the direction of the connecting vector with the ideal direction
                    else
                    {
                        Vector2 coordC = grid.metadata.indicesToCoordinates(colRow);

                        double angleCellToRobot = atan2((coordR.y - coordC.y), (coordR.x - coordC.x));
                        double angleDifference = atan2(sin(idealPropagationDirection - angleCellToRobot), cos(idealPropagationDirection - angleCellToRobot));

                        grid.dataAt(colRow).auxWeight = Utils::evaluate1DGaussian(angleDifference,
                                                        hit ? settings.stdevHit : settings.stdevMiss);
                        grid.dataAt(colRow).distance = (row == robotPosition.y || col == robotPosition.x)
                                                       ? 1
                                                       : sqrt(2);

                        activePropagationSet.insert(colRow);
                    }
                }
            }
        }

        // propagate these short-range estimations to the entire environment using the navigation map
        // also calculate the distance field
        propagateProbabilities(grid, openPropagationSet, closedPropagationSet, activePropagationSet);

        // if we are going to use these probabilities, normalize them. Otherwise, to the trash with them
        if (advection)
        {
            double sum = 0;
            mapFunctionToCells(grid, [&sum](Cell & cell, size_t index)
                               {
                                   sum += cell.auxWeight;
                               }
                              );
            mapFunctionToCells(grid, [&sum](Cell & cell, size_t index)
                               {
                                   cell.auxWeight /= sum;
                               }
                               , MapFunctionMode::Parallel);
        }
        else
        {
            mapFunctionToCells(grid, [](Cell & cell, size_t index)
                               {
                                   cell.auxWeight = 0;
                               }, MapFunctionMode::Parallel);
        }

        // DIFFUSION
        if (settings.useDiffusionTerm && hit)
        {
            // calculate and normalize these probabilities before combining them with the advection ones
            std::vector<double> diffusionProb = std::vector<double>(grid.data.size(), 0);
            double sum = 0;
            mapFunctionToCells(grid, [&sum, &diffusionProb](Cell & cell, size_t index)
                               {
                                   diffusionProb[index] = std::max(0.1, Utils::evaluate1DGaussian(cell.distance, 3));
                                   sum += diffusionProb[index];
                               });

            if (sum > 0)
            {
                mapFunctionToCells(grid, [&sum, &diffusionProb](Cell & cell, size_t index)
                                   {
                                       diffusionProb[index] /= sum;
                                       cell.auxWeight += diffusionProb[index];
                                   }, MapFunctionMode::Parallel);
            }
        }

        // BAYESIAN FILTER
        mapFunctionToCells(grid, [](Cell & cell, size_t index)
                           {
                               cell.sourceProb *= cell.auxWeight;
                               cell.auxWeight = 0;
                           }, MapFunctionMode::Parallel);

        Normalize(grid);
    }

    void GrGSLLib::propagateProbabilities(Grid2D<Cell> grid, HashSet& openPropagationSet, HashSet& closedPropagationSet,
                                          HashSet& activePropagationSet)
    {
#define DebugPropagation 0 // Run the propagation step by step to see how the sets change
#if DebugPropagation

        std::vector<ColorRGBA> colors (grid.data.size());
#endif
        while (!activePropagationSet.empty())
        {
            while (!activePropagationSet.empty())
            {
                Vector2Int activeCell = *activePropagationSet.begin();
                activePropagationSet.erase(activePropagationSet.begin());
                closedPropagationSet.insert(activeCell);

                int startR = std::max(0, activeCell.y - 1);
                int endR = std::min((int) grid.metadata.dimensions.y - 1, activeCell.y + 1);
                int startC = std::max(0, activeCell.x - 1);
                int endC = std::min((int) grid.metadata.dimensions.x - 1, activeCell.x + 1);

                // 8-neighbour propagation
                for (int col = startC; col <= endC; col++)
                    for (int row = startR; row <= endR; row++)
                        if (grid.freeAt(col, row))
                            calculateWeight(grid, {col, row}, activeCell, openPropagationSet, closedPropagationSet, activePropagationSet);
            }
#if DebugPropagation
            mapFunctionToCells(grid, [&](Cell & cell, size_t index)
                            {
                                if(Utils::contains(openPropagationSet, grid.metadata.indices2D(index)))
                                    colors[index] = Utils::create_color(0, 1, 0, 1);
                                else if(Utils::contains(closedPropagationSet, grid.metadata.indices2D(index)))
                                    colors[index] = Utils::create_color(1, 0, 0, 1);
                                else
                                    colors[index] = Utils::create_color(0, 0, 1, 1);

                            }, MapFunctionMode::Parallel);
            Utils::publishDebugMarkers(Grid2D<ColorRGBA>(colors, grid.occupancy, grid.metadata));
            std::cin.get(); // IMPORTANT!! you need to run the node in a xterm window! otherwise cin is blocked by ros and we cannot pause and step
#endif

            activePropagationSet.clear();
            for (auto& par : openPropagationSet)
                activePropagationSet.insert(par);

            openPropagationSet.clear();
        }
    }

    void GrGSLLib::calculateWeight(Grid2D<Cell> grid, Vector2Int newCell, Vector2Int activeCell, HashSet& openPropagationSet,
                                   HashSet& closedPropagationSet, HashSet& activePropagationSet)
    {
        if (closedPropagationSet.find(newCell) != closedPropagationSet.end() ||
                activePropagationSet.find(newCell) != activePropagationSet.end())
            return;

        // if there already was a path to this cell
        if (openPropagationSet.find(newCell) != openPropagationSet.end())
        {
            double d = grid.dataAt(activeCell).distance +
                       ((newCell.x == activeCell.x || newCell.y == activeCell.y) ?
                        1 : sqrt(2)); // distance of this new path to the same cell

            // if the distance is the same, keep the best probability!
            if (Utils::approx(d, grid.dataAt(newCell).distance))
                grid.dataAt(newCell).auxWeight = std::max(grid.dataAt(activeCell).auxWeight, grid.dataAt(newCell).auxWeight);
            else if (d < grid.dataAt(newCell).distance)
            {
                // keep the shortest path
                grid.dataAt(newCell).auxWeight = grid.dataAt(activeCell).auxWeight;
                grid.dataAt(newCell).distance = d;
            }
        }
        else
        {
            grid.dataAt(newCell).auxWeight = grid.dataAt(activeCell).auxWeight;
            grid.dataAt(newCell).distance = grid.dataAt(activeCell).distance +
                                            ((newCell.x == activeCell.x || newCell.y == activeCell.y) ?
                                             1 : sqrt(2));
            openPropagationSet.insert(newCell);
        }
    }

    void GrGSLLib::Normalize(Grid2D<GrGSL_internal::Cell> grid)
    {
        Utils::NormalizeDistribution<Cell>(grid.data,
                                           [](Cell & cell) -> double&
                                           {
                                               return cell.sourceProb;
                                           }
                                           , grid.occupancy);
    }


    double GrGSLLib::informationGain(const WindVector& windVec, Grid2D<Cell> grid, const Settings& settings, Vector2 positionOfLastHit)
    {
        auto predictionCells = grid.data; // temp copy of the matrix of cells that we can modify to simulate the effect of a measurement
        auto accessProb = [](const Cell & cell)
                          {
                              return cell.sourceProb;
                          };

        //simulate a hit in the considered position and see how much info that gives us
        GrGSLLib::estimateProbabilitiesfromGasAndWind(
            Grid2D<Cell>(predictionCells, grid.occupancy, grid.metadata),
            settings,
            true,
            true,
            windVec.angle,
            positionOfLastHit,
            Vector2Int(windVec.col, windVec.row));
        double infoHit = Utils::KLD<Cell>(predictionCells, grid.data, grid.occupancy, accessProb);

        //simulate a miss and repeat
        predictionCells = grid.data;
        GrGSLLib::estimateProbabilitiesfromGasAndWind(
            Grid2D<Cell>(predictionCells, grid.occupancy, grid.metadata),
            settings,
            false,
            true,
            windVec.angle,
            positionOfLastHit,
            Vector2Int(windVec.col, windVec.row));
        double infoMiss = Utils::KLD<Cell>(predictionCells, grid.data, grid.occupancy, accessProb);

        //expected value of info, considering that the probability of a hit can be equated to the currently estimated source prob... which is a hack
        size_t index = grid.metadata.indexOf({windVec.col, windVec.row});
        return grid.data[index].sourceProb * infoHit + (1 - grid.data[index].sourceProb) * infoMiss;
    }

    void GrGSLLib::mapFunctionToCells(Grid2D<Cell> grid, std::function<void(Cell&, size_t)> function, MapFunctionMode mode)
    {
        if (mode == MapFunctionMode::Parallel)
        {
            #pragma omp parallel for
            for (size_t index = 0; index < grid.data.size(); index++)
                if (grid.occupancy[index] == Occupancy::Free)
                    function(grid.data[index], index);
        }
        else
        {
            for (size_t index = 0; index < grid.data.size(); index++)
                if (grid.occupancy[index] == Occupancy::Free)
                    function(grid.data[index], index);
        }
    }


    void GrGSLLib::VisualizeMarkers(Grid2D<Cell> grid, GrGSL_internal::Markers& markers, rclcpp::Node::SharedPtr node)
    {
        constexpr auto emptyMarker = []()
                                     {
                                         Marker points;
                                         points.header.frame_id = "map";
                                         points.ns = "cells";
                                         points.id = 0;
                                         points.type = Marker::POINTS;
                                         points.action = Marker::ADD;

                                         points.color.r = 1.0;
                                         points.color.g = 0.0;
                                         points.color.b = 1.0;
                                         points.color.a = 1.0;
                                         points.scale.x = 0.15;
                                         points.scale.y = 0.15;
                                         return points;
                                     };

        Marker points = emptyMarker();
        points.header.stamp = node->now();

        for (int row = 0; row < grid.metadata.dimensions.y; row++)
        {
            for (int col = 0; col < grid.metadata.dimensions.x; col++)
            {
                if (grid.freeAt(col, row))
                {
                    auto coords = grid.metadata.indicesToCoordinates(col, row);
                    Point p;
                    p.x = coords.x;
                    p.y = coords.y;
                    p.z = 0;

                    std_msgs::msg::ColorRGBA color =
                        Utils::valueToColor(grid.dataAt(col, row).sourceProb, 0.00001, 0.1, Utils::valueColorMode::Logarithmic);

                    points.points.push_back(p);
                    points.colors.push_back(color);
                }
            }
        }
        markers.probabilityMarkers->publish(points);
    }

    void GrGSLLib::VisualizeMarkers(Grid2D<double> grid, GrGSL_internal::Markers& markers, rclcpp::Node::SharedPtr node)
    {
        constexpr auto emptyMarker = []()
                                     {
                                         Marker points;
                                         points.header.frame_id = "map";
                                         points.ns = "cells";
                                         points.id = 0;
                                         points.type = Marker::POINTS;
                                         points.action = Marker::ADD;

                                         points.color.r = 1.0;
                                         points.color.g = 0.0;
                                         points.color.b = 1.0;
                                         points.color.a = 1.0;
                                         points.scale.x = 0.15;
                                         points.scale.y = 0.15;
                                         return points;
                                     };

        Marker points = emptyMarker();
        points.header.stamp = node->now();

        for (int row = 0; row < grid.metadata.dimensions.y; row++)
        {
            for (int col = 0; col < grid.metadata.dimensions.x; col++)
            {
                if (grid.freeAt(col, row))
                {
                    auto coords = grid.metadata.indicesToCoordinates(col, row);
                    Point p;
                    p.x = coords.x;
                    p.y = coords.y;
                    p.z = 0;

                    std_msgs::msg::ColorRGBA color =
                        Utils::valueToColor(grid.dataAt(col, row), 0.00001, 0.1, Utils::valueColorMode::Logarithmic);

                    points.points.push_back(p);
                    points.colors.push_back(color);
                }
            }
        }
        markers.probabilityMarkers->publish(points);
    }


    Vector2 GrGSLLib::expectedValueSource(Grid2D<Cell> grid, double proportionBest)
    {
        struct CellData
        {
            Vector2Int indices;
            double probability;
            CellData(Vector2Int ind, double prob)
            {
                indices = ind;
                probability = prob;
            }
        };
        std::vector<CellData> data;
        for (int i = 0; i < grid.data.size(); i++)
        {
            if (grid.occupancy[i] == Occupancy::Free)
            {
                CellData cd(grid.metadata.indices2D(i), grid.data[i].sourceProb);
                data.push_back(cd);
            }
        }

        std::sort(data.begin(), data.end(), [](const CellData & a, const CellData & b)
                  {
                      return a.probability > b.probability;
                  });

        double averageX = 0, averageY = 0;
        double sum = 0;

        for (int i = 0; i < data.size() * proportionBest; i++)
        {
            CellData& cd = data[i];
            Vector2 coord = grid.metadata.indicesToCoordinates(cd.indices.x, cd.indices.y);
            averageX += cd.probability * coord.x;
            averageY += cd.probability * coord.y;
            sum += cd.probability;
        }
        return Vector2(averageX / sum, averageY / sum);
    }

    double GrGSLLib::varianceSourcePosition(Grid2D<Cell> grid)
    {
        Vector2 expected = expectedValueSource(grid, 1.);
        double x = 0, y = 0;
        for (int i = 0; i < grid.data.size(); i++)
        {
            if (grid.occupancy[i] == Occupancy::Free)
            {
                Vector2Int indices = grid.metadata.indices2D(i);
                Vector2 coords = grid.metadata.indicesToCoordinates(indices);
                double p = grid.dataAt(indices).sourceProb;
                x += std::pow(coords.x - expected.x, 2) * p;
                y += std::pow(coords.y - expected.y, 2) * p;
            }
        }
        return x + y;
    }

}