#include "GrGSLLib.hpp"
#include <gsl_server/algorithms/Common/Utils/Math.hpp>
#include <gsl_server/algorithms/Common/Utils/RosUtils.hpp>
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
        GridUtils::reduceOccupancyMap(algorithm.map.data, algorithm.map.info.width, grid.occupancy, grid.metadata);

        // remove "free" cells that are actually unreachable
        {
            HashSet openPropagationSet;
            HashSet activePropagationSet;
            HashSet closedPropagationSet;

            Vector2Int currentIndices = grid.metadata.coordinatesToIndex(algorithm.currentRobotPose.pose.pose);
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
            Vector2 coordR = grid.metadata.indexToCoordinates(robotPosition);
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

                    //the cells that are immediately around the robot (within 1 space) always get the same probability as the best direction
                    if (std::abs(row - robotPosition.y) <= 1 && std::abs(col - robotPosition.x) <= 1)
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
                        Vector2 coordC = grid.metadata.indexToCoordinates(colRow);

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

        while (!activePropagationSet.empty())
        {
            while (!activePropagationSet.empty())
            {
                Vector2Int activeCell = *activePropagationSet.begin();
                activePropagationSet.erase(activePropagationSet.begin());
                closedPropagationSet.insert(activeCell);

                int startR = std::max(0, activeCell.x - 1);
                int endR = std::min((int) grid.metadata.dimensions.y - 1, activeCell.x + 1);
                int startC = std::max(0, activeCell.y - 1);
                int endC = std::min((int) grid.metadata.dimensions.x - 1, activeCell.y + 1);

                // 8-neighbour propagation
                for (int col = startC; col <= endC; col++)
                    for (int row = startR; row <= endR; row++)
                        calculateWeight(grid, {col, row}, activeCell, openPropagationSet, closedPropagationSet, activePropagationSet);
            }

            activePropagationSet.clear();
            for (auto& par : openPropagationSet)
            {
                if (grid.freeAt(par))
                    activePropagationSet.insert(par);
                else
                    closedPropagationSet.insert(par);
            }

            openPropagationSet.clear();
        }
    }

    void GrGSLLib::calculateWeight(Grid2D<Cell> grid, Vector2Int newCell, Vector2Int activeCell, HashSet& openPropagationSet,
                                   HashSet& closedPropagationSet, HashSet& activePropagationSet)
    {
        if (closedPropagationSet.find(newCell) == closedPropagationSet.end() &&
                activePropagationSet.find(newCell) == activePropagationSet.end())
        {

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
    }

    void GrGSLLib::Normalize(Grid2D<GrGSL_internal::Cell> grid)
    {
        Utils::NormalizeDistribution<Cell>(grid.data,
                                           [](Cell& cell) -> double&
                                           {
                                               return cell.sourceProb;
                                           }
                                           , grid.occupancy);
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
                if (grid.freeAt(col,row))
                {
                    auto coords = grid.metadata.indexToCoordinates(col, row);
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

}