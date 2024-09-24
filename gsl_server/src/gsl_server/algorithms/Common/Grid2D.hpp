#pragma once
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <gsl_server/core/Vectors.hpp>
#include <vector>
#include "Occupancy.hpp"

namespace GSL
{
    //See the grid class
    struct Grid2DMetadata
    {
        Vector2 origin;
        float cellSize; // in meters
        Vector2Int dimensions;
        uint16_t scale; // with respect to the original occupancy map. Scale=5 means each cell in the grid is a 5x5 square in the ROS map
        size_t numFreeCells;

        Vector2Int coordinatesToIndex(double x, double y) const
        {
            return Vector2Int((x - origin.x) / (cellSize), (y - origin.y) / (cellSize));
        }

        Vector2Int coordinatesToIndex(const Vector2& v) const
        {
            return coordinatesToIndex(v.x, v.y);
        }

        Vector2Int coordinatesToIndex(const geometry_msgs::msg::Pose& pose) const
        {
            return coordinatesToIndex(pose.position.x, pose.position.y);
        }

        Vector2 indexToCoordinates(int col, int row, bool centerOfCell = true) const
        {
            float offset = centerOfCell ? 0.5 : 0;
            return Vector2(origin.x + (col + offset) * cellSize, origin.y + (row + offset) * cellSize);
        }

        Vector2 indexToCoordinates(const Vector2Int& indices, bool centerOfCell = true) const
        {
            return indexToCoordinates(indices.x, indices.y, centerOfCell);
        }

        size_t indexOf(const Vector2Int& v) const
        {
            return v.x + v.y * dimensions.x;
        }

        Vector2Int indices2D(size_t index) const
        {
            return Vector2Int(index % dimensions.x, index / dimensions.x);
        }

        bool indicesInBounds(const Vector2Int& indices) const
        {
            return indices.x >= 0 && indices.x < dimensions.x && indices.y >= 0 && indices.y < dimensions.y;
        }
    };

    //A grid represents a 2D map with occupancy and some arbitraty per-cell data. The GridMetadata field allows it to convert 1D to 2D indices and vice-versa
    template <typename T>
    struct Grid2D
    {
        std::vector<T>& data;
        std::vector<Occupancy>& occupancy;
        Grid2DMetadata& metadata;

        Grid2D(std::vector<T>& _data, std::vector<Occupancy>& _occupancy, Grid2DMetadata& _metadata)
            : data(_data), occupancy(_occupancy), metadata(_metadata)
        {}

        T& dataAt(size_t col, size_t row) const
        {
            return data[metadata.indexOf({col, row})];
        }

        Occupancy& occupancyAt(size_t col, size_t row) const
        {
            return occupancy[metadata.indexOf({col, row})];
        }

        bool freeAt(size_t col, size_t row) const
        {
            return occupancyAt(col, row) == Occupancy::Free;
        }

        T& dataAt(const Vector2Int& indices) const
        {
            return data[metadata.indexOf(indices)];
        }

        Occupancy& occupancyAt(const Vector2Int& indices) const
        {
            return occupancy[metadata.indexOf(indices)];
        }

        bool freeAt(const Vector2Int& indices) const
        {
            return occupancyAt(indices) == Occupancy::Free;
        }
    };

    class GridUtils
    {
    public:
        GridUtils() = delete;

        //reduce the resolution of an occupancy grid, considering that a cell in the smaller map is occupied as soon as a single smaller cell in it is
        static void reduceOccupancyMap(const std::vector<int8_t>& map, size_t mapWidth, std::vector<Occupancy>& occupancy,
                                       const Grid2DMetadata& metadata)
        {
            int scale = metadata.scale; // scale for dynamic map reduction
            for (int j = 0; j < metadata.dimensions.y; j++)
            {
                for (int i = 0; i < metadata.dimensions.x; i++)
                {
                    bool squareIsFree = true;

                    for (int row = j * scale; row < (j + 1) * scale; row++)
                    {
                        for (int col = i * scale; col < (i + 1) * scale; col++)
                        {
                            if (map[col + row * mapWidth] != 0)
                                squareIsFree = false;
                        }
                    }
                    if (squareIsFree)
                        occupancy[metadata.indexOf({i, j})] = Occupancy::Free;
                    occupancy[metadata.indexOf({i, j})] = squareIsFree ? Occupancy::Free : Occupancy::Obstacle;
                }
            }
        }
    };

} // namespace GSL