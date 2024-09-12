#pragma once
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <gsl_server/core/Vectors.hpp>
#include <vector>

namespace GSL
{
    //See the grid class
    struct GridMetadata
    {
        Vector2 origin;
        float cellSize; // in meters
        size_t width, height;
        uint16_t scale; // with respect to the original occupancy map. Scale=5 means each cell in the grid is a 5x5 square in the ROS map
        size_t numFreeCells;

        Vector2Int coordinatesToIndex(double x, double y) const
        {
            return Vector2Int((y - origin.y) / (cellSize), (x - origin.x) / (cellSize));
        }

        Vector2Int coordinatesToIndex(const Vector2& v) const
        {
            return coordinatesToIndex(v.x, v.y);
        }

        Vector2Int coordinatesToIndex(const geometry_msgs::msg::Pose& pose) const
        {
            return coordinatesToIndex(pose.position.x, pose.position.y);
        }

        Vector3Int coordinatesToIndex(double x, double y, double z) const
        {
            return Vector3Int((y - origin.y) / (cellSize), (x - origin.x) / (cellSize), (z - 0) / (cellSize)); //TODO assuming map is at z=0 for anything that requires height
        }

        Vector2 indexToCoordinates(int i, int j, bool centerOfCell = true) const
        {
            float offset = centerOfCell ? 0.5 : 0;
            return Vector2(origin.x + (j + offset) * cellSize, origin.y + (i + offset) * cellSize);
        }

        Vector2 indexToCoordinates(const Vector2Int& indices, bool centerOfCell = true) const
        {
            return indexToCoordinates(indices.x, indices.y, centerOfCell);
        }

        size_t indexOf(const Vector2Int& v) const
        {
            return v.y + v.x * width;
        }

        Vector2Int indices2D(size_t index) const
        {
            return Vector2Int(index / width, index % width);
        }

        bool indicesInBounds(const Vector2Int& indices) const
        {
            return indices.x >= 0 && indices.x < height && indices.y >= 0 && indices.y < width;
        }
    };

    enum class Occupancy
    {
        Free,
        Obstacle
    };

    //A grid represents a 2D map with occupancy and some arbitraty per-cell data. The GridMetadata field allows it to convert 1D to 2D indices and vice-versa
    template <typename T>
    struct Grid
    {
        std::vector<T>& data;
        std::vector<Occupancy>& occupancy;
        GridMetadata& metadata;

        Grid(std::vector<T>& _data, std::vector<Occupancy>& _occupancy, GridMetadata& _metadata)
            : data(_data), occupancy(_occupancy), metadata(_metadata)
        {}

        T& dataAt(size_t i, size_t j) const
        {
            return data[metadata.indexOf({i, j})];
        }

        Occupancy& occupancyAt(size_t i, size_t j) const
        {
            return occupancy[metadata.indexOf({i, j})];
        }

        bool freeAt(size_t i, size_t j) const
        {
            return occupancyAt(i, j) == Occupancy::Free;
        }
    };

    class GridUtils
    {
    public:
        GridUtils() = delete;

        //reduce the resolution of an occupancy grid, considering that a cell in the smaller map is occupied as soon as a single smaller cell in it is
        static void reduceOccupancyMap(const std::vector<int8_t>& map, size_t mapWidth, std::vector<Occupancy>& occupancy,
                                       const GridMetadata& metadata)
        {
            int scale = metadata.scale; // scale for dynamic map reduction
            for (int i = 0; i < metadata.height; i++)
            {
                for (int j = 0; j < metadata.width; j++)
                {
                    bool squareIsFree = true;

                    for (int row = i * scale; row < (i + 1) * scale; row++)
                    {
                        for (int col = j * scale; col < (j + 1) * scale; col++)
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