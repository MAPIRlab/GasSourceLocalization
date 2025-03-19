#pragma once
#include <cstddef>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <gsl_server/core/Vectors.hpp>
#include <vector>
#include "Occupancy.hpp"

namespace GSL
{
    //See the grid class
    struct Grid3DMetadata
    {
        Vector3 origin;
        float cellSize; // in meters
        Vector3Int dimensions;
        uint16_t scale; // with respect to the original occupancy map. Scale=5 means each cell in the grid is a 5x5 square in the ROS map
        size_t numFreeCells;

        Vector3Int coordinatesToIndices(double x, double y, double z) const
        {
            return Vector3Int((x - origin.x) / (cellSize), (y - origin.y) / (cellSize), (z - origin.z) / cellSize);
        }

        Vector3Int coordinatesToIndices(const Vector3& v) const
        {
            return coordinatesToIndices(v.x, v.y, v.z);
        }

        Vector3Int coordinatesToIndices(const geometry_msgs::msg::Pose& pose) const
        {
            return coordinatesToIndices(pose.position.x, pose.position.y, pose.position.z);
        }

        Vector3 indicesToCoordinates(int i, int j, int h, bool centerOfCell = true) const
        {
            float offset = centerOfCell ? 0.5 : 0;
            return Vector3(origin.x + (i + offset) * cellSize, origin.y + (j + offset) * cellSize, origin.z + (h + offset) * cellSize);
        }

        Vector3 indicesToCoordinates(const Vector3Int& indices, bool centerOfCell = true) const
        {
            return indicesToCoordinates(indices.x, indices.y, indices.z, centerOfCell);
        }

        Vector3 indicesToCoordinates(size_t index, bool centerOfCell = true) const
        {
            return indicesToCoordinates(indices3D(index), centerOfCell);
        }

        size_t indexOf(size_t x, size_t y, size_t z) const
        {
            return x + y * dimensions.x + z * dimensions.x * dimensions.y;
        }

        size_t indexOf(const Vector3Int& v) const
        {
            return indexOf(v.x, v.y, v.z);
        }

        Vector3Int indices3D(size_t index) const
        {
            size_t z = index / (dimensions.x * dimensions.y);
            size_t remainder = index % (dimensions.x * dimensions.y);
            return Vector3Int(remainder % dimensions.x, remainder / dimensions.x, z);
        }

        Vector2Int indices2D(size_t index) const
        {
            size_t remainder = index % (dimensions.x * dimensions.y);
            return Vector2Int(remainder % dimensions.x, remainder / dimensions.x);
        }

        bool indicesInBounds(const Vector3Int& indices) const
        {
            return indices.x >= 0 && indices.x < dimensions.x
                   && indices.y >= 0 && indices.y < dimensions.y
                   && indices.z >= 0 && indices.z < dimensions.z;
        }
    };

    //A grid represents a 2D map with occupancy and some arbitraty per-cell data. The GridMetadata field allows it to convert 1D to 2D indices and vice-versa
    template <typename T>
    struct Grid3D
    {
        std::vector<T>& data;
        std::vector<Occupancy>& occupancy;
        Grid3DMetadata& metadata;

        Grid3D(std::vector<T>& _data, std::vector<Occupancy>& _occupancy, Grid3DMetadata& _metadata)
            : data(_data), occupancy(_occupancy), metadata(_metadata)
        {}

        T& dataAt(size_t i, size_t j, size_t h) const
        {
            return data[metadata.indexOf(i, j, h)];
        }

        Occupancy& occupancyAt(size_t i, size_t j, size_t h) const
        {
            return occupancy[metadata.indexOf(i, j, h)];
        }

        bool freeAt(size_t i, size_t j, size_t h) const
        {
            return occupancyAt(i, j, h) == Occupancy::Free;
        }
    };

} // namespace GSL