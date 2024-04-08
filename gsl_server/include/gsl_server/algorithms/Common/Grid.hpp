#pragma once
#include <vector>
#include <gsl_server/core/Vectors.hpp>

namespace GSL
{
    struct GridMetadata
    {
        Vector2 origin;
        float cellSize;
        int numFreeCells;
        int width, height;
        Vector2Int coordinatesToIndex(double x, double y) const
        {
            return Vector2Int((y - origin.y) / (cellSize), (x - origin.x) / (cellSize));
        }

        Vector2 indexToCoordinates(int i, int j, bool centerOfCell = true) const
        {
            float offset = centerOfCell ? 0.5 : 0;
            return Vector2(origin.x + (j + offset) * cellSize, origin.y + (i + offset) * cellSize);
        }

        size_t indexOf(size_t i, size_t j)
        {
            return j + i * width;
        }
    };

    enum class Occupancy {Free, Obstacle};
    template <typename T>
    struct Grid
    {
        std::vector<T>& data;
        std::vector<Occupancy>& occupancy;
        GridMetadata& metadata;

        Grid(std::vector<T>& _data, std::vector<Occupancy>& _occupancy, GridMetadata& _metadata)
            : data(_data), occupancy(_occupancy), metadata(_metadata)
        {}

        T& dataAt(size_t i, size_t j)
        {
            return data[metadata.indexOf(i,j)];
        }

        Occupancy& occupancyAt(size_t i, size_t j)
        {
            return occupancy[metadata.indexOf(i,j)];
        }

        bool freeAt(size_t i, size_t j)
        {
            return occupancyAt(i,j) == Occupancy::Free;
        }
    };

} // namespace GSL