#pragma once
#include "Grid2D.hpp"

namespace GSL
{
    // a smaller piece of an existing grid
    template <typename T>
    class SubGrid2D
    {
    public:
        SubGrid2D(Grid2D<T> grid, Vector2 origin, Vector2Int dimensions);

        const Vector2 origin;
        const Vector2Int dimensions;

        // new methods
        // ----------------------------
        size_t indexOriginal(size_t col, size_t row)
        {
            size_t x = col + originIndices.x;
            size_t y = row + originIndices.y;
            return grid.metadata.indexOf(x, y);
        }

        size_t indexOriginal(Vector2Int indices)
        {
            return indexOriginal(indices.x, indices.y);
        }

        // metadata substitutions
        // ----------------------------
        Vector2Int coordsToIndices(const Vector2& coords)
        {
            return Vector2Int((coords.x - origin.x) / (grid.metadata.cellSize), (coords.y - origin.y) / (grid.metadata.cellSize));
        }

        float cellSize()
        {
            return grid.metadata.cellSize;
        }

        // Grid2D methods
        // ----------------------------
        T& dataAt(size_t col, size_t row) const
        {
            return grid.data.at(indexOriginal({col, row}));
        }

        Occupancy& occupancyAt(size_t col, size_t row) const
        {
            return grid.occupancy.at(indexOriginal(col, row));
        }

        bool occupancyAt(size_t col, size_t row) const
        {
            return occupancyAt(col, row) == Occupancy::Free;
        }

        T& dataAt(const Vector2Int& indices) const
        {
            return grid.data.at(indexOriginal(indices));
        }

        Occupancy& occupancyAt(const Vector2Int& indices) const
        {
            return grid.occupancy.at(indexOriginal(indices));
        }

        bool occupancyAt(const Vector2Int& indices) const
        {
            return occupancyAt(indices) == Occupancy::Free;
        }

    private:
        Vector2Int originIndices;
        Grid2D<T> grid; // not public because accessing the original grid directly makes it way too easy to mess things up with the indices
    };

    // implementation
    //----------------
    template <typename T>
    SubGrid2D<T>::SubGrid2D(Grid2D<T> grid, Vector2 origin, Vector2Int dimensions)
        : grid(grid), origin(origin), dimensions(dimensions)
    {
        originIndices = grid.metadata.coordinatesToIndices(origin);
    }

} // namespace GSL