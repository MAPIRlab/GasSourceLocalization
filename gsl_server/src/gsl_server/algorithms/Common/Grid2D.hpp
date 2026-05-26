#pragma once
#include "DDA/2D/RayCast.h"
#include "Occupancy.hpp"
#include "gsl_server/algorithms/Semantics/Semantics/Common/AABB.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <gsl_server/core/Macros.hpp>
#include <gsl_server/core/Vectors.hpp>
#include <vector>

namespace GSL
{
    // See the grid class
    struct Grid2DMetadata
    {
        Vector2 origin;
        float cellSize; // in meters
        Vector2Int dimensions;
        uint16_t scale = 1; // with respect to the original occupancy map. Scale=5 means each cell in the grid is a 5x5 square in the ROS map
        size_t numFreeCells;

        Vector2Int coordinatesToIndices(float x, float y) const
        {
            return Vector2Int((x - origin.x) / (cellSize), (y - origin.y) / (cellSize));
        }

        Vector2Int coordinatesToIndices(const Vector2& v) const
        {
            return coordinatesToIndices(v.x, v.y);
        }

        Vector2Int coordinatesToIndices(const geometry_msgs::msg::Pose& pose) const
        {
            return coordinatesToIndices(pose.position.x, pose.position.y);
        }

        Vector2 indicesToCoordinates(int col, int row, bool centerOfCell = true) const
        {
            float offset = centerOfCell ? 0.5 : 0;
            return Vector2(origin.x + (col + offset) * cellSize, origin.y + (row + offset) * cellSize);
        }

        Vector2 indicesToCoordinates(const Vector2Int& indices, bool centerOfCell = true) const
        {
            return indicesToCoordinates(indices.x, indices.y, centerOfCell);
        }

        Vector2 indexToCoordinates(size_t index, bool centerOfCell = true) const
        {
            return indicesToCoordinates(indices2D(index), centerOfCell);
        }

        size_t indexOf(size_t x, size_t y) const
        {
            return x + y * dimensions.x;
        }

        size_t indexOf(const Vector2Int& v) const
        {
            return indexOf(v.x, v.y);
        }

        Vector2Int indices2D(size_t index) const
        {
            return Vector2Int(index % dimensions.x, index / dimensions.x);
        }

        bool indicesInBounds(const Vector2Int& indices) const
        {
            return indices.x >= 0 && indices.x < dimensions.x && indices.y >= 0 && indices.y < dimensions.y;
        }

        AABB2D GetAABB() const
        {
            return AABB2D{.min = origin,
                          .max = origin + (Vector2)dimensions * cellSize};
        }

        AABB2D indicesToCoordinates(const AABB2DInt& aabb) const
        {
            return AABB2D{.min = indicesToCoordinates(aabb.min),
                          .max = indicesToCoordinates(aabb.max)};
        }
    };

    // A grid represents a 2D map with occupancy and some arbitraty per-cell data. The GridMetadata field allows it to convert 1D to 2D indices and vice-versa
    // If you want to represent an occupancy map without additional data, you can use a Grid2D<Occupancy, false> and have both .occupancy and .data point to the same vector
    // Important: By default, Grid2D is a non-owning struct (contains only references) to make accessing data easier. The second (optional) template parameter changes this behavior
    // To pass an owning grid into a function that expects a default (non-owning) one, you can use the AsNonOwning() conversion method
    template <typename T, bool Owning = false>
    struct Grid2D
    {
        using DataT = std::conditional<Owning, std::vector<T>, std::vector<T>&>::type;
        using OccupancyT = std::conditional<Owning, std::vector<Occupancy>, std::vector<Occupancy>&>::type;
        using MetadataT = std::conditional<Owning, Grid2DMetadata, Grid2DMetadata&>::type;

        DataT data;
        OccupancyT occupancy;
        MetadataT metadata;

        Grid2D(std::vector<T>& _data, std::vector<Occupancy>& _occupancy, Grid2DMetadata& _metadata)
            : data(_data), occupancy(_occupancy), metadata(_metadata)
        {
            static_assert(!(Owning && std::is_same<T, Occupancy>::value),
                          "Don't use Grid2D<Occupancy, true>! This will create two copies of the occupancy array. Use Map2D instead.");
            GSL_ASSERT(data.size() == occupancy.size() && data.size() == metadata.dimensions.x * metadata.dimensions.y);
        }

        // Important! If this is an owning grid, it will copy *both* the the data and the occupancy/metadata
        template <typename OtherT, bool OtherOwning>
        Grid2D(std::vector<T>& data, const Grid2D<OtherT, OtherOwning>& other)
            : data(data), occupancy(other.occupancy), metadata(other.metadata)
        {
            static_assert(!(Owning && std::is_same<T, Occupancy>::value),
                          "Don't use Grid2D<Occupancy, true>! This will create two copies of the occupancy array. Use Map2D instead.");
            GSL_ASSERT(data.size() == occupancy.size() && data.size() == metadata.dimensions.x * metadata.dimensions.y);
        }

        // marked as explicit because this can convert between owning and non-owning and that's a bit dangerous
        template <bool OtherOwning>
        explicit Grid2D(const Grid2D<T, OtherOwning>& other)
            : data(other.data), occupancy(other.occupancy), metadata(other.metadata)
        {}

        T& dataAt(size_t col, size_t row) const
        {
            return data.at(metadata.indexOf(col, row));
        }

        Occupancy& occupancyAt(size_t col, size_t row) const
        {
            return occupancy.at(metadata.indexOf(col, row));
        }

        T& dataAt(const Vector2Int& indices) const
        {
            return data.at(metadata.indexOf(indices));
        }

        Occupancy& occupancyAt(const Vector2Int& indices) const
        {
            return occupancy.at(metadata.indexOf(indices));
        }

        T& dataAt(const Vector2& coords) const
        {
            return dataAt(metadata.coordinatesToIndices(coords));
        }

        Occupancy& occupancyAt(const Vector2& coords) const
        {
            return occupancyAt(metadata.coordinatesToIndices(coords));
        }

        Grid2D<Occupancy> AsOccupancy() { return Grid2D<Occupancy>(occupancy, occupancy, metadata); }
        Grid2D<T> AsNonOwning() { return Grid2D<T, false>(data, occupancy, metadata); }
    };

    // unlike a Grid, a Map is an always-owning struct
    // it only contains occupancy information, no additional data
    // can be used conveniently through the AsGrid() method
    struct Map2D
    {
        std::vector<Occupancy> occupancy;
        Grid2DMetadata metadata;
        Grid2D<Occupancy> AsGrid() { return Grid2D<Occupancy>(occupancy, occupancy, metadata); }
    };

    class GridUtils
    {
    public:
        GridUtils() = delete;

        // reduce the resolution of an occupancy grid, considering that a cell in the coarser map is occupied as soon as a single smaller cell in it is
        static void reduceOccupancyMap(const std::vector<int8_t>& map, size_t mapWidth, size_t mapHeight, std::vector<Occupancy>& occupancy,
                                       const Grid2DMetadata& metadata)
        {
            int scale = metadata.scale; // scale for dynamic map reduction
            for (int j = 0; j < metadata.dimensions.y; j++)
            {
                for (int i = 0; i < metadata.dimensions.x; i++)
                {
                    bool squareIsFree = true;
                    bool squareIsObstacle = false;

                    for (int row = j * scale; row < (j + 1) * scale && row < mapHeight; row++)
                    {
                        for (int col = i * scale; col < (i + 1) * scale && col < mapWidth; col++)
                        {
                            int value = map.at(col + row * mapWidth);
                            if (value != (int)Occupancy::Free)
                                squareIsFree = false;
                            if (value == (int)Occupancy::Obstacle)
                                squareIsObstacle = true;
                        }
                    }
                    if (squareIsFree)
                        occupancy.at(metadata.indexOf({i, j})) = Occupancy::Free;
                    else if (squareIsObstacle)
                        occupancy.at(metadata.indexOf({i, j})) = Occupancy::Obstacle;
                    else
                        occupancy.at(metadata.indexOf({i, j})) = Occupancy::Unknown;
                }
            }
        }

        // run the DDA algorithm to check if a straight line from origin to end intersects any obstacles
        static bool PathFree(Grid2DMetadata metadata, const std::vector<Occupancy>& occupancy, const Vector2& origin, const Vector2& end)
        {
            Vector2Int originInd = metadata.coordinatesToIndices(origin);
            Vector2Int endInd = metadata.coordinatesToIndices(end);

            DDA::_2D::Map<GSL::Occupancy> map(occupancy, metadata.origin, metadata.cellSize, {metadata.dimensions.x, metadata.dimensions.y});

            // check there are no obstacles between origin and end
            if (!(occupancy.at(metadata.indexOf(originInd)) == Occupancy::Free && occupancy.at(metadata.indexOf(endInd)) == Occupancy::Free))
                return false;
            Vector2 direction = end - origin;
            DDA::_2D::RayCastInfo raycastInfo = DDA::_2D::castRay<GSL::Occupancy>(
                origin, direction, vmath::length(direction),
                DDA::_2D::Map<GSL::Occupancy>(occupancy, metadata.origin, metadata.cellSize, {metadata.dimensions.x, metadata.dimensions.y}),
                [](const GSL::Occupancy& occ)
                {
                    return occ == GSL::Occupancy::Free;
                });

            return !raycastInfo.hitSomething;
        }

        static Map2D CropMap(Grid2D<Occupancy> grid, AABB2D bounds)
        {
            // ensure the aabb is within the bounds of the original map
            Vector2 maxCoords = grid.metadata.indicesToCoordinates(grid.metadata.dimensions, false) - Vector2{0.001, 0.001};
            bounds.min.x = std::clamp(bounds.min.x, grid.metadata.origin.x, maxCoords.x);
            bounds.min.y = std::clamp(bounds.min.y, grid.metadata.origin.y, maxCoords.y);
            bounds.max.x = std::clamp(bounds.max.x, grid.metadata.origin.x, maxCoords.x);
            bounds.max.y = std::clamp(bounds.max.y, grid.metadata.origin.y, maxCoords.y);

            // get croppin'
            Map2D cropped;
            cropped.metadata.cellSize = grid.metadata.cellSize;
            cropped.metadata.origin = bounds.min;
            cropped.metadata.dimensions = vmath::ceil(bounds.size() / grid.metadata.cellSize);
            cropped.metadata.numFreeCells = 0;

            cropped.occupancy.resize(cropped.metadata.dimensions.x * cropped.metadata.dimensions.y);

            Vector2Int offset = grid.metadata.coordinatesToIndices(cropped.metadata.origin);
            for (size_t i = 0; i < cropped.metadata.dimensions.x; i++)
                for (size_t j = 0; j < cropped.metadata.dimensions.y; j++)
                {
                    cropped.AsGrid().occupancyAt(i, j) = grid.occupancyAt(Vector2Int(i, j) + offset);
                    if (cropped.AsGrid().occupancyAt(i, j))
                        cropped.metadata.numFreeCells++;
                }

            return cropped;
        }
    };

} // namespace GSL