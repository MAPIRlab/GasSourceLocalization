#include "Node.hpp"

namespace GSL
{

    RoomNode::RoomNode(Grid2D<Occupancy> grid)
    {
        SetOccupancy(grid);
    }

    void RoomNode::SetOccupancy(Grid2D<Occupancy> grid)
    {
        gridMetadata = grid.metadata;
        occupancy = grid.occupancy;

        // recompute centroid
        {
            Vector2 coordinatesSum(0, 0);
            for (size_t i = 0; i < occupancy.size(); i++)
                if (occupancy.at(i) == Occupancy::Free)
                    coordinatesSum += gridMetadata.indexToCoordinates(i);

            centroid = coordinatesSum / gridMetadata.numFreeCells;
        }

        // re-create the gmrf map, keeping the history of observations
        std::vector<gmrfw::TobservationGMRF> observations;

        gas.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y);  // TODO what happens to the gas map on resize?
        wind.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y); // this is fine, because the wind map will be overriden entirely on next query
        outletMask.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y, -1);
    }

    bool RoomNode::IsValidPoint(Vector2 location)
    {
        Vector2Int indices = gridMetadata.coordinatesToIndices(location);
        if (!gridMetadata.indicesInBounds(indices))
            return false;

        return AsGrid().freeAt(indices);
    }

    bool RoomNode::AddObservation(Vector2 location, float gasObs)
    {
        // TODO
        return IsValidPoint(location);
    }

    void RoomNode::UpdateDoorwayMask()
    {
        outletMask.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y, -1);
        std::fill(outletMask.begin(), outletMask.end(), -1);

        Grid2D<int> maskGrid(outletMask, occupancy, gridMetadata);
        numCellsOutlet.resize(doorways.size(), 0);
        for (size_t i = 0; i < doorways.size(); i++)
        {
            const DoorwayNode& arc = doorways.at(i);
            AABB2DInt aabbIdx{
                gridMetadata.coordinatesToIndices(arc.aabb.min),
                gridMetadata.coordinatesToIndices(arc.aabb.max)};

            for (Vector2Int indices : aabbIdx)
                if (maskGrid.metadata.indicesInBounds(indices) && maskGrid.freeAt(indices))
                {
                    maskGrid.dataAt(indices) = i;
                    numCellsOutlet.at(i)++;
                }
        }
    }

    void RoomNode::UpdateWindMap(std::shared_ptr<gmrfw::CGMRF_map> gmrf)
    {
#pragma omp parallel for
        for (size_t i = 0; i < wind.size(); i++)
        {
            Vector2 coords = gridMetadata.indexToCoordinates(i);
            gmrfw::WindVector vec = gmrf->getEstimation(coords.x, coords.y);
            wind.at(i).x = vec.x;
            wind.at(i).y = vec.y;
        }
    }

    const Grid2D<Vector2> RoomNode::GetWindMap()
    {
        return AsGrid();
    }

    const Grid2D<int> RoomNode::GetOutletsMask()
    {
        return Grid2D<int>(outletMask, occupancy, gridMetadata);
    }

    const std::vector<size_t>& RoomNode::GetOutletsCellCount()
    {
        return numCellsOutlet;
    }

    Grid2D<Vector2> RoomNode::AsGrid()
    {
        return Grid2D<Vector2>(wind, occupancy, gridMetadata);
    }

    const Grid2D<Occupancy> RoomNode::GetOccupancy()
    {
        return Grid2D<Occupancy>(occupancy, occupancy, gridMetadata);
    }
} // namespace GSL