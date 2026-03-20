#include "Node.hpp"

namespace GSL
{

    RealNode::RealNode(Grid2D<Occupancy> grid, gmrfw::CGMRF_map::Parameters gmrf_params)
        : gmrf_parameters(gmrf_params)
    {
        SetOccupancy(grid);
    }

    gmrfw::TOccupancyMap RealNode::ToGMRFOcc(const std::vector<Occupancy>& _occ, const Grid2DMetadata& metadata)
    {
        gmrfw::TOccupancyMap occMap;

        std::transform(_occ.begin(), _occ.end(), std::back_inserter(occMap.data), [](const Occupancy value) -> int8_t
                       {
                           return static_cast<int8_t>(value);
                       });

        occMap.width = metadata.dimensions.x;
        occMap.height = metadata.dimensions.y;
        occMap.resolution = metadata.cellSize;
        occMap.origin_x = metadata.origin.x;
        occMap.origin_y = metadata.origin.y;

        return occMap;
    }

    void RealNode::SetOccupancy(Grid2D<Occupancy> grid)
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
        if (gmrf)
            observations = gmrf->getObservations_GMRF();

        gmrf.emplace(ToGMRFOcc(occupancy, gridMetadata), gmrf_parameters, false, false);
        gas.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y);  // TODO what happens to the gas map on resize?
        wind.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y); // this is fine, because the wind map will be overriden entirely on next query
        outletMask.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y, -1);
        windDirty = true;

        if (observations.size() > 0)
            gmrf->setObservations_GMRF(observations);
    }

    bool RealNode::IsValidPoint(Vector2 location)
    {
        Vector2Int indices = gridMetadata.coordinatesToIndices(location);
        if (!gridMetadata.indicesInBounds(indices))
            return false;

        return AsGrid().freeAt(indices);
    }

    bool RealNode::AddObservation(Vector2 location, Vector2 windVector)
    {
        constexpr float variance = 0.001; // TODO

        float speed = vmath::length(windVector);
        float direction = std::atan2(windVector.y, windVector.x);
        bool accepted = gmrf->insertObservation_GMRF(
            speed,
            direction,
            variance, variance,
            location.x, location.y);
        if (accepted)
            windDirty = true;
        return accepted;
    }

    bool RealNode::AddObservation(Vector2 location, float gasObs)
    {
        // TODO
        return IsValidPoint(location);
    }

    void RealNode::UpdateArcsMask()
    {
        outletMask.resize(gridMetadata.dimensions.x * gridMetadata.dimensions.y, -1);
        std::fill(outletMask.begin(), outletMask.end(), -1);

        Grid2D<int> maskGrid(outletMask, occupancy, gridMetadata);
        for (size_t i = 0; i < arcs.size(); i++)
        {
            const Arc& arc = arcs.at(i);
            AABB2DInt aabbIdx{
                gridMetadata.coordinatesToIndices(arc.aabb.min),
                gridMetadata.coordinatesToIndices(arc.aabb.max)};

            for (Vector2Int indices : aabbIdx)
                if (maskGrid.metadata.indicesInBounds(indices) && maskGrid.freeAt(indices))
                    maskGrid.dataAt(indices) = i;
        }
    }

    const Grid2D<Vector2> RealNode::GetWindMap()
    {
        if (windDirty)
        {
            gmrf->MAP_estimation_GMRF(50);

#pragma omp parallel for
            for (size_t i = 0; i < wind.size(); i++)
            {
                Vector2 coords = gridMetadata.indexToCoordinates(i);
                gmrfw::WindVector vec = gmrf->getEstimation(coords.x, coords.y);
                wind.at(i).x = vec.x;
                wind.at(i).y = vec.y;
            }
            windDirty = false;
        }
        return AsGrid();
    }

    const Grid2D<int> RealNode::GetOutletsMask()
    {
        return Grid2D<int>(outletMask, occupancy, gridMetadata);
    }

    Grid2D<Vector2> RealNode::AsGrid()
    {
        return Grid2D<Vector2>(wind, occupancy, gridMetadata);
    }

    const Grid2D<Occupancy> RealNode::GetOccupancy()
    {
        return Grid2D<Occupancy>(occupancy, occupancy, gridMetadata);
    }
} // namespace GSL