#include "Node.hpp"

namespace GSL
{

    RealNode::RealNode(const Grid2DMetadata& metadata, const std::vector<Occupancy>& _occupancy, gmrfw::CGMRF_map::Parameters gmrf_params)
        : gmrf_parameters(gmrf_params)
    {
        SetOccupancy(metadata, _occupancy);
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

    void RealNode::SetOccupancy(const Grid2DMetadata& metadata, const std::vector<Occupancy>& _occupancy)
    {
        gridMetadata = metadata;
        occupancy = _occupancy;

        // recompute centroid
        {
            Vector2 coordinatesSum(0, 0);
            for (size_t i = 0; i < occupancy.size(); i++)
                if (occupancy.at(i) == Occupancy::Free)
                    coordinatesSum += gridMetadata.indexToCoordinates(i);

            centroid = coordinatesSum / metadata.numFreeCells;
        }

        // re-create the gmrf map, keeping the history of observations
        std::vector<gmrfw::TobservationGMRF> observations;
        if (gmrf)
            observations = gmrf->getObservations_GMRF();

        gmrf.emplace(ToGMRFOcc(occupancy, metadata), gmrf_parameters, false, false);
        gas.resize(metadata.dimensions.x * metadata.dimensions.y);  // TODO what happens to the gas map on resize?
        wind.resize(metadata.dimensions.x * metadata.dimensions.y); // this is fine, because the wind map will be overriden entirely on next query
        windDirty = true;

        if (observations.size() > 0)
            gmrf->setObservations_GMRF(observations);
    }

    void RealNode::AddObservation(Vector2 location, Vector2 windVector)
    {
        constexpr float variance = 0.001; // TODO
        gmrf->insertObservation_GMRF(
            vmath::length(windVector),
            std::atan2(windVector.y, windVector.x),
            variance, variance,
            location.x, location.y);
        windDirty = true;
    }

    void RealNode::AddObservation(Vector2 location, float gasObs)
    {
        // TODO
    }

    const Grid2D<Vector2> RealNode::GetWindMap()
    {
        if (windDirty)
        {
            gmrf->MAP_estimation_GMRF();

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

    Grid2D<Vector2> RealNode::AsGrid()
    {
        return Grid2D<Vector2>(wind, occupancy, gridMetadata);
    }
} // namespace GSL