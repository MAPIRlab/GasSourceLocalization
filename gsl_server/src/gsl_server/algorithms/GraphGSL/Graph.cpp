#include "Graph.hpp"

namespace GSL
{

    Node::Node(const Grid2DMetadata& metadata, const std::vector<Occupancy>& _occupancy, gmrfw::CGMRF_map::Parameters gmrf_params)
        : gmrf_parameters(gmrf_params)
    {
        SetOccupancy(metadata, _occupancy);
    }

    gmrfw::TOccupancyMap Node::ToGMRFOcc(const std::vector<Occupancy>& _occ, const Grid2DMetadata& metadata)
    {
        gmrfw::TOccupancyMap occMap;

        std::transform(_occ.begin(), _occ.end(), std::back_inserter(occMap.data), [](const Occupancy value)
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

    void Node::SetOccupancy(const Grid2DMetadata& metadata, const std::vector<Occupancy>& _occupancy)
    {
        gridMetadata = metadata;
        occupancy = _occupancy;

        // re-create the gmrf map, keeping the history of observations
        std::vector<gmrfw::TobservationGMRF> observations;
        if (gmrf)
            observations = gmrf->getObservations_GMRF();

        gmrf.emplace(ToGMRFOcc(occupancy, metadata), gmrf_parameters, false, false);
        wind.resize(metadata.dimensions.x * metadata.dimensions.y);

        if (observations.size() > 0)
            gmrf->setObservations_GMRF(observations);
    }

    void Node::AddObservation(Vector2 location, Vector2 windVector)
    {
        constexpr float variance = 0.001;
        gmrf->insertObservation_GMRF(
            vmath::length(windVector),
            std::atan2(windVector.y, windVector.x),
            variance, variance,
            location.x, location.y);
    }

    const Grid2D<Vector2> Node::GetWindMap()
    {
        // TODO caching
        gmrf->MAP_estimation_GMRF();

#pragma omp parallel for
        for (size_t i = 0; i < wind.size(); i++)
        {
            Vector2 coords = gridMetadata.indexToCoordinates(i);
            gmrfw::WindVector vec = gmrf->getEstimation(coords.x, coords.y);
            wind.at(i).x = vec.x;
            wind.at(i).y = vec.y;
        }
        return AsGrid();
    }

    Grid2D<Vector2> Node::AsGrid()
    {
        return Grid2D<Vector2>(wind, occupancy, gridMetadata);
    }

} // namespace GSL