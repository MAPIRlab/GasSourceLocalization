#pragma once
#include "gsl_server/algorithms/Common/Grid2D.hpp"
#include <gmrf_wind_core/gmrf_map.h>
#include <gsl_server/core/Vectors.hpp>

namespace GSL
{
    struct Node
    {
        Node(const Grid2DMetadata& metadata,
             const std::vector<Occupancy>& occupancy,
             gmrfw::CGMRF_map::Parameters gmrf_params);

        void SetOccupancy(const Grid2DMetadata& metadata, const std::vector<Occupancy>& _occupancy);
        void AddObservation(Vector2 location, Vector2 windVector);
        const Grid2D<Vector2> GetWindMap();
        
    private:
        static gmrfw::TOccupancyMap ToGMRFOcc(const std::vector<Occupancy>& _occ, const Grid2DMetadata& metadata);
        Grid2D<Vector2> AsGrid();

        std::vector<Occupancy> occupancy;
        std::vector<Vector2> wind;
        std::optional<gmrfw::CGMRF_map> gmrf;
        Grid2DMetadata gridMetadata;
        gmrfw::CGMRF_map::Parameters gmrf_parameters;
    };

    struct Arc
    {
        std::weak_ptr<Node> from;
        std::weak_ptr<Node> to;
        float weight;
    };

    class Graph
    {
        std::vector<std::shared_ptr<Node>> nodes;
    };
} // namespace GSL