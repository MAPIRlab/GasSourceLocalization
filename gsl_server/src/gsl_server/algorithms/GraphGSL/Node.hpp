#pragma once
#include <gmrf_wind_core/gmrf_map.h>
#include <gsl_server/core/Vectors.hpp>
#include "gsl_server/algorithms/Common/Grid2D.hpp"
#include "gsl_server/algorithms/Semantics/Semantics/Common/AABB.hpp"

namespace GSL
{
    struct Arc
    {
        std::weak_ptr<class Node> to;
        float weight;
        AABB2D aabb;
        Vector2 spawnPoint;
    };
    
    struct Node
    {
        Node(const Grid2DMetadata& metadata,
             const std::vector<Occupancy>& occupancy,
             gmrfw::CGMRF_map::Parameters gmrf_params);

        void SetOccupancy(const Grid2DMetadata& metadata, const std::vector<Occupancy>& _occupancy);
        void AddObservation(Vector2 location, Vector2 windVector);
        void AddObservation(Vector2 location, float gasObs);
        const Grid2D<Vector2> GetWindMap();
        Vector2 GetCentroid() {return centroid;}

        std::vector<Arc> arcs;

    private:
        static gmrfw::TOccupancyMap ToGMRFOcc(const std::vector<Occupancy>& _occ, const Grid2DMetadata& metadata);
        Grid2D<Vector2> AsGrid();

        bool windDirty = true;
        std::vector<Occupancy> occupancy;
        std::vector<Vector2> wind;
        std::vector<float> gas;
        std::optional<gmrfw::CGMRF_map> gmrf;
        Grid2DMetadata gridMetadata;
        gmrfw::CGMRF_map::Parameters gmrf_parameters;
        Vector2 centroid;
    };
} // namespace GSL