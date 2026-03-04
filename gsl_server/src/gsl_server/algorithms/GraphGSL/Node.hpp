#pragma once
#include "gsl_server/algorithms/Common/Grid2D.hpp"
#include "gsl_server/algorithms/Semantics/Semantics/Common/AABB.hpp"
#include <gmrf_wind_core/gmrf_map.h>
#include <gsl_server/core/Vectors.hpp>

namespace GSL
{
    struct Arc
    {
        std::weak_ptr<class Node> to;
        float weight;
        AABB2D aabb;
        Vector2 spawnPoint;
    };

    class Node
    {
    public:
        virtual Vector2 GetPosition() = 0;

        std::vector<Arc> arcs;
        std::string id;
    };

    class RealNode : public Node
    {
    public:
        RealNode(Grid2D<Occupancy> grid, gmrfw::CGMRF_map::Parameters gmrf_params);

        void SetOccupancy(Grid2D<Occupancy> grid);
        void AddObservation(Vector2 location, Vector2 windVector);
        void AddObservation(Vector2 location, float gasObs);
        const Grid2D<Vector2> GetWindMap();
        Vector2 GetPosition() override { return centroid; }

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

    class EmptyNode : public Node
    {
    public:
        EmptyNode(Vector2 pos) : position(pos)
        {}
        Vector2 GetPosition() override { return position; }

    private:
        Vector2 position;
    };
} // namespace GSL