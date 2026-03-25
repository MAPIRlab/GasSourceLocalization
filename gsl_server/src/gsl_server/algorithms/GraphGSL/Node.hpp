#pragma once
#include "gsl_server/algorithms/Common/Grid2D.hpp"
#include "gsl_server/algorithms/Semantics/Semantics/Common/AABB.hpp"
#include <gmrf_wind_core/gmrf_map.h>
#include <gsl_server/core/Vectors.hpp>

namespace GSL
{
    struct Arc
    {
        std::weak_ptr<class Node> from;
        std::weak_ptr<class Node> to;
        float weight;
        AABB2D aabb;
        Vector2 spawnPoint;
    };

    class Node
    {
    public:
        virtual Vector2 GetPosition() = 0;
        virtual bool IsValidPoint(Vector2 location) = 0;
        virtual bool AddObservation(Vector2 location, Vector2 windVector) = 0;
        virtual bool AddObservation(Vector2 location, float gasObs) = 0;
        virtual void UpdateArcsMask() {}

        std::vector<Arc> arcs;
        std::string id;
    };

    class RealNode : public Node
    {
    public:
        RealNode(Grid2D<Occupancy> grid, gmrfw::CGMRF_map::Parameters gmrf_params);

        bool IsValidPoint(Vector2 location) override;
        bool AddObservation(Vector2 location, Vector2 windVector) override;
        bool AddObservation(Vector2 location, float gasObs) override;
        void UpdateArcsMask() override;

        void SetOccupancy(Grid2D<Occupancy> grid);
        const Grid2D<Occupancy> GetOccupancy();
        const Grid2D<Vector2> GetWindMap();
        const Grid2D<int> GetOutletsMask();
        Vector2 GetPosition() override { return centroid; }
        bool isDirty() const { return windDirty; }

    private:
        static gmrfw::TOccupancyMap ToGMRFOcc(const std::vector<Occupancy>& _occ, const Grid2DMetadata& metadata);
        Grid2D<Vector2> AsGrid();

        bool windDirty = false;
        std::vector<Occupancy> occupancy;
        std::vector<int> outletMask;
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
        bool IsValidPoint(Vector2 location) override { return false; }
        bool AddObservation(Vector2 location, Vector2 windVector) override { return false; }
        bool AddObservation(Vector2 location, float gasObs) override { return false; }

    private:
        Vector2 position;
    };
} // namespace GSL