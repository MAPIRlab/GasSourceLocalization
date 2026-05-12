#pragma once
#include "gsl_server/algorithms/Common/Grid2D.hpp"
#include "gsl_server/algorithms/Common/KernelDMVW/KernelDMVW.hpp"
#include "gsl_server/algorithms/Semantics/Semantics/Common/AABB.hpp"
#include <gmrf_wind_core/gmrf_map.h>
#include <gsl_server/core/Vectors.hpp>

namespace GSL
{
    struct DoorwayNode
    {
        std::weak_ptr<class PlaceNode> from;
        std::weak_ptr<class PlaceNode> to;
        AABB2D aabb;

        DoorwayNode(const std::string& _name) : name(_name) {}
        std::string_view GetName() const { return name; }

        size_t GetIndex() const;
        const DoorwayNode& OtherSide() const; // the node which represents the other direction through this doorway

    private:
        std::string name; // name is shared between the two directional versions of the doorway
    };

    class PlaceNode
    {
    public:
        virtual Vector2 GetPosition() = 0;
        virtual bool IsValidPoint(Vector2 location) = 0;
        virtual bool AddObservation(Vector2 location, Vector2 wind, float gasObs) = 0;
        virtual void UpdateDoorwayMask() {}
        const DoorwayNode& GetDoorway(std::string_view name);

        std::vector<DoorwayNode> doorways;
        std::string id;
    };

    class RoomNode : public PlaceNode
    {
    public:
        RoomNode(Grid2D<Occupancy> grid);

        bool IsValidPoint(Vector2 location) override;
        bool AddObservation(Vector2 location, Vector2 wind, float gasObs) override;
        void UpdateDoorwayMask() override;

        void UpdateWindMap(std::shared_ptr<gmrfw::CGMRF_map> gmrf);
        void SetOccupancy(Grid2D<Occupancy> grid);
        const Grid2D<Occupancy> GetOccupancy();
        const Grid2D<Vector2> GetWindMap();
        const Grid2D<KernelDMVW::KernelCell> GetGasMap();
        const Grid2D<int> GetOutletsMask();
        const std::vector<size_t>& GetOutletsCellCount();
        Vector2 GetPosition() override { return centroid; }
        AABB2D GetAABB() const;

    private:
        Grid2D<Vector2> WindAsGrid();

        std::vector<Occupancy> occupancy;
        std::vector<int> outletMask;
        std::vector<size_t> numCellsOutlet;
        std::vector<Vector2> wind;
        
        KernelDMVW::GasMap gasMap;
        Grid2DMetadata gridMetadata;
        Vector2 centroid;
    };

    class OutsideNode : public PlaceNode
    {
    public:
        OutsideNode(Vector2 pos) : position(pos)
        {}
        Vector2 GetPosition() override { return position; }
        bool IsValidPoint(Vector2 location) override { return false; }
        bool AddObservation(Vector2 location, Vector2 wind, float gasObs) override { return false; }

    private:
        Vector2 position;
    };
} // namespace GSL