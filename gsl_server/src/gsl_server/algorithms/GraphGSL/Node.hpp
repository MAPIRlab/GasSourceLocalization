#pragma once
#include "gsl_server/algorithms/Common/Grid2D.hpp"
#include "gsl_server/algorithms/Common/KernelDMVW/KernelDMVW.hpp"
#include "gsl_server/algorithms/Common/NQAQuadtree.hpp"
#include "gsl_server/algorithms/Common/VisibilityMap.hpp"
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
        std::set<std::shared_ptr<DoorwayNode>> samePhysicalDoorway;

        DoorwayNode(const std::string& _name);
        std::string_view GetName() const { return name; }

        std::string_view GetDebuggingName() const { return _debugging_name; }
        void SetDebuggingName(std::string_view debugging_name) { _debugging_name = debugging_name; }

        size_t GetIndex() const;
        const std::shared_ptr<DoorwayNode> OtherSide() const; // the node which represents the other direction through this doorway

    private:
        std::string name;            // name is shared between the two directional versions of the doorway
        std::string _debugging_name; // NOT shared, this is only to make the debugger show a readable identifier
    };

    struct CellIdentifier
    {
        std::shared_ptr<PlaceNode> node;
        Vector2Int indices;

        static inline const Vector2Int WHOLE_NODE = {-INT_MAX, -INT_MAX};
        static constexpr float AABBCENTER = -1; // used to distinguish aabb entries in the expectedGasMaps structure
    };

    class PlaceNode : public std::enable_shared_from_this<PlaceNode>
    {
    public:
        virtual Vector2 GetPosition() const = 0;
        virtual bool IsValidPoint(Vector2 location) = 0;
        virtual bool AddObservation(Vector2 location, Vector2 wind, float gasObs) = 0;
        virtual void UpdateDoorwayMask() {}
        const std::shared_ptr<DoorwayNode> GetDoorway(std::string_view name);
        virtual std::vector<Vector2> RepresentativePoints() const { return {GetPosition()}; }
        CellIdentifier GetNodeIdentifier() { return CellIdentifier{shared_from_this(), CellIdentifier::WHOLE_NODE}; }
        virtual void ResetObservations() {}

        std::vector<std::shared_ptr<DoorwayNode>> doorways;
        std::string id;
    };

    class RoomNode : public PlaceNode
    {
    public:
        RoomNode(Grid2D<Occupancy> grid, KernelDMVW::GasMap::Params kernelParams);

        bool IsValidPoint(Vector2 location) override;
        bool AddObservation(Vector2 location, Vector2 wind, float gasObs) override;
        void UpdateDoorwayMask() override;

        void UpdateWindMap(std::shared_ptr<gmrfw::CGMRF_map> gmrf);
        void SetOccupancy(Grid2D<Occupancy> grid);
        const Grid2D<Occupancy> GetOccupancy() const;
        const Grid2D<Vector2> GetWindMap();
        const Grid2D<KernelDMVW::KernelCell> GetGasMap();
        const Grid2D<int> GetOutletsMask();
        Grid2D<float> GetSourceProbabilities();
        const std::vector<size_t>& GetOutletsCellCount();
        Vector2 GetPosition() const override { return centroid; }
        AABB2D GetAABB() const;
        const std::vector<NQA::Node>& GetQuadtreeLeaves() const { return quadtreeLeaves; }
        std::vector<Vector2> RepresentativePoints() const override;
        const VisibilityMap& GetVisibilityMap() const { return visibilityMap; }
        CellIdentifier GetCellIdentifier(size_t index);
        void ResetObservations() override;

    private:
        Grid2D<Vector2> WindAsGrid();

        std::vector<Occupancy> occupancy;
        std::vector<int> outletMask;
        std::vector<size_t> numCellsOutlet;
        std::vector<Vector2> wind;
        std::vector<float> sourceProbabilities;

        KernelDMVW::GasMap gasMap;
        Grid2DMetadata gridMetadata;
        Vector2 centroid;
        std::vector<NQA::Node> quadtreeLeaves;
        VisibilityMap visibilityMap;
    };

    class OutsideNode : public PlaceNode
    {
    public:
        OutsideNode(Vector2 pos) : position(pos)
        {}
        Vector2 GetPosition() const override { return position; }
        bool IsValidPoint(Vector2 location) override { return false; }
        bool AddObservation(Vector2 location, Vector2 wind, float gasObs) override { return false; }

    private:
        Vector2 position;
    };

} // namespace GSL

namespace std
{
    template <> struct hash<GSL::CellIdentifier>
    {
        size_t operator()(const GSL::CellIdentifier& x) const
        {
            return (size_t)(x.node.get()) ^ (size_t)(x.indices.x) ^ (size_t)(x.indices.y);
        }
    };

    template <> struct less<GSL::CellIdentifier>
    {
        bool operator()(const GSL::CellIdentifier& a, const GSL::CellIdentifier& b) const
        {
            return a.node < b.node                                    //
                   || (a.node == b.node && a.indices.x < b.indices.x) //
                   || (a.node == b.node && a.indices.x == b.indices.x && a.indices.y < b.indices.y);
        }
    };
} // namespace std
