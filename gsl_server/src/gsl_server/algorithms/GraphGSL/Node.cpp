#include "Node.hpp"
#include "gsl_server/algorithms/Common/Utils/Pointers.hpp"
#include <gsl_server/algorithms/Common/NQAQuadtree.hpp>

namespace GSL
{

    RoomNode::RoomNode(Grid2D<Occupancy> grid, KernelDMVW::GasMap::Params kernelParams)
        : gasMap(grid, kernelParams),
          visibilityMap(grid.metadata.dimensions.x, grid.metadata.dimensions.y, 5)
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

        size_t numCells = gridMetadata.dimensions.x * gridMetadata.dimensions.y;
        wind.resize(numCells);
        outletMask.resize(numCells, -1);
        sourceProbabilities.resize(numCells);
        for (size_t i = 0; i < numCells; i++)
            if (occupancy.at(i))
                sourceProbabilities.at(i) = 1.f / gridMetadata.numFreeCells;
            else
                sourceProbabilities.at(i) = 0;

        // quadtree decomposition
        NQA::Quadtree quadtree(GetOccupancy());
        size_t best = std::numeric_limits<size_t>::max();
        for (size_t i = 5; i < 10; i++)
        {
            std::vector<NQA::Node> _quadtreeLeaves = quadtree.fusedLeaves(i);
            if (_quadtreeLeaves.size() < best)
            {
                best = _quadtreeLeaves.size();
                quadtreeLeaves = _quadtreeLeaves;
            }
        }

        // assert the leaves are actually free
        for (const auto& leaf : quadtreeLeaves)
            for (Vector2Int indices : leaf.getAABB())
                GSL_ASSERT(GetOccupancy().occupancyAt(indices));

        // visibility map (simulation optimization)
        visibilityMap.Populate(GetOccupancy());
    }

    bool RoomNode::IsValidPoint(Vector2 location)
    {
        Vector2Int indices = gridMetadata.coordinatesToIndices(location);
        if (!gridMetadata.indicesInBounds(indices))
            return false;

        return WindAsGrid().occupancyAt(indices);
    }

    bool RoomNode::AddObservation(Vector2 location, Vector2 wind, float gasObs)
    {
        gasMap.AddReading(gasObs, wind, location);
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
            const std::shared_ptr<DoorwayNode> doorway = doorways.at(i);
            AABB2DInt aabbIdx{
                gridMetadata.coordinatesToIndices(doorway->aabb.min),
                gridMetadata.coordinatesToIndices(doorway->aabb.max)};

            for (Vector2Int indices : aabbIdx)
                if (maskGrid.metadata.indicesInBounds(indices) && maskGrid.occupancyAt(indices))
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

    Grid2D<Vector2> RoomNode::GetWindMap()
    {
        return WindAsGrid();
    }

    const Grid2D<KernelDMVW::KernelCell> RoomNode::GetGasMap()
    {
        return gasMap.GetMap();
    }

    const Grid2D<int> RoomNode::GetOutletsMask()
    {
        return Grid2D<int>(outletMask, occupancy, gridMetadata);
    }

    Grid2D<float> RoomNode::GetSourceProbabilities()
    {
        return Grid2D<float>(sourceProbabilities, occupancy, gridMetadata);
    }

    const std::vector<size_t>& RoomNode::GetOutletsCellCount()
    {
        return numCellsOutlet;
    }

    AABB2D RoomNode::GetAABB() const
    {
        return gridMetadata.GetAABB();
    }

    std::vector<Vector2> RoomNode::RepresentativePoints() const
    {
        constexpr size_t num = 2;

        AABB2D aabb = GetAABB();
        Vector2 step = aabb.size() / (num + 1);

        std::vector<Vector2> points;

        for (size_t i = 1; i <= num; i++)
            for (size_t j = 1; j <= num; j++)
            {
                Vector2 p = aabb.min + Vector2(step.x * i, step.y * j);
                if (GetOccupancy().occupancyAt(p))
                    points.push_back(p);
            }
        return points;
    }

    CellIdentifier RoomNode::GetCellIdentifier(size_t index)
    {
        return CellIdentifier{this, gridMetadata.indices2D(index)};
    }

    void RoomNode::ResetObservations()
    {
        gasMap.Reset();
        wind.assign(wind.size(), Vector2{0, 0});
    }

    Grid2D<Vector2> RoomNode::WindAsGrid()
    {
        return Grid2D<Vector2>(wind, occupancy, gridMetadata);
    }

    const Grid2D<Occupancy> RoomNode::GetOccupancy() const
    {
        // this is a little funky because the Grid class holds modifyable references, so even though we are returning a *const* grid, the references *inside* the object are not const
        // therefore, we have to do a cast to non-const (through the pointer, to avoid copies) before constructing the grid
        const Grid2D<Occupancy> grid(
            *(std::vector<Occupancy>*)&occupancy,
            *(std::vector<Occupancy>*)&occupancy,
            *(Grid2DMetadata*)&gridMetadata);
        return grid;
    }

    const std::shared_ptr<DoorwayNode> PlaceNode::GetDoorway(std::string_view name)
    {
        for (const std::shared_ptr<DoorwayNode> doorway : doorways)
            if (doorway->GetName() == name)
                return doorway;
        GSL_ERROR("Place node {} has no doorway named {}", id, name);
        throw std::exception();
    }

    DoorwayNode::DoorwayNode(const std::string& _name) : name(_name), _debugging_name(name)
    {}

    void DoorwayNode::FitToMapEdge()
    {
        auto fromNode = from.lock();
        if (!fromNode || !Is<RoomNode>(fromNode))
            return;

        auto occupancy = As<RoomNode>(fromNode)->GetOccupancy();
        AABB2DInt aabbIdx{
            occupancy.metadata.coordinatesToIndices(aabb.min),
            occupancy.metadata.coordinatesToIndices(aabb.max)};

        std::vector<Vector2Int> validIndices;
        for (Vector2Int indices : aabbIdx)
        {
            bool isEdge = indices.x == 0 || indices.x == occupancy.metadata.dimensions.x - 1 || indices.y == 0 || indices.y == occupancy.metadata.dimensions.y - 1;
            isEdge = isEdge && occupancy.metadata.indicesInBounds(indices);

            // cell is also an "edge" cell if one of the neighbors is unknown (other room)
            if (!isEdge && occupancy.metadata.indicesInBounds(indices))
            {
                std::vector<Vector2Int> neighbors{indices + Vector2Int{-1, 0},
                                                  indices + Vector2Int{1, 0},
                                                  indices + Vector2Int{0, -1},
                                                  indices + Vector2Int{0, 1}};
                for (Vector2Int neighbor : neighbors)
                {
                    if (occupancy.metadata.indicesInBounds(neighbor) && occupancy.dataAt(neighbor) == Occupancy::Unknown)
                    {
                        isEdge = true;
                        break;
                    }
                }
            }

            if (isEdge && occupancy.dataAt(indices))
                validIndices.push_back(indices);
        }

        AABB2DInt newAABB;
        for (const Vector2Int& indices : validIndices)
        {
            newAABB.min.x = std::min(newAABB.min.x, indices.x);
            newAABB.min.y = std::min(newAABB.min.y, indices.y);
            newAABB.max.x = std::max(newAABB.max.x, indices.x);
            newAABB.max.y = std::max(newAABB.max.y, indices.y);
        }

        GSL_VERIFY_MSG(validIndices.size() > 0 && (newAABB.min.x == newAABB.max.x || newAABB.min.y == newAABB.max.y),
                       "Doorway '{}' AABB is not a line after fitting to map edge. Its geometric definition is probably not correct", _debugging_name);

        aabb = occupancy.metadata.indicesToCoordinates(newAABB);
    }

    size_t DoorwayNode::GetIndex() const
    {
        return std::distance(from.lock()->doorways.begin(),
                             std::find_if(from.lock()->doorways.begin(),
                                          from.lock()->doorways.end(),
                                          [this](const std::shared_ptr<DoorwayNode>& other)
                                          {
                                              return other.get() == this;
                                          }));
    }

    const std::shared_ptr<DoorwayNode> DoorwayNode::OtherSide() const
    {
        return to.lock()->GetDoorway(name);
    }
} // namespace GSL