#pragma once
#include "gsl_server/algorithms/Semantics/Semantics/Common/AABB.hpp"
#include <vector>
#include <memory>
#include <gsl_server/core/Vectors.hpp>
#include <gsl_server/algorithms/Common/Grid2D.hpp>

// NQA Quadtree stands for Not Quite A Quadtree, as we are allowing some nodes to have 2 children rather than 4 under special circumstances
// Why? Well, why not?

namespace GSL::NQA
{
    class Quadtree;

    struct Node
    {
        Node(Vector2Int _origin, Vector2Int _size);

        GSL::Vector2Int origin;
        GSL::Vector2Int size;

        Occupancy value; // all "cells" (or pixels, or whatever) in this node have the same value in the image

        //children are arranged in this order: top-left, top-right, bottom-left, bottom-right
        std::array<std::shared_ptr<Node>, 4> children;

        static std::shared_ptr<Node> createNode(Vector2Int _origin, Vector2Int _size);

        bool SubdivideIfNeeded(Grid2D<Occupancy> _map);
        bool ForceSubdivide(); // returns false if it is not a leaf or is too small to subdivide
        bool isLeaf() {return children[0] == nullptr && children[1] == nullptr && children[2] == nullptr && children[3] == nullptr;}
        AABB2DInt getAABB() const { return AABB2DInt{origin, origin + size}; }
    private:
    };

    class Quadtree
    {
    public:
        Quadtree(const Grid2D<Occupancy>& map);

        std::shared_ptr<Node> root; //there is no global collection of nodes, each node owns its direct children
        std::vector<std::weak_ptr<Node>> leaves;

        const Grid2D<Occupancy> map;

        // returns a vector that contains Nodes created by fusing free leaves together to create larger blocks
        // *tries* to keep the resulting nodes square-ish, and never exceeds maxSize in either dimension
        // ultimately, this a greedy heuristic method that is not guaranteed to generate the minimum number of nodes possible for the given size
        std::vector<Node> fusedLeaves(int maxSize);
    };
} // namespace GSL::NQA