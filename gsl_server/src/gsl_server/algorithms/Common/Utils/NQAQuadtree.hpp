#pragma once
#include <vector>
#include <memory>
#include <gsl_server/core/Vectors.hpp>

// NQA Quadtree stands for Not Quite A Quadtree, as we are allowing some nodes to have 2 children rather than 4 under special circumstances
// Why? Well, why not?

namespace GSL::Utils::NQA
{
    class Quadtree;

    struct Node
    {
        friend class Quadtree;
        Node(Quadtree* qt, GSL::Vector2Int _origin, GSL::Vector2Int _size, const std::vector<std::vector<uint8_t>>& map);

        GSL::Vector2Int origin;
        GSL::Vector2Int size;

        Node* parent;
        Quadtree* quadtree;

        bool isLeaf;
        uint8_t value; // all "cells" (or pixels, or whatever) in this node have the same value in the image

        //children are arranged in this order: top-left, top-right, bottom-left, bottom-right
        std::array<std::shared_ptr<Node>, 4> children;

        static std::shared_ptr<Node> createNode(Quadtree* qt, Vector2Int _origin, Vector2Int _size, const std::vector<std::vector<uint8_t>>& _map);

        bool subdivide(); // returns false if it is not a leaf or is too small to subdivide


    private:
        const std::vector<std::vector<uint8_t>>& map;
        bool initialize();
    };

    class Quadtree
    {
    public:
        Quadtree(const std::vector<std::vector<uint8_t>>& _map, uint maxAllowedSize);

        std::shared_ptr<Node> root; //there is no global collection of nodes, each node owns its direct children
        std::vector<std::weak_ptr<Node>> leaves;

        const std::vector<std::vector<uint8_t>> map;
        uint maxAllowedSize; //how big can leaves be along their biggest axis

        // returns a vector that contains Nodes created by fusing free leaves together to create larger blocks
        // *tries* to keep the resulting nodes square-ish, and never exceeds maxSize in either dimension
        // ultimately, this a greedy heuristic method that is not guaranteed to generate the minimum number of nodes possible for the given size
        std::vector<Node> fusedLeaves(int maxSize);
    };
} // namespace GSL::Utils::NQA