#pragma once
#include <vector>
#include <deque>
#include <memory>
#include <gsl_server/core/Vectors.hpp>

namespace GSL::Utils::NQA
{
    class Quadtree;

    struct Node
    {
        friend class Quadtree;

        GSL::Vector2Int origin;
        GSL::Vector2Int size;

        Node* parent;
        Quadtree* quadtree;

        bool isLeaf;
        uint8_t value; // all "cells" (or pixels, or whatever) in this node have the same value in the image
        std::array<std::shared_ptr<Node>, 4> children;

        static std::shared_ptr<Node> createNode(Quadtree* qt, GSL::Vector2Int _origin, GSL::Vector2Int _size,
                                                const std::vector<std::vector<uint8_t>>& map);

        bool subdivide(); // returns false if it is not a leaf or is too small to subdivide

        Node(Quadtree* qt, GSL::Vector2Int _origin, GSL::Vector2Int _size, const std::vector<std::vector<uint8_t>>& map);

    private:
        const std::vector<std::vector<uint8_t>>& map;
        bool initialize();
    };

    // Not-Quite-A-Quadtree. Some nodes have two children instead of 4!
    class Quadtree
    {
    public:
        std::shared_ptr<Node> root;

        std::vector<std::weak_ptr<Node>> leaves;

        Quadtree(const std::vector<std::vector<uint8_t>>& map);

        const std::vector<std::vector<uint8_t>> map;

        std::vector<Node> fusedLeaves(int maxSize);
    };
} // namespace GSL::Utils::NQA