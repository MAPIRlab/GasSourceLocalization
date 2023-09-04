#pragma once
#include <array>
#include <Utils/Vector2Int.h>
#include <Utils/Vector2.h>
#include <deque>
#include <memory>

namespace NQA
{
    class NQAQuadtree;

    struct Node
    {
        friend class NQAQuadtree;

        Utils::Vector2Int origin;
        Utils::Vector2Int size;

        Node* parent;
        NQAQuadtree* quadtree;

        bool isLeaf;
        uint8_t value; // all "cells" (or pixels, or whatever) in this node have the same value in the image
        std::array<std::shared_ptr<Node>, 4> children;

        static std::shared_ptr<Node> createNode(NQAQuadtree* qt, Utils::Vector2Int _origin, Utils::Vector2Int _size, const std::vector<std::vector<uint8_t>>& map);

        bool subdivide(); // returns false if it is not a leaf or is too small to subdivide

        Node(NQAQuadtree* qt, Utils::Vector2Int _origin, Utils::Vector2Int _size, const std::vector<std::vector<uint8_t>>& map);

    private:
        const std::vector<std::vector<uint8_t>>& map;
        bool initialize();
    };

    // Not-Quite-A-Quadtree. Some nodes have two children instead of 4!
    class NQAQuadtree
    {
    public:
        std::shared_ptr<Node> root;

        std::vector<std::weak_ptr<Node>> leaves;

        NQAQuadtree(const std::vector<std::vector<uint8_t>>& map);

        const std::vector<std::vector<uint8_t>> map;

        std::vector<Node> fusedLeaves(int maxSize);
    };
}