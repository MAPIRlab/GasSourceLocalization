#include <gsl_server/Utils/NQAQuadtree.hpp>
#include <unordered_map>
#include <unordered_set>
#include <list>
#include <set>
#include <cmath>
#include <float.h>

namespace GSL::Utils::NQA
{
    using namespace GSL;
    Quadtree::Quadtree(const std::vector<std::vector<uint8_t>>& _map) : map(_map)
    {
        root = Node::createNode(this, Vector2Int(0, 0), Vector2Int(map.size(), map[0].size()), map);
        root->parent = nullptr;
    }

    std::shared_ptr<Node> Node::createNode(Quadtree* qt, Vector2Int _origin, Vector2Int _size, const std::vector<std::vector<uint8_t>>& _map)
    {
        std::shared_ptr<Node> noderef{new Node(qt, _origin, _size, _map)};

        noderef->isLeaf = noderef->initialize();
        if (noderef->isLeaf && qt)
            qt->leaves.emplace_back(noderef);

        return noderef;
    }

    Node::Node(Quadtree* qt, Vector2Int _origin, Vector2Int _size, const std::vector<std::vector<uint8_t>>& _map)
        : quadtree(qt), origin(_origin), size(_size), isLeaf(true), value(-1), map(_map)
    {
        children[0] = nullptr;
        children[1] = nullptr;
        children[2] = nullptr;
        children[3] = nullptr;
    }

    bool Node::initialize()
    {
        value = map[origin.x][origin.y];

        bool leaf = true;
        for (int i = origin.x; i < origin.x + size.x; i++)
        {
            for (int j = origin.y; j < origin.y + size.y; j++)
            {
                if (map[i][j] != value)
                {
                    leaf = false;
                    break;
                }
            }
            if (!leaf)
                break;
        }

        if (!leaf)
            leaf = !subdivide(); // if subdivision fails: bad luck, you are still a leaf. Should never happen.
        return leaf;
    }

    bool Node::subdivide()
    {
        if (!isLeaf || (size.x < 2 && size.y < 2))
            return false;

        // unlike a proper quadtree, we can have a node that only has two children, rather than four. This can happen if we have a non-square cell
        // that reaches the minimum size on one of the dimensions

        // if it's like this          we divide it like this and consider these the *bottom* children (for array-indexing purposes)
        //   -------------              -------------
        //   |           |              |     |     |
        //   -------------              -------------

        // if it's like this          we divide it like this and consider these the *left* children (for array-indexing purposes)
        //  ----------                   ----------
        //  |        |                   |        |
        //  |        |                   |        |
        //  |        |                   |        |
        //  |        |                   |--------|
        //  |        |                   |        |
        //  |        |                   |        |
        //  |        |                   |        |
        //  ----------                   ----------

        // top left
        {
            Vector2Int newOrigin(origin.x, origin.y + std::ceil(size.y * 0.5));
            Vector2Int newSize(std::ceil(size.x * 0.5), size.y / 2);

            if (size.x == 1)
                newSize.x = 1;

            if (size.y == 1)
                children[0] = nullptr;
            else
            {
                children[0] = Node::createNode(quadtree, newOrigin, newSize, map);
                children[0]->parent = this;
            }
        }
        // top right
        {
            Vector2Int newOrigin(origin.x + std::ceil(size.x * 0.5), origin.y + std::ceil(size.y * 0.5));
            if (size.x == 1 || size.y == 1)
                children[1] = nullptr;
            else
            {
                children[1] = Node::createNode(quadtree, newOrigin, size / 2, map);
                children[1]->parent = this;
            }
        }
        // bottom left
        {
            Vector2Int newOrigin(origin.x, origin.y);
            Vector2Int newSize(std::ceil(size.x * 0.5), std::ceil(size.y * 0.5));

            if (size.y == 1)
                newSize.y = 1;

            if (size.x == 1)
                newSize.x = 1;

            children[2] = Node::createNode(quadtree, newOrigin, newSize, map);
            children[2]->parent = this;
        }
        // bottom right
        {
            Vector2Int newOrigin(origin.x + std::ceil(size.x * 0.5), origin.y);
            Vector2Int newSize(size.x / 2, std::ceil(size.y * 0.5));

            if (size.y == 1)
                newSize.y = 1;

            if (size.x == 1)
                children[3] = nullptr;
            else
            {
                children[3] = Node::createNode(quadtree, newOrigin, newSize, map);
                children[3]->parent = this;
            }
        }

        isLeaf = false;

        return true;
    }

    std::vector<Node> Quadtree::fusedLeaves(int maxSize)
    {
        std::list<Node> free_leaves; //**copies** of all the leaves that correspond to free parts of the map. Not actually part of the tree!

#if 1
        // use the existing leaves
        for (int i = 0; i < leaves.size(); i++)
        {
            auto locked = leaves[i].lock();
            if (locked->value == 1)
            {
                free_leaves.emplace_back(nullptr, locked->origin, locked->size, locked->map);
                free_leaves.back().value = 1;
            }
        }
#else

        // use the original map, cell by cell
        for (int i = 0; i < map.size(); i++)
        {
            for (int j = 0; j < map[0].size(); j++)
            {
                if (map[i][j] != 1)
                    continue;
                free_leaves.emplace_back(nullptr, Vector2Int{i, j}, Vector2Int{1, 1}, map);
                free_leaves.back().value = 1;
            }
        }
#endif

        std::vector<std::vector<Node*>> pointersImage(
            map.size(), std::vector<Node*>(map[0].size(), nullptr)); // for each cell in the original map, a pointer to the leaf that contains it

        // populate the pòinters image
        for (auto itr = free_leaves.begin(); itr != free_leaves.end(); itr++)
        {
            Node* leaf = &(*itr);
            Vector2Int start = leaf->origin;
            Vector2Int end = leaf->origin + leaf->size;
            #pragma omp parallel for collapse(2)
            for (int r = start.x; r < end.x; r++)
            {
                for (int c = start.y; c < end.y; c++)
                {
                    pointersImage[r][c] = leaf;
                }
            }
        }

        // register which nodes are neighbours
        std::unordered_map<Node*, std::unordered_set<Node*>> allNeighbours;
        allNeighbours.reserve(leaves.size());
        {
            auto checkAndAdd = [](Node* current, Node* neighbour, std::unordered_set<Node*>& neighboursSet) {
                if (current == nullptr || neighbour == nullptr)
                    return;
                if (neighbour != current)
                    neighboursSet.insert(neighbour);
            };
            for (int i = 1; i < pointersImage.size() - 1; i++)
            {
                for (int j = 1; j < pointersImage[0].size() - 1; j++)
                {
                    Node* current_val = pointersImage[i][j];
                    std::unordered_set<Node*>& my_neighbours = allNeighbours[current_val];

                    checkAndAdd(current_val, pointersImage[i - 1][j], my_neighbours);
                    checkAndAdd(current_val, pointersImage[i][j - 1], my_neighbours);

                    checkAndAdd(current_val, pointersImage[i + 1][j], my_neighbours);
                    checkAndAdd(current_val, pointersImage[i][j + 1], my_neighbours);
                }
            }
        }

        {
            // keep track of the ones that have been fused to remove them from the list at the end of the process
            // cant delete them half way through because that will invalidate all the pointers
            std::set<Node*> deletedNodes;
            std::unordered_set<Node*> to_be_evaluated;
            for (Node& node : free_leaves)
                to_be_evaluated.insert(&node);

            auto sizeFused = [](Node* a, Node* b, Vector2Int& outOrigin, Vector2Int& outSize) {
                int minX = std::min(a->origin.x, b->origin.x);
                int minY = std::min(a->origin.y, b->origin.y);

                int maxX = std::max(a->origin.x + a->size.x, b->origin.x + b->size.x);
                int maxY = std::max(a->origin.y + a->size.y, b->origin.y + b->size.y);
                outOrigin = {minX, minY};
                outSize = Vector2Int{maxX, maxY} - outOrigin;
            };

            auto fuse = [&free_leaves, &allNeighbours, &deletedNodes, &to_be_evaluated, this, sizeFused](Node* a, Node* b) {
                Vector2Int origin;
                Vector2Int size;
                sizeFused(a, b, origin, size);

                free_leaves.emplace_back(nullptr, origin, size, this->map);
                Node* fused = &free_leaves.back();
                fused->value = 1;

                // clean up
                for (Node* n : allNeighbours[a])
                {
                    if (n == b)
                        continue;
                    allNeighbours[n].erase(a);
                    allNeighbours[n].insert(fused);
                    allNeighbours[fused].insert(n);
                }
                for (Node* n : allNeighbours[b])
                {
                    if (n == a)
                        continue;
                    allNeighbours[n].erase(b);
                    allNeighbours[n].insert(fused);
                    allNeighbours[fused].insert(n);
                }
                deletedNodes.insert(a);
                deletedNodes.insert(b);

                to_be_evaluated.erase(a);
                to_be_evaluated.erase(b);
                to_be_evaluated.insert(fused);
            };

            // iterate over the existing nodes and see what can be fused
            while (to_be_evaluated.size() != 0)
            {
                Node* current = *to_be_evaluated.begin();

                Node* best = nullptr;
                float distanceFromSquare = FLT_MAX;

                for (Node* neighbour : allNeighbours[current])
                {
                    // if(can be fused)
                    if ((current->origin.x == neighbour->origin.x && current->size.x == neighbour->size.x &&
                         (current->size.y + neighbour->size.y) <= maxSize) ||
                        (current->origin.y == neighbour->origin.y && current->size.y == neighbour->size.y &&
                         (current->size.x + neighbour->size.x) <= maxSize))
                    {
                        Vector2Int origin;
                        Vector2Int size;
                        sizeFused(current, neighbour, origin, size);

                        float dist = std::abs((size.y / size.x) - 1);
                        if (dist < distanceFromSquare)
                        {
                            distanceFromSquare = dist;
                            best = neighbour;
                        }
                    }
                }
                if (best != nullptr)
                    fuse(current, best);

                to_be_evaluated.erase(current);
            }

            // delete the fused nodes
            auto itr = free_leaves.begin();
            while (itr != free_leaves.end())
            {
                if (deletedNodes.find(&(*itr)) != deletedNodes.end())
                {
                    itr = free_leaves.erase(itr);
                }
                else
                    itr++;
            }
        }

        return std::vector<Node>(free_leaves.begin(), free_leaves.end());
    }
} // namespace GSL::Utils::NQA