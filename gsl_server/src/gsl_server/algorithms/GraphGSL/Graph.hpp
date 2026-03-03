#pragma once
#include <filesystem>
#include <vector>

namespace GSL
{
    class Graph
    {
    public:
        static Graph ReadFromDisk(const std::filesystem::path& path);

    public:
        std::vector<std::shared_ptr<class Node>> nodes;
    };
} // namespace GSL