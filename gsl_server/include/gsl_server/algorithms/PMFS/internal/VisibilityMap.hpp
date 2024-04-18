#pragma once
#include <unordered_map>
#include <unordered_set>
#include <gsl_server/core/Vectors.hpp>

namespace GSL
{
    enum class Visibility{Visible, NotVisible, OutOfRange};
    class VisibilityMap
    {
    public:
        size_t range = 0;

        Visibility isVisible(const Vector2Int& from, const Vector2Int& to) const
        {
            if(std::abs(from.x-to.x) > range || std::abs(from.y-to.y) > range)
                return Visibility::OutOfRange;

            const std::unordered_set<Vector2Int>& set = map.at(from);
            if (set.find(to) == set.end())
                return Visibility::NotVisible;
            return Visibility::Visible;
        }

        void emplace(const Vector2Int& key, const std::unordered_set<Vector2Int>& value)
        {
            map.emplace(key, value);
        }

        std::unordered_set<Vector2Int>& at(const Vector2Int& key)
        {
            return map.at(key);
        }

    private:
        std::unordered_map<Vector2Int, std::unordered_set<Vector2Int>> map;
    };
}