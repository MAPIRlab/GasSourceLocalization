#pragma once
#include <gsl_server/core/Vectors.hpp>
#include <gsl_server/core/Logging.hpp>
#include <vector>
#include <gsl_server/Utils/Profiling.hpp>

namespace GSL
{

    template <class Iter> 
    class Range
    {
        Iter b;
        Iter e;

    public:
        Range(Iter b, Iter e) : b(b), e(e)
        {}

        Iter begin() const
        {
            return b;
        }
        Iter end() const
        {
            return e;
        }

        static Range<Iter> make_range(Iter beginIter, size_t startIndex, size_t endIndex)
        {
            return Range<Iter>(beginIter + startIndex, beginIter + endIndex);
        }
    };

    enum class Visibility
    {
        Visible,
        NotVisible,
        OutOfRange
    };
    class VisibilityMap
    {
    public:
        VisibilityMap() = delete;
        VisibilityMap(size_t mapWidth, size_t height, size_t _range)
            : range(_range), bucketSize(std::pow(2 * _range, 2)), 
              m_width(mapWidth*bucketSize),
              m_map(mapWidth * height * bucketSize, Vector2Int{-1,-1})
        {}

        const size_t range;

        Visibility isVisible(const Vector2Int& from, const Vector2Int& to) const
        {
            if (std::abs(from.x - to.x) > range || std::abs(from.y - to.y) > range)
                return Visibility::OutOfRange;

            auto set = at_c(from);
            //if (!find(set, to))
            if (std::find(set.begin(), set.end(), to) == set.end())
                return Visibility::NotVisible;
            return Visibility::Visible;
        }

        void emplace(const Vector2Int& key, const std::vector<Vector2Int>& value)
        {
            for (size_t i = 0; i < value.size(); i++)
            {
                size_t index = indexOf(key) + i;
                m_map[index] = value[i];
            }
        }

        Range<std::vector<Vector2Int>::iterator> at(const Vector2Int& key)
        {
            size_t beginIndex = indexOf(key);
            return Range<std::vector<Vector2Int>::iterator>::make_range(m_map.begin(), beginIndex, beginIndex + bucketSize);
        }

        const Range<std::vector<Vector2Int>::const_iterator> at_c(const Vector2Int& key) const
        {
            size_t beginIndex = indexOf(key);
            return Range<std::vector<Vector2Int>::const_iterator>::make_range(m_map.begin(), beginIndex, beginIndex + bucketSize);
        }


    private:
        const size_t bucketSize;
        const size_t m_width;
        std::vector<Vector2Int> m_map;

        size_t indexOf(const Vector2Int& key) const
        {
            return key.x * m_width + key.y * bucketSize;
        }

        bool find(const Range<std::vector<Vector2Int>::const_iterator>& range, const Vector2Int& value) const
        {
            for(const Vector2Int& v : range)
            {
                if(v == value)
                    return true;
                else if (v == Vector2Int{-1,-1})
                    return false;
            }
            return false;
        }
    };
} // namespace GSL