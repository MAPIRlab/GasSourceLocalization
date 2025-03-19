#pragma once
#include <algorithm>
#include <iterator>

namespace GSL::Utils
{
    template<typename C>
    inline size_t indexOfMax(C& collection)
    {
        return std::distance( collection.begin(), std::max_element(collection.begin(), collection.end()));
    }


    template<typename C, typename Predicate>
    inline bool containsPred(const C& _collection, const Predicate& predicate)
    {
        for (const auto& t : _collection)
        {
            if (predicate(t))
                return true;
        }
        return false;
    }

    template<typename C, typename T>
    inline bool contains(const C& _collection, const T& element)
    {
        return _collection.find(element) != _collection.end();
    }

}