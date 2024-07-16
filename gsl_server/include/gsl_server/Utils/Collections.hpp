#pragma once
#include <iterator>

namespace GSL::Utils
{
    template<typename C>
    inline size_t indexOfMax(C& collection)
    {
        return std::distance( collection.begin(), std::max_element(collection.begin(), collection.end()));
    }
}