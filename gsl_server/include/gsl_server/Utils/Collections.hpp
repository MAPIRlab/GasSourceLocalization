#pragma once
#include <c++/11/bits/stl_algo.h> 

namespace GSL::Utils
{
    template<typename Collection>
    size_t indexOfMax(Collection& collection)
    {
        return std::distance( collection.begin(), std::max_element(collection.begin(), collection.end()));
    }
}