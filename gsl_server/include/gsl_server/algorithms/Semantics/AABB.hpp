#pragma once
#include <gsl_server/core/Vectors.hpp>

namespace GSL
{
    struct AABB2D
    {
        Vector2 min;
        Vector2 max;
    };

    struct AABB3D
    {
        Vector3 min;
        Vector3 max;
    };


    //AABB expressed as the minimum and maximum indices. Implements iterator to traverse the box, x first, y later
    struct AABB2DInt
    {
        Vector2Int min;
        Vector2Int max;

        AABB2DInt(){}
        AABB2DInt(const Vector2Int& _min, const Vector2Int& _max):min(_min), max(_max)
        {}

        bool operator==(const AABB2DInt& other)
        {
            return min == other.min && max == other.max;
        }


        class Iterator
        {
        public:
            Iterator(AABB2DInt& _aabb, Vector2Int start): aabb(_aabb), current(start){}
            Iterator(AABB2DInt& _aabb): aabb(_aabb), current(_aabb.min){}
            
            Vector2Int operator*() const { return current; }

            // Prefix increment
            Iterator& operator++()
            {
                current.x++;
                if(current.x>aabb.max.x)
                {
                    current.x = 0;
                    current.y++;
                }
                return *this;
            }

            // Postfix increment
            Iterator operator++(int) { Iterator tmp = *this; ++(*this); return tmp; }

            friend bool operator== (const Iterator& a, const Iterator& b) { return a.aabb == b.aabb && a.current == b.current; };
            friend bool operator!= (const Iterator& a, const Iterator& b) { return a.aabb != b.aabb && a.current != b.current; };
        private:
            AABB2DInt& aabb;
            Vector2Int current;
        };

        Iterator begin()
        {
            return Iterator(*this, min);
        }

        Iterator end()
        {
            return ++Iterator(*this, max);
        }
    };
}