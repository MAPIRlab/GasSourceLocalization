#pragma once
#include <gsl_server/core/Logging.hpp>
#include <unordered_map>
#include <string>
#include <iterator>

namespace GSL
{
    class ClassDistribution
    {
    public:
        //custom iterator to remove any reference to the fact that we internally use an unordered_map
        struct Iterator;

        float ProbabilityOf(const std::string& _class)
        {
            return probabilityDist[_class];
        }

        void UpdateProbOf(const std::string& _class, float prob)
        {
            try
            {
                probabilityDist.at(_class) = prob;
            }
            catch(std::exception& e)
            {
                GSL_ERROR("Exception while updating probability of object class {}: {}", _class, e.what());
            }
        }

        void Normalize()
        {
            float sum = 0;
            for(const auto& [_class, prob] : probabilityDist)
                sum += prob;
            
            for(auto& [_class, prob] : probabilityDist)
                prob /= sum;
        }

        Iterator begin() {return Iterator(&*probabilityDist.begin());}
        Iterator end() {return Iterator(&*probabilityDist.end());}


    private:
        std::unordered_map<std::string, float> probabilityDist;







        //Iterator definition
    public:
        struct Iterator
        {
            using iterator_category = std::forward_iterator_tag;
            using difference_type   = std::ptrdiff_t;
            using value_type        = std::pair<const std::string, float>;
            using pointer           = value_type*;
            using reference         = value_type&;

            Iterator(pointer ptr) : m_ptr(ptr) {}

            reference operator*() const { return *m_ptr; }
            pointer operator->() { return m_ptr; }

            // Prefix increment
            Iterator& operator++() { m_ptr++; return *this; }  

            // Postfix increment
            Iterator operator++(int) { Iterator tmp = *this; ++(*this); return tmp; }

            friend bool operator== (const Iterator& a, const Iterator& b) { return a.m_ptr == b.m_ptr; };
            friend bool operator!= (const Iterator& a, const Iterator& b) { return a.m_ptr != b.m_ptr; };     
        private:
            pointer m_ptr;
        };
    };
} // namespace GSL