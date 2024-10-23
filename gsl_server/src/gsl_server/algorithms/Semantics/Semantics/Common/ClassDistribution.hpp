#pragma once
#include <gsl_server/core/Logging.hpp>
#include <iterator>
#include <map>
#include <string>

namespace GSL
{
    class ClassDistribution
    {
    public:
        // custom iterator to remove any reference to the fact that we internally use a std::map
        struct Iterator;

        void Initialize(const std::vector<std::string>& classes)
        {
            if (initialized)
                return;

            float p = 1.0 / classes.size();
            for (const std::string& _class : classes)
                probabilityDist[_class] = p;
            initialized = true;
        }

        float ProbabilityOf(const std::string& _class) const
        {
            try
            {
                return probabilityDist.at(_class);
            }
            catch (std::exception& e)
            {
                GSL_ERROR("Exception while retrieving probability of object class {}: {}", _class, e.what());
            }
            return 0;
        }

        void SetProbOf(const std::string& _class, float prob)
        {
            try
            {
                probabilityDist.at(_class) = prob;
            }
            catch (std::exception& e)
            {
                GSL_ERROR("Exception while updating probability of object class {}: {}", _class, e.what());
            }
        }

        float TotalProb()
        {
            float p = 0;
            for (const auto& [name, prob] : probabilityDist)
                p += prob;
            return p;
        }

        void Normalize()
        {
            float sum = 0;
            for (const auto& [_class, prob] : probabilityDist)
                sum += prob;

            for (auto& [_class, prob] : probabilityDist)
                prob /= sum;
        }

        void Clear()
        {
            for (auto& [_class, prob] : probabilityDist)
                prob = 0;
        }

        Iterator begin()
        {
            return Iterator(probabilityDist.begin());
        }
        Iterator end()
        {
            return Iterator(probabilityDist.end());
        }

    private:
        std::map<std::string, float> probabilityDist;
        bool initialized = false;

        // Iterator definition
    public:
        struct Iterator
        {
            using iterator_category = std::forward_iterator_tag;
            using difference_type = std::ptrdiff_t;
            using value_type = std::pair<const std::string, float>;
            using pointer = value_type*;
            using reference = value_type&;
            using underlying_it = std::map<std::string, float>::iterator;

            Iterator(underlying_it it)
                : m_iter(it)
            {}

            reference operator*() const
            {
                return *m_iter;
            }
            pointer operator->()
            {
                return &*m_iter;
            }

            // Prefix increment
            Iterator& operator++()
            {
                m_iter++;
                return *this;
            }

            // Postfix increment
            Iterator operator++(int)
            {
                Iterator tmp = *this;
                ++(*this);
                return tmp;
            }

            friend bool operator==(const Iterator& a, const Iterator& b)
            {
                return a.get_pointer() == b.get_pointer();
            };
            friend bool operator!=(const Iterator& a, const Iterator& b)
            {
                return a.get_pointer() != b.get_pointer();
            };

        private:
            underlying_it m_iter;

            pointer get_pointer() const
            {
                return &*m_iter;
            }
        };
    };
} // namespace GSL