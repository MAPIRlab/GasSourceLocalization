#pragma once

#include <memory>


template <typename Subclass, typename Baseclass>
Subclass* As(Baseclass* p)
{
    return dynamic_cast<Subclass*>(p);
}

template <typename Subclass, typename Baseclass>
bool Is(Baseclass* p)
{
    return dynamic_cast<Subclass*>(p) != nullptr;
}

// Unique_ptr

template <typename Subclass, typename Baseclass>
Subclass* As(const std::unique_ptr<Baseclass>& p)
{
    return dynamic_cast<Subclass*>(p.get());
}

template <typename Subclass, typename Baseclass>
bool Is(const std::unique_ptr<Baseclass>& p)
{
    return dynamic_cast<Subclass*>(p.get()) != nullptr;
}