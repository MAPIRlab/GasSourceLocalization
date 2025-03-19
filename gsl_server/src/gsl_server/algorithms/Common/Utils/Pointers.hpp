#pragma once

#include <memory>

//----------------------------------
//----------------------------------
//Utility functions for casting/checking polymorphic types
//overloaded to work with raw, unique and shared pointers

//Usage:
    //Is<>:
    // BaseClass* ptr = new DerivedClassA();
    // if(Is<DerivedClassA>(ptr))
    //      ...
    // else if(Is<DerivedClassB>(ptr))

//------------------

    //As<>:
    // BaseClass* ptr = new DerivedClassA();
    // As<DerivedClassA>(ptr)->methodFromDerived();

//----------------------------------
//----------------------------------


// Raw
template <typename Subclass, typename Baseclass>
    requires(std::derived_from<Subclass, Baseclass>)
Subclass* As(Baseclass* p)
{
    return dynamic_cast<Subclass*>(p);
}

template <typename Subclass, typename Baseclass>
    requires(std::derived_from<Subclass, Baseclass>)
bool Is(Baseclass* p)
{
    return dynamic_cast<Subclass*>(p) != nullptr;
}

// Unique_ptr

template <typename Subclass, typename Baseclass>
    requires(std::derived_from<Subclass, Baseclass>)
Subclass* As(const std::unique_ptr<Baseclass>& p)
{
    return dynamic_cast<Subclass*>(p.get());
}

template <typename Subclass, typename Baseclass>
    requires(std::derived_from<Subclass, Baseclass>)
bool Is(const std::unique_ptr<Baseclass>& p)
{
    return dynamic_cast<Subclass*>(p.get()) != nullptr;
}


// Shared_ptr

template <typename Subclass, typename Baseclass>
    requires(std::derived_from<Subclass, Baseclass>)
Subclass* As(const std::shared_ptr<Baseclass>& p)
{
    return dynamic_cast<Subclass*>(p.get());
}

template <typename Subclass, typename Baseclass>
    requires(std::derived_from<Subclass, Baseclass>)
bool Is(const std::shared_ptr<Baseclass>& p)
{
    return dynamic_cast<Subclass*>(p.get()) != nullptr;
}