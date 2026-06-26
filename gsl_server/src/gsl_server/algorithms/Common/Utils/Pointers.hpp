#pragma once

#include <memory>

//----------------------------------
//----------------------------------
//Utility functions for casting/checking polymorphic types
//overloaded to work with raw, unique and shared pointers

//Usage:
    //Is<>:
    // FirstClass* ptr = new DerivedClassA();
    // if(Is<DerivedClassA>(ptr))
    //      ...
    // else if(Is<DerivedClassB>(ptr))

//------------------

    //As<>:
    // FirstClass* ptr = new DerivedClassA();
    // As<DerivedClassA>(ptr)->methodFromDerived();

//----------------------------------
//----------------------------------


// Raw
template <typename SecondClass, typename FirstClass>
    requires(std::derived_from<SecondClass, FirstClass> || std::derived_from<FirstClass, SecondClass>)
SecondClass* As(FirstClass* p)
{
    return dynamic_cast<SecondClass*>(p);
}

template <typename SecondClass, typename FirstClass>
    requires(std::derived_from<SecondClass, FirstClass>)
bool Is(FirstClass* p)
{
    return dynamic_cast<SecondClass*>(p) != nullptr;
}

// Unique_ptr

template <typename SecondClass, typename FirstClass>
    requires(std::derived_from<SecondClass, FirstClass> || std::derived_from<FirstClass, SecondClass>)
SecondClass* As(const std::unique_ptr<FirstClass>& p)
{
    return dynamic_cast<SecondClass*>(p.get());
}

template <typename SecondClass, typename FirstClass>
    requires(std::derived_from<SecondClass, FirstClass>)
bool Is(const std::unique_ptr<FirstClass>& p)
{
    return dynamic_cast<SecondClass*>(p.get()) != nullptr;
}


// Shared_ptr

template <typename SecondClass, typename FirstClass>
    requires(std::derived_from<SecondClass, FirstClass> || std::derived_from<FirstClass, SecondClass>)
std::shared_ptr<SecondClass> As(const std::shared_ptr<FirstClass>& p)
{
    return std::dynamic_pointer_cast<SecondClass>(p);
}

template <typename SecondClass, typename FirstClass>
    requires(std::derived_from<SecondClass, FirstClass>)
bool Is(const std::shared_ptr<FirstClass>& p)
{
    return dynamic_cast<SecondClass*>(p.get()) != nullptr;
}

template <typename SecondClass, typename FirstClass>
    requires(std::derived_from<SecondClass, FirstClass>)
bool Is(const std::weak_ptr<FirstClass>& p)
{
    return Is<SecondClass>(p.lock());
}