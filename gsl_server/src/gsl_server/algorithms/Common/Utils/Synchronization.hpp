#pragma once
#include <mutex>

namespace GSL::Utils
{
    template <typename T>
    class Synced
    {
    public:
        Synced() : obj() {}
        Synced(const T& value) : obj(value) {}

        T obj;
        std::mutex mtx;
    };

    template <typename T>
    class SyncedAccess
    {
    public:
        SyncedAccess(Synced<T>& synced) : obj(synced.obj), mtx(synced.mtx)
        {
            mtx.lock();
        }

        ~SyncedAccess()
        {
            mtx.unlock();
        }
        T& Get() { return obj; }

    private:
        T& obj;
        std::mutex& mtx;
    };
} // namespace GSL::Utils