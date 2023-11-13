#pragma once
#include <functional>
#include <vector>
#include <mutex>

namespace GSL
{
    // A list of functions to be executed later, all in one go. Can be used to delay a synchronous service call rather than do it inside
    // of a spinning callback, or to trigger functionality from UI. Thread-safe.
    class FunctionQueue
    {
    public:
        void submit(const std::function<void()>& func)
        {
            mutex.lock();
            queue.push_back(func);
            mutex.unlock();
        }

        void run()
        {
            mutex.lock();
            auto queueCopy = queue;
            queue.clear();
            mutex.unlock();

            for (auto& func : queueCopy)
                func();
        }

    private:
        std::mutex mutex;
        std::vector<std::function<void()>> queue;
    };
} // namespace GSL