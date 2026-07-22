#pragma once

#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <rclcpp/utilities.hpp> // only for rclcpp::ok(). Can easily be removed if not using ROS
#include <thread>
#include <vector>

class ThreadPool
{
public:
    explicit ThreadPool(size_t numThreads = std::thread::hardware_concurrency());
    ~ThreadPool();

    void QueueJob(const std::function<void()>& job);
    void Stop();   // does not stop jobs which aleady started, but prevents new jobs from being started and empties the queue
    bool IsBusy(); // returns true if any job is running or queued
    void Wait();

private:
    std::vector<std::jthread> workers;
    size_t workersBusy;
    std::deque<std::function<void()>> jobs;
    std::mutex jobsMutex;
    std::condition_variable condition;

    bool destructing = false;

    void WorkerLoop(size_t threadId);
};

// ------------------
// Implementation
// ------------------

inline ThreadPool::ThreadPool(size_t numThreads)
    : workers(numThreads), workersBusy(0)
{
    for (size_t i = 0; i < workers.size(); ++i)
    {
        workers[i] = std::jthread(std::bind(&ThreadPool::WorkerLoop, this, i));
    }
}

inline ThreadPool::~ThreadPool()
{
    destructing = true;
    condition.notify_all();
}

inline void ThreadPool::QueueJob(const std::function<void()>& job)
{
    {
        std::unique_lock<std::mutex> lock(jobsMutex);
        jobs.emplace_back(job);
    }
    condition.notify_one();
}

inline void ThreadPool::Stop()
{
    {
        std::unique_lock<std::mutex> lock(jobsMutex);
        jobs.clear();
    }
    condition.notify_all();
}

inline bool ThreadPool::IsBusy()
{
    bool poolbusy;
    {
        std::unique_lock<std::mutex> lock(jobsMutex);
        poolbusy = !jobs.empty() || workersBusy > 0;
    }
    return poolbusy;
}

inline void ThreadPool::Wait()
{
    while (IsBusy())
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
}

inline void ThreadPool::WorkerLoop(size_t threadId)
{
    while (!destructing && rclcpp::ok())
    {
        std::optional<std::function<void()>> job;
        {
            std::unique_lock<std::mutex> lock(jobsMutex);
            condition.wait(lock, [this]()
                           { return destructing || !jobs.empty(); });

            if (destructing)
                return;

            if (jobs.empty())
                continue;
            job = std::move(jobs.front());
            jobs.pop_front();
            workersBusy++;
        }

        job.value()();
        {
            std::unique_lock<std::mutex> lock(jobsMutex);
            workersBusy--;
        }
    }
}