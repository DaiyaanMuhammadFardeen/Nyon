#pragma once

#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <future>
#include <atomic>
#include <cassert>

namespace Nyon::Utils {

/**
 * @brief Thread pool for parallel task execution
 * 
 * Efficiently distributes physics tasks across all available CPU cores.
 * Uses work-stealing queue for load balancing.
 */
class ThreadPool {
public:
    explicit ThreadPool(size_t numThreads = 0);
    ~ThreadPool();

    // Non-copyable, non-movable
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    /**
     * @brief Submit a task to the thread pool
     * @param task Function to execute
     * @return Future to retrieve result
     */
    template<typename F, typename... Args>
    auto Submit(F&& f, Args&&... args) -> std::future<typename std::invoke_result<F, Args...>::type>;

    /**
     * @brief Wait for all tasks to complete
     * 
     * WARNING: Must NOT be called from a worker thread of this pool.
     * Calling WaitAll() from within a task submitted to this pool will cause a deadlock.
     */
    void WaitAll();

    /**
     * @brief Get number of worker threads
     */
    size_t GetThreadCount() const { return m_Workers.size(); }

    /**
     * @brief Get instance count of pending tasks
     */
    size_t GetPendingTaskCount() const;

    /**
     * @brief Get singleton instance
     */
    static ThreadPool& Instance();

    /**
     * @brief Initialize singleton with specified thread count
     */
    static void Initialize(size_t numThreads = 0);

    /**
     * @brief Shutdown singleton
     */
    static void Shutdown();

private:
    void WorkerThread();

    std::vector<std::thread> m_Workers;
    std::queue<std::function<void()>> m_Tasks;
    mutable std::mutex m_QueueMutex;
    std::condition_variable m_Condition;
    std::atomic<bool> m_Stop{false};
    std::atomic<size_t> m_ActiveTasks{0};
    std::condition_variable m_AllDoneCondition;

    static std::unique_ptr<ThreadPool> s_Instance;
    
    // Thread-local flag to detect if current thread is a worker thread
    // Used to prevent deadlock when WaitAll() is called from within a task
    inline static thread_local bool tls_IsWorkerThread = false;
};

// Template implementation must be in header
template<typename F, typename... Args>
auto ThreadPool::Submit(F&& f, Args&&... args) -> std::future<typename std::invoke_result<F, Args...>::type> {
    using ReturnType = typename std::invoke_result<F, Args...>::type;

    auto task = std::make_shared<std::packaged_task<ReturnType()>>(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...)
    );

    std::future<ReturnType> result = task->get_future();
    {
        std::lock_guard<std::mutex> lock(m_QueueMutex);
        if (m_Stop) {
            throw std::runtime_error("ThreadPool is stopped");
        }
        m_Tasks.emplace([task]() { (*task)(); });
        m_ActiveTasks++;
    }
    m_Condition.notify_one();
    return result;
}

} // namespace Nyon::Utils
