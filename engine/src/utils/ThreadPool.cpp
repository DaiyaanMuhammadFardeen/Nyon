#include "nyon/utils/ThreadPool.h"
#include <iostream>
namespace Nyon::Utils {
std::unique_ptr<ThreadPool> ThreadPool::s_Instance = nullptr;
ThreadPool::ThreadPool(size_t numThreads) {
    if (numThreads == 0) {
        numThreads = std::thread::hardware_concurrency();
        if (numThreads == 0) {
            numThreads = 1;   } }
    std::cerr << "[ThreadPool] Initializing with " << numThreads << " threads\n";
    for (size_t i = 0; i < numThreads; ++i) {
        m_Workers.emplace_back(&ThreadPool::WorkerThread, this); } }
ThreadPool::~ThreadPool() { {
        std::lock_guard<std::mutex> lock(m_QueueMutex);
        m_Stop = true; }
    m_Condition.notify_all();
    for (auto& worker : m_Workers) {
        if (worker.joinable()) {
            worker.join(); } } }
void ThreadPool::WorkerThread() {
    tls_IsWorkerThread = true;
    while (true) {
        std::function<void()> task; {
            std::unique_lock<std::mutex> lock(m_QueueMutex);
            m_Condition.wait(lock, [this] {
                return m_Stop || !m_Tasks.empty(); });
            if (m_Stop && m_Tasks.empty()) {
                return; }
            task = std::move(m_Tasks.front());
            m_Tasks.pop(); }
        task();
        if (--m_ActiveTasks == 0) {
            m_AllDoneCondition.notify_all(); } } }
void ThreadPool::WaitAll() {
    assert(!tls_IsWorkerThread && "WaitAll() must not be called from a worker thread - will cause deadlock!");
    std::unique_lock<std::mutex> lock(m_QueueMutex);
    m_AllDoneCondition.wait(lock, [this] {
        return m_ActiveTasks == 0 && m_Tasks.empty(); }); }
size_t ThreadPool::GetPendingTaskCount() const {
    std::lock_guard<std::mutex> lock(m_QueueMutex);
    return m_Tasks.size(); }
ThreadPool& ThreadPool::Instance() {
    if (!s_Instance) {
        s_Instance = std::make_unique<ThreadPool>(); }
    return *s_Instance; }
void ThreadPool::Initialize(size_t numThreads) {
    if (!s_Instance) {
        s_Instance = std::make_unique<ThreadPool>(numThreads); } }
void ThreadPool::Shutdown() {
    s_Instance.reset(); } }  
