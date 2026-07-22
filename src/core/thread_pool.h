#pragma once
// Minimal fixed-size worker pool for the reply/IO path: chunk an index range [0,count) across
// workers and block until done. Used ONLY to fan out per-vehicle reply build+send, which is
// embarrassingly parallel (each index touches only its own slot + socket). The physics tick
// stays single-threaded (that is the fast, deterministic path — see core/world.cpp); this pool
// exists because the reply sendto() is a kernel-bound syscall that parallelizes across cores.
//
// Reuse-safe: persistent workers, no per-call thread creation. Not for the physics path.
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace skysim::core {

class ThreadPool {
  public:
    explicit ThreadPool(int workers) {
        const int n = workers > 1 ? workers : 1;
        for (int i = 0; i < n; ++i) {
            threads_.emplace_back([this, i, n] { worker_loop(i, n); });
        }
    }

    ~ThreadPool() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
            ++generation_;
        }
        cv_.notify_all();
        for (auto &t : threads_) {
            t.join();
        }
    }

    ThreadPool(const ThreadPool &) = delete;
    ThreadPool &operator=(const ThreadPool &) = delete;

    size_t size() const { return threads_.size(); }

    // Run fn(i) for i in [0,count), split into <=size() contiguous chunks, one per worker plus
    // the calling thread. Blocks until all chunks finish. Safe to call only from one thread.
    void parallel_for(size_t count, const std::function<void(size_t begin, size_t end)> &fn) {
        if (count == 0) {
            return;
        }
        const size_t nchunks = std::min(threads_.size() + 1, count);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            fn_ = &fn;
            count_ = count;
            chunks_ = nchunks;
            remaining_ = nchunks - 1; // workers handle chunks 1..nchunks-1; caller does chunk 0
            ++generation_;
        }
        cv_.notify_all();

        run_chunk(0, nchunks, count, fn); // caller runs the first chunk

        std::unique_lock<std::mutex> lock(done_mutex_);
        done_cv_.wait(lock, [this] { return remaining_.load(std::memory_order_acquire) == 0; });
    }

  private:
    static void run_chunk(size_t idx, size_t chunks, size_t count,
                          const std::function<void(size_t, size_t)> &fn) {
        const size_t begin = count * idx / chunks;
        const size_t end = count * (idx + 1) / chunks;
        if (begin < end) {
            fn(begin, end);
        }
    }

    void worker_loop(int worker_index, int /*n*/) {
        uint64_t seen = 0;
        while (true) {
            const std::function<void(size_t, size_t)> *fn;
            size_t count, chunks;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this, &seen] { return generation_ != seen; });
                seen = generation_;
                if (stop_) {
                    return;
                }
                fn = fn_;
                count = count_;
                chunks = chunks_;
            }
            // This worker owns chunk (worker_index + 1) if it exists.
            const size_t idx = static_cast<size_t>(worker_index) + 1;
            if (idx < chunks) {
                run_chunk(idx, chunks, count, *fn);
            }
            if (idx < chunks) {
                if (remaining_.fetch_sub(1, std::memory_order_acq_rel) == 1) {
                    std::lock_guard<std::mutex> lock(done_mutex_);
                    done_cv_.notify_one();
                }
            }
        }
    }

    std::vector<std::thread> threads_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool stop_ = false;
    uint64_t generation_ = 0;
    const std::function<void(size_t, size_t)> *fn_ = nullptr;
    size_t count_ = 0;
    size_t chunks_ = 0;

    std::atomic<size_t> remaining_{0};
    std::mutex done_mutex_;
    std::condition_variable done_cv_;
};

} // namespace skysim::core
