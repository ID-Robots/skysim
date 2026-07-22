// ThreadPool: every index in [0,count) is visited exactly once, chunks are contiguous and
// disjoint, and the pool is reusable across many calls and counts.
#include <atomic>
#include <cstdio>
#include <mutex>
#include <vector>

#include "core/thread_pool.h"

namespace {
int g_failures = 0;
#define CHECK(cond)                                                                                          \
    do {                                                                                                     \
        if (!(cond)) {                                                                                       \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                      \
            ++g_failures;                                                                                    \
        }                                                                                                    \
    } while (0)
} // namespace

int main() {
    skysim::core::ThreadPool pool(4);
    CHECK(pool.size() == 4);

    // Coverage: each index visited exactly once, across a range of counts (incl. < workers).
    for (size_t count :
         {size_t(0), size_t(1), size_t(3), size_t(4), size_t(5), size_t(100), size_t(1000), size_t(9973)}) {
        std::vector<std::atomic<int>> visits(count == 0 ? 1 : count);
        for (auto &v : visits) {
            v.store(0);
        }
        std::atomic<size_t> sum_ranges{0};
        pool.parallel_for(count, [&](size_t begin, size_t end) {
            CHECK(begin < end); // never called with an empty chunk
            sum_ranges.fetch_add(end - begin);
            for (size_t i = begin; i < end; ++i) {
                visits[i].fetch_add(1);
            }
        });
        CHECK(sum_ranges.load() == count);
        for (size_t i = 0; i < count; ++i) {
            CHECK(visits[i].load() == 1);
        }
    }

    // Reuse under load: accumulate a known sum in parallel many times.
    for (int iter = 0; iter < 500; ++iter) {
        const size_t n = 777;
        std::atomic<long> total{0};
        pool.parallel_for(n, [&](size_t begin, size_t end) {
            long local = 0;
            for (size_t i = begin; i < end; ++i) {
                local += static_cast<long>(i);
            }
            total.fetch_add(local);
        });
        CHECK(total.load() == static_cast<long>(n * (n - 1) / 2));
    }

    // Single-worker pool still runs everything (on the calling thread).
    {
        skysim::core::ThreadPool solo(1);
        std::atomic<int> hits{0};
        solo.parallel_for(50, [&](size_t b, size_t e) { hits.fetch_add(static_cast<int>(e - b)); });
        CHECK(hits.load() == 50);
    }

    if (g_failures == 0) {
        std::printf("test_thread_pool: all checks OK\n");
        return 0;
    }
    std::printf("test_thread_pool: %d failure(s)\n", g_failures);
    return 1;
}
