#pragma once
// Tick-time metrics: ring buffer of recent tick durations -> p50/p99 (DESIGN.md "Scaling
// notes": watch tick p99 < dt). Written by the tick thread, read via snapshot() copies.
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace skysim::core {

class TickMetrics {
  public:
    void record_us(double us) {
        samples_[head_] = us;
        head_ = (head_ + 1) % samples_.size();
        count_ = count_ < samples_.size() ? count_ + 1 : count_;
    }

    struct Snapshot {
        double p50_us = 0.0;
        double p99_us = 0.0;
        uint64_t straggler_events = 0;
        uint64_t freezes = 0;
    };

    Snapshot snapshot() const {
        Snapshot s;
        s.straggler_events = straggler_events;
        s.freezes = freezes;
        if (count_ == 0) {
            return s;
        }
        std::array<double, kWindow> sorted;
        std::copy_n(samples_.begin(), count_, sorted.begin());
        std::sort(sorted.begin(), sorted.begin() + count_);
        s.p50_us = sorted[count_ / 2];
        s.p99_us = sorted[std::min(count_ - 1, count_ * 99 / 100)];
        return s;
    }

    uint64_t straggler_events = 0; // deadline misses (held ticks)
    uint64_t freezes = 0;          // hold budget exhausted -> vehicle frozen

  private:
    static constexpr size_t kWindow = 4096;
    std::array<double, kWindow> samples_{};
    size_t head_ = 0;
    size_t count_ = 0;
};

} // namespace skysim::core
