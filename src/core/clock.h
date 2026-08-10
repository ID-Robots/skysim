#pragma once
// Fixed-dt physics clock + time policy. See docs/DESIGN.md "Time policy".
// Invariant: no wall-clock reads in strict mode; pacing (interactive) lives here only.
#include <chrono>
#include <cstdint>

namespace skysim::core {

enum class TimeMode : uint8_t { kStrict, kInteractive };

class PhysicsClock {
public:
    explicit PhysicsClock(double dt_s) : dt_s_(dt_s) {}
    double dt() const { return dt_s_; }
    double now() const { return tick_ * dt_s_; }      // physics time, exact multiples of dt
    uint64_t tick_index() const { return tick_; }
    void advance() { ++tick_; }
private:
    double dt_s_;
    uint64_t tick_ = 0;
};

// TODO(M1/M4): TickGate — one barrier on the next frame of every *connected* vehicle, with
// the straggler policy injected: strict aborts on timeout, interactive adds deadline pacing
// and evicts (freeze, grace despawn). The barrier population is not the whole fleet: a slot
// that has never connected has nothing to wait for, and interactive freezing takes a vehicle
// out of the barrier rather than stepping it. A late vehicle that is still in the barrier
// must never be stepped without its frame in either mode — see run_interactive for why.

}  // namespace skysim::core
