#pragma once
// Jolt world wrapper. The ONLY file allowed to see Jolt's Y-up axes; everything above
// speaks NED (see CLAUDE.md invariant 2). Init order rules: CLAUDE.md "Jolt-specific".
#include <cstdint>
#include <memory>

namespace skysim::core {

struct WorldConfig {
    double dt_s = 1.0 / 400.0;
    uint32_t max_bodies = 4096;
    int worker_threads = -1;  // -1 => hardware_concurrency - 1
    uint64_t rng_seed = 1;    // all randomness derives from this (determinism invariant)
};

class World {
public:
    explicit World(const WorldConfig& cfg);
    ~World();
    // TODO(M2): body create/destroy (tick-boundary only), apply_forces batch, step(),
    //           publish_snapshot(); NED<->Jolt conversion helpers live here.
    // TODO(M5): add_static_tile(shape_blob) / remove_static_tile(id).
private:
    struct Impl;                 // keeps Jolt headers out of the rest of the codebase
    std::unique_ptr<Impl> impl_;
};

}  // namespace skysim::core
