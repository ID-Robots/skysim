#pragma once
// Streams cooked Jolt MeshShape tiles (.jshape + index.json) in/out around active vehicles.
// Pure residency logic — the caller (tick thread) applies add/remove against the World at
// tick boundaries only. See docs/DESIGN.md "Terrain".
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "core/frames.h"

namespace skysim::terrain {

struct StreamerConfig {
    std::filesystem::path tile_dir;
    double keep_radius_m = 600.0; // tiles resident within this of any vehicle (2D N/E)
    size_t max_resident = 64;     // hard memory bound (M5 acceptance asserts this)
};

struct TileInfo {
    std::string file; // .jshape filename inside tile_dir
    core::Vec3 aabb_min_ned{};
    core::Vec3 aabb_max_ned{};
};

class TileStreamer {
  public:
    // Parses <tile_dir>/index.json (tile_cooker output). Throws on a missing/bad index
    // (startup only; no exceptions in the tick path).
    explicit TileStreamer(StreamerConfig cfg);

    struct Update {
        std::vector<size_t> add;    // tile indices to load
        std::vector<size_t> remove; // tile indices to unload
    };

    // Desired-set diff for the current vehicle positions: the max_resident nearest tiles
    // within keep_radius (deterministic order: distance, then index). Marks the plan as
    // applied — call once per streaming interval from the tick thread.
    Update update(const std::vector<core::Vec3> &vehicle_positions_ned);

    const std::vector<TileInfo> &tiles() const { return tiles_; }
    size_t resident_count() const { return resident_total_; }
    bool is_resident(size_t idx) const { return resident_[idx]; }
    std::filesystem::path tile_path(size_t idx) const { return cfg_.tile_dir / tiles_[idx].file; }

  private:
    StreamerConfig cfg_;
    std::vector<TileInfo> tiles_;
    std::vector<bool> resident_;
    size_t resident_total_ = 0;
};

} // namespace skysim::terrain
