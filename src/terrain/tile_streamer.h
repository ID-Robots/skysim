#pragma once
// Streams cooked Jolt MeshShape tiles (.jshape + index.json) in/out around active vehicles.
// Static body add/remove happens only at tick boundaries. See docs/DESIGN.md "Terrain".
#include <cstdint>
#include <filesystem>

namespace skysim::terrain {

struct StreamerConfig {
    std::filesystem::path tile_dir;
    double keep_radius_m = 600.0;   // tiles resident within this of any vehicle
    size_t max_resident = 64;       // hard memory bound (M5 acceptance asserts this)
};

// TODO(M5): class TileStreamer { update(vehicle_aabbs) -> {to_add, to_remove}; }
// Cooked format: Jolt SaveBinaryState blobs; index.json carries tile AABBs + ENU/WGS84 anchor.

}  // namespace skysim::terrain
