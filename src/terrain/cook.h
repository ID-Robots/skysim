#pragma once
// OBJ tile -> Jolt MeshShape -> SaveBinaryState (.jshape). Used by the tile_cooker CLI and
// by tests. Input OBJ is map-frame ENU (x east, y north, z up) with CCW-outward winding;
// vertices are converted to Jolt axes here (ENU -> Jolt is the cyclic map (y,z,x), a proper
// rotation, so winding/outwardness is preserved). See docs/DESIGN.md "Terrain".
#include <array>
#include <filesystem>
#include <string>

namespace skysim::terrain {

struct CookResult {
    size_t triangles = 0;
    // Tile AABB in world NED (for index.json / streamer bookkeeping).
    std::array<double, 3> aabb_min_ned{};
    std::array<double, 3> aabb_max_ned{};
};

// Returns false and fills *error on failure. No exceptions cross this boundary.
bool cook_obj_tile(const std::filesystem::path &obj_in, const std::filesystem::path &jshape_out,
                   CookResult *result, std::string *error);

} // namespace skysim::terrain
