// tile_cooker: OBJ tiles (map-frame ENU, outward winding) -> Jolt MeshShape .jshape files
// + index.json (AABBs in NED). Pre-step for demo/scanned-city maps: tools/cooker/pretile.py.
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include "terrain/cook.h"

int main(int argc, char **argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: tile_cooker <out_dir> <tile1.obj> [tile2.obj ...]\n");
        return 2;
    }
    const std::filesystem::path out_dir = argv[1];
    std::filesystem::create_directories(out_dir);

    std::string index; // one JSON object per tile entry, assembled by hand (no JSON dep)
    size_t total_tris = 0;
    for (int i = 2; i < argc; ++i) {
        const std::filesystem::path obj = argv[i];
        const std::filesystem::path out = out_dir / (obj.stem().string() + ".jshape");
        skysim::terrain::CookResult res;
        std::string err;
        if (!skysim::terrain::cook_obj_tile(obj, out, &res, &err)) {
            std::fprintf(stderr, "tile_cooker: %s\n", err.c_str());
            return 1;
        }
        char entry[512];
        std::snprintf(entry, sizeof(entry),
                      "    {\"file\":\"%s\",\"triangles\":%zu,"
                      "\"aabb_min_ned\":[%.3f,%.3f,%.3f],\"aabb_max_ned\":[%.3f,%.3f,%.3f]}",
                      out.filename().c_str(), res.triangles, res.aabb_min_ned[0], res.aabb_min_ned[1],
                      res.aabb_min_ned[2], res.aabb_max_ned[0], res.aabb_max_ned[1], res.aabb_max_ned[2]);
        index += (index.empty() ? std::string() : std::string(",\n")) + entry;
        total_tris += res.triangles;
        std::printf("cooked %s -> %s (%zu tris)\n", obj.c_str(), out.c_str(), res.triangles);
    }

    FILE *f = std::fopen((out_dir / "index.json").c_str(), "w");
    if (f == nullptr) {
        std::fprintf(stderr, "tile_cooker: cannot write index.json\n");
        return 1;
    }
    std::fprintf(f, "{\n  \"frame\": \"NED\",\n  \"tiles\": [\n%s\n  ]\n}\n", index.c_str());
    std::fclose(f);
    std::printf("tile_cooker: %zu tile(s), %zu triangles total\n", static_cast<size_t>(argc - 2), total_tris);
    return 0;
}
