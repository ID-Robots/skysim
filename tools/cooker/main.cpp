// tile_cooker: OBJ tiles (map-frame ENU, outward winding) -> Jolt MeshShape .jshape files
// + index.json (AABBs in NED). Pre-step for demo/scanned-city maps: tools/cooker/pretile.py.
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

#include "terrain/cook.h"

int main(int argc, char **argv) {
    if (argc < 3) {
        std::fprintf(stderr,
                     "usage: tile_cooker <out_dir> [--anchor lat,lon,alt] <tile1.obj> [tile2.obj ...]\n");
        return 2;
    }
    const std::filesystem::path out_dir = argv[1];

    // Optional WGS84 anchor for the tile set. Without it a cooked map is bare ENU
    // metres with no way to check it against the vehicle's home point, and the
    // buildings can silently sit somewhere other than where the operator sees
    // them on the map (DESIGN.md asks for the anchor to be recorded).
    int first_obj = 2;
    bool have_anchor = false;
    double anchor[3] = {0.0, 0.0, 0.0};
    if (argc > 3 && std::string(argv[2]) == "--anchor") {
        if (std::sscanf(argv[3], "%lf,%lf,%lf", &anchor[0], &anchor[1], &anchor[2]) < 2) {
            std::fprintf(stderr, "tile_cooker: --anchor expects lat,lon[,alt]\n");
            return 2;
        }
        have_anchor = true;
        first_obj = 4;
    }
    if (first_obj >= argc) {
        std::fprintf(stderr, "tile_cooker: no input tiles\n");
        return 2;
    }
    std::filesystem::create_directories(out_dir);
    // Remove products of previous cooks: a stale .jshape from a removed input would be
    // silently loaded by the sim as phantom geometry.
    for (const auto &entry : std::filesystem::directory_iterator(out_dir)) {
        if (entry.path().extension() == ".jshape" || entry.path().filename() == "index.json") {
            std::filesystem::remove(entry.path());
        }
    }

    std::string index; // one JSON object per tile entry, assembled by hand (no JSON dep)
    size_t total_tris = 0;
    for (int i = first_obj; i < argc; ++i) {
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
    std::fprintf(f, "{\n  \"frame\": \"NED\",\n");
    if (have_anchor) {
        std::fprintf(f,
                     "  \"origin_wgs84\": [%.7f,%.7f,%.3f],\n"
                     "  \"crs\": \"ENU metres about origin_wgs84\",\n",
                     anchor[0], anchor[1], anchor[2]);
    }
    std::fprintf(f, "  \"tiles\": [\n%s\n  ]\n}\n", index.c_str());
    std::fclose(f);
    std::printf("tile_cooker: %zu tile(s), %zu triangles total\n", static_cast<size_t>(argc - first_obj),
                total_tris);
    return 0;
}
