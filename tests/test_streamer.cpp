// M5: TileStreamer residency logic — proximity loading, hard max_resident memory bound,
// deterministic eviction — plus World add/remove_static_tile against a real cooked tile.
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "core/world.h"
#include "terrain/cook.h"
#include "terrain/tile_streamer.h"

namespace {

int g_failures = 0;

#define CHECK(cond)                                                                                          \
    do {                                                                                                     \
        if (!(cond)) {                                                                                       \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                      \
            ++g_failures;                                                                                    \
        }                                                                                                    \
    } while (0)

// 5x5 grid of 100 m tiles centered on the origin, index.json in tile_cooker's format.
void write_synthetic_index(const std::filesystem::path &dir) {
    std::ofstream f(dir / "index.json");
    f << "{\n  \"frame\": \"NED\",\n  \"tiles\": [\n";
    for (int i = -2; i <= 2; ++i) {
        for (int j = -2; j <= 2; ++j) {
            const double n0 = i * 100.0 - 50.0, n1 = i * 100.0 + 50.0;
            const double e0 = j * 100.0 - 50.0, e1 = j * 100.0 + 50.0;
            f << "    {\"file\":\"t_" << (i + 2) << "_" << (j + 2) << ".jshape\",\"triangles\":12,"
              << "\"aabb_min_ned\":[" << n0 << "," << e0 << ",-30.000],"
              << "\"aabb_max_ned\":[" << n1 << "," << e1 << ",-0.000]},\n";
        }
    }
    f << "  ]\n}\n";
}

} // namespace

int main() {
    using namespace skysim::terrain;
    const auto dir = std::filesystem::temp_directory_path() / "skysim_test_streamer";
    std::filesystem::create_directories(dir);
    write_synthetic_index(dir);

    // --- Bad config throws at startup only. ---
    {
        bool threw = false;
        try {
            TileStreamer bad({"/nonexistent/tiles", 100.0, 4});
        } catch (const std::exception &) {
            threw = true;
        }
        CHECK(threw);
    }

    // --- Proximity: only tiles within the radius become resident. ---
    {
        TileStreamer s({dir, 120.0, 64});
        CHECK(s.tiles().size() == 25);
        auto plan = s.update({{0.0, 0.0, -10.0}});
        // 120 m from the origin: own tile (d=0), 4 edge neighbours (d=50), 4 corner
        // neighbours (d~70.7), plus the 4 next-over edge tiles (d=150 > 120? no).
        CHECK(plan.add.size() == 9);
        CHECK(s.resident_count() == 9);
        CHECK(plan.remove.empty());
        // No movement => no churn.
        plan = s.update({{0.0, 0.0, -10.0}});
        CHECK(plan.add.empty() && plan.remove.empty());
        // Move two tiles over: residency follows, old tiles unload.
        plan = s.update({{200.0, 0.0, -10.0}});
        CHECK(!plan.add.empty() && !plan.remove.empty());
        CHECK(s.resident_count() <= 9);
        // No vehicles: everything unloads.
        plan = s.update({});
        CHECK(s.resident_count() == 0);
    }

    // --- Hard memory bound: never more than max_resident, nearest win. ---
    {
        TileStreamer s({dir, 100000.0, 4}); // radius covers everything; the bound must cap
        auto plan = s.update({{0.0, 0.0, -10.0}});
        CHECK(plan.add.size() == 4);
        CHECK(s.resident_count() == 4);
        CHECK(s.is_resident(12));               // center tile (index 2*5+2) is nearest, must be in
        for (int step = 0; step < 20; ++step) { // sweep across the map; bound always holds
            s.update({{step * 25.0, step * 10.0, -10.0}});
            CHECK(s.resident_count() <= 4);
        }
    }

    // --- Two far-apart vehicles: union of neighbourhoods, still bounded. ---
    {
        TileStreamer s({dir, 60.0, 64});
        s.update({{-200.0, -200.0, -10.0}, {200.0, 200.0, -10.0}});
        CHECK(s.resident_count() >= 2);
        CHECK(s.is_resident(0) && s.is_resident(24)); // both corner tiles resident
        CHECK(!s.is_resident(12));                    // center is near neither
    }

    // --- World integration: streamed tile collides, unload removes the collision. ---
    {
        const auto obj = dir / "box.obj";
        {
            std::ofstream f(obj); // 40x40x30 box at ENU origin (pretile format subset)
            f << "v -20 -20 0\nv 20 -20 0\nv 20 20 0\nv -20 20 0\n"
              << "v -20 -20 30\nv 20 -20 30\nv 20 20 30\nv -20 20 30\n";
            const int quads[6][4] = {{1, 2, 6, 5}, {2, 3, 7, 6}, {3, 4, 8, 7},
                                     {4, 1, 5, 8}, {5, 6, 7, 8}, {4, 3, 2, 1}};
            for (const auto &q : quads) {
                f << "f " << q[0] << " " << q[1] << " " << q[2] << "\n"
                  << "f " << q[0] << " " << q[2] << " " << q[3] << "\n";
            }
        }
        std::string err;
        CHECK(skysim::terrain::cook_obj_tile(obj, dir / "box.jshape", nullptr, &err));

        skysim::core::WorldConfig cfg;
        cfg.dt_s = 1.0 / 800.0;
        cfg.worker_threads = 2;
        skysim::core::World w(cfg);
        CHECK(w.raycast({0.0, 0.0, -100.0}, {0.0, 0.0, 1.0}, 200.0) < 0.0); // empty world
        const uint32_t tile = w.add_static_tile(dir / "box.jshape");
        CHECK(tile != 0);
        const double d = w.raycast({0.0, 0.0, -100.0}, {0.0, 0.0, 1.0}, 200.0);
        CHECK(std::abs(d - 70.0) < 0.01); // roof at z=-30
        w.remove_static_tile(tile);
        CHECK(w.raycast({0.0, 0.0, -100.0}, {0.0, 0.0, 1.0}, 200.0) < 0.0); // gone again
        CHECK(w.add_static_tile(dir / "missing.jshape") == 0);              // load failure -> 0
        w.remove_static_tile(9999);                                         // unknown id: no-op
    }

    if (g_failures == 0) {
        std::printf("test_streamer: all checks OK\n");
        return 0;
    }
    std::printf("test_streamer: %d failure(s)\n", g_failures);
    return 1;
}
