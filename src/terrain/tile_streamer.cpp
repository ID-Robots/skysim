// See tile_streamer.h. index.json is tile_cooker's line-oriented output; parsed with
// sscanf per line (we own both ends of the format — no JSON dependency).
#include "terrain/tile_streamer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <stdexcept>

namespace skysim::terrain {

namespace {

// 2D (N/E) distance from a point to a tile's AABB footprint; 0 inside.
double distance_to_tile(const core::Vec3 &pos, const TileInfo &tile) {
    const double dn = std::max({tile.aabb_min_ned[0] - pos[0], 0.0, pos[0] - tile.aabb_max_ned[0]});
    const double de = std::max({tile.aabb_min_ned[1] - pos[1], 0.0, pos[1] - tile.aabb_max_ned[1]});
    return std::sqrt(dn * dn + de * de);
}

} // namespace

TileStreamer::TileStreamer(StreamerConfig cfg) : cfg_(std::move(cfg)) {
    const auto index_path = cfg_.tile_dir / "index.json";
    std::ifstream in(index_path);
    if (!in) {
        throw std::runtime_error("TileStreamer: cannot open " + index_path.string());
    }
    std::string line;
    while (std::getline(in, line)) {
        TileInfo t;
        char file[256];
        if (std::sscanf(line.c_str(),
                        " {\"file\":\"%255[^\"]\",\"triangles\":%*u,"
                        "\"aabb_min_ned\":[%lf,%lf,%lf],\"aabb_max_ned\":[%lf,%lf,%lf]}",
                        file, &t.aabb_min_ned[0], &t.aabb_min_ned[1], &t.aabb_min_ned[2], &t.aabb_max_ned[0],
                        &t.aabb_max_ned[1], &t.aabb_max_ned[2]) == 7) {
            t.file = file;
            tiles_.push_back(std::move(t));
        }
    }
    if (tiles_.empty()) {
        throw std::runtime_error("TileStreamer: no tiles in " + index_path.string());
    }
    resident_.assign(tiles_.size(), false);
}

TileStreamer::Update TileStreamer::update(const std::vector<core::Vec3> &vehicle_positions_ned) {
    // Desired set: nearest max_resident tiles within keep_radius of ANY vehicle.
    std::vector<std::pair<double, size_t>> in_range;
    for (size_t i = 0; i < tiles_.size(); ++i) {
        double best = std::numeric_limits<double>::infinity();
        for (const auto &pos : vehicle_positions_ned) {
            best = std::min(best, distance_to_tile(pos, tiles_[i]));
        }
        if (best <= cfg_.keep_radius_m) {
            in_range.emplace_back(best, i);
        }
    }
    std::sort(in_range.begin(), in_range.end()); // (distance, index): deterministic
    if (in_range.size() > cfg_.max_resident) {
        in_range.resize(cfg_.max_resident);
    }

    std::vector<bool> desired(tiles_.size(), false);
    for (const auto &[dist, idx] : in_range) {
        desired[idx] = true;
    }

    Update out;
    for (size_t i = 0; i < tiles_.size(); ++i) {
        if (desired[i] && !resident_[i]) {
            out.add.push_back(i);
            resident_[i] = true;
            ++resident_total_;
        } else if (!desired[i] && resident_[i]) {
            out.remove.push_back(i);
            resident_[i] = false;
            --resident_total_;
        }
    }
    return out;
}

} // namespace skysim::terrain
