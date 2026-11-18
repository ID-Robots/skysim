#pragma once
// The lit ground, remembered (TASK-2661).
//
// Measured on a 1920x1080 frame looking straight down from 30 m (tools/render_probe), three
// quarters of the time went into one call per pixel: the procedural ground texture
// (WorldTexture::ground), up to twenty value-noise lookups for grass, gravel, kerbs and shrubs.
// Another eighth went into the shadow ray. Both are pure functions of the ground point and the
// pixel's footprint, the sun never moves, and the ground a camera sees from one frame to the
// next is almost all the same ground. So this computes the lit ground colour once per texel of a
// world-space mip pyramid and every frame after that reads it back: bilinear within a level,
// blended between the two levels around the pixel's footprint (trilinear, as a GPU samples a
// mip-mapped texture).
//
// Level e has texels 2^e metres across, computed as WorldTexture::ground computes a pixel that
// wide, so each level is already band-limited to its texel size and sampling it does not alias.
// The picture differs from the direct evaluation the way a texture-mapped one does: a little
// softer in the finest octave, never in where things are. The survey still camera keeps the
// direct evaluation (photogrammetry wants the exact texture); this is for the pilot camera.
//
// Texels are filled in tiles, on whichever render thread first needs one, and shared by every
// vehicle's camera: the ground is the world's, not a vehicle's. Memory is bounded: past the
// budget, tiles no frame has used for longest go first (evict(), between frames).
//
// Of the two levels a pixel blends, the one weighted more is always read (built if need be).
// The other is filled a frame's budget of tiles at a time (begin_frame), and until it is the pixel
// reads the heavier level alone: a camera that descends needs the next finer level everywhere at
// once each time its height halves, and building all of it in one frame stalled the stream for
// most of a second. Read alone, the heavier level is the same ground a little softer, and by the
// time it carries most of the weight it has long been filled.
#include "core/thread_pool.h"
#include "render/scene.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace skysim::render {

class GroundCache {
  public:
    // Texels per tile side, and the levels kept: 2^-12 m (0.24 mm, a camera 0.3 m above the
    // ground at 1920 px) to 2^8 m (ground at the edge of the draw distance, seen edge-on).
    static constexpr int kTile = 32;
    static constexpr int kMinLevel = -12;
    static constexpr int kMaxLevel = 8;
    // Bytes one tile holds (4 x 16-bit channels per texel), for sizing a budget.
    static constexpr size_t kTileBytes = static_cast<size_t>(kTile) * kTile * 4 * sizeof(uint16_t);

    // `scene` must outlive the cache. budget_bytes = 0 means unbounded.
    GroundCache(std::shared_ptr<const RasterScene> scene, size_t budget_bytes);
    GroundCache(const GroundCache &) = delete;
    GroundCache &operator=(const GroundCache &) = delete;

    struct Tile;

    // A render thread's lookup memo: the tiles it touched last, so a run of neighbouring pixels
    // costs no hashing. One per thread (per band); cheap to make. Valid within one frame: tiles
    // are only evicted between frames.
    class Sampler {
      public:
        explicit Sampler(GroundCache &cache) : cache_(cache) {}
        // The lit ground colour (albedo under sky and sun, shadows included; before haze) at
        // (north, east) on the ground plane, for a pixel `footprint_m` across.
        void lit(double north_m, double east_m, double footprint_m, double out[3]);

      private:
        friend class GroundCache;
        // The tile, ready; null only when `optional` and it is not built and the frame's budget
        // for optional tiles is spent.
        const Tile *tile(int level, int64_t ti, int64_t tj, bool optional);
        // False (out untouched) only when `optional` and a tile it needs is not available.
        bool bilinear(int level, double north_m, double east_m, bool optional, double out[3]);
        struct Entry {
            uint64_t key{~uint64_t{0}};
            const Tile *tile{nullptr};
        };
        GroundCache &cache_;
        std::array<Entry, 16> memo_{};
    };

    // Tiles a frame may build for the lighter of a pixel's two levels (see above): about 80 ms of
    // one core, a quarter of a million texels.
    static constexpr int kOptionalTilesPerFrame = 256;

    // Starts a frame: tiles touched from here on count as used by it, and it may build up to
    // `optional_tiles` tiles nobody strictly needs yet. Returns its number.
    uint32_t begin_frame(int optional_tiles = kOptionalTilesPerFrame) {
        optional_budget_.store(optional_tiles, std::memory_order_relaxed);
        return frame_.fetch_add(1, std::memory_order_relaxed) + 1;
    }
    // Drops least recently used tiles until the cache is within its budget again. Call between
    // frames, never while a Sampler of this cache is in use.
    void evict();

    // The tiles a pixel at (north, east) with footprint `footprint_m` reads: the heavier level's
    // (`required`) and the lighter one's (`optional`), as tile keys. For a frame to collect ahead.
    static void tiles_for(double north_m, double east_m, double footprint_m, std::vector<uint64_t> &required,
                          std::vector<uint64_t> &optional);
    // Builds the tiles of `required`, and of `optional` as many as the frame's budget allows, that
    // are not built yet, spread over `pool` (null: on the calling thread). A frame that collects
    // them first (tiles_for over a sparse grid of its pixels) fills a new view on every thread at
    // once; left to the drawing itself, the bands that reach a tile while another builds it wait,
    // and a camera that climbs or descends — a new level every halving of its height — drew a
    // cold frame on little more than one core. Keys may repeat. Call between begin_frame() and
    // the drawing.
    void prepare(std::vector<uint64_t> &required, std::vector<uint64_t> &optional, core::ThreadPool *pool);

    struct Stats {
        size_t tiles{0};
        size_t bytes{0};
        uint64_t tiles_built{0};   // since start
        uint64_t tiles_evicted{0}; // since start
        size_t budget_bytes{0};
    };
    Stats stats() const;

    // The level a pixel `footprint_m` across reads, and the blend towards the next coarser one.
    static void level_of(double footprint_m, int &level, double &blend);

  private:
    static constexpr size_t kShards = 64;
    struct Shard {
        std::mutex mutex;
        std::unordered_map<uint64_t, std::shared_ptr<Tile>> tiles;
    };

    static uint64_t key_of(int level, int64_t ti, int64_t tj);
    static void unkey(uint64_t key, int &level, int64_t &ti, int64_t &tj);
    Shard &shard_of(uint64_t key) { return shards_[(key * 0x9E3779B97F4A7C15ULL) >> 58]; }
    const Tile *acquire(int level, int64_t ti, int64_t tj, uint64_t key, bool optional);
    void build(Tile &tile, int level, int64_t ti, int64_t tj) const;

    std::shared_ptr<const RasterScene> scene_;
    size_t budget_bytes_;
    std::array<Shard, kShards> shards_;
    std::atomic<uint32_t> frame_{0};
    std::atomic<int> optional_budget_{0};
    std::atomic<size_t> tile_count_{0};
    std::atomic<uint64_t> built_{0};
    std::atomic<uint64_t> evicted_{0};
};

} // namespace skysim::render
