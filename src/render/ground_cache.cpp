#include "render/ground_cache.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace skysim::render {
namespace {

// Lit colour is stored as 16-bit fixed point: 0..4 in steps of 1/16384, far finer than the 1/255
// a frame is quantised to after haze, and room for any albedo under the brightest light.
constexpr double kScale = 16384.0;
constexpr double kInvScale = 1.0 / kScale;

// The key packs (level, tile row, tile column) into 64 bits, 29 bits per index. Coordinates are
// clamped well inside that range even on the finest level: ground a camera can see is within the
// draw distance of a vehicle, and nothing flies a thousand kilometres from the world's origin.
constexpr double kMaxCoordM = 1.0e6;

// 2^-level for every level the cache keeps (and one past it), exact: a table, not libm's ldexp.
struct InverseSizes {
    double v[GroundCache::kMaxLevel - GroundCache::kMinLevel + 2];
    constexpr InverseSizes() : v{} {
        for (int k = 0; k < GroundCache::kMaxLevel - GroundCache::kMinLevel + 2; ++k) {
            double x = 1.0;
            const int level = GroundCache::kMinLevel + k;
            for (int i = 0; i < (level < 0 ? -level : level); ++i) {
                x = level < 0 ? x * 2.0 : x * 0.5;
            }
            v[k] = x;
        }
    }
};
constexpr InverseSizes kInverseSize{};

// floor() as an integer, for |v| far below 2^63: truncation, then one down for a negative
// fraction. The coordinates here are clamped to a million metres.
inline int64_t floor_int(double v) {
    const auto i = static_cast<int64_t>(v);
    return i - (v < static_cast<double>(i) ? 1 : 0);
}

} // namespace

struct GroundCache::Tile {
    std::atomic<uint8_t> state{0}; // 0 empty, 1 being built, 2 ready
    std::atomic<uint32_t> last_used{0};
    std::array<uint16_t, static_cast<size_t>(kTile) * kTile * 3> rgb{};
};

GroundCache::GroundCache(std::shared_ptr<const RasterScene> scene, size_t budget_bytes)
    : scene_(std::move(scene)), budget_bytes_(budget_bytes) {}

uint64_t GroundCache::key_of(int level, int64_t ti, int64_t tj) {
    constexpr uint64_t kMask = (uint64_t{1} << 29) - 1;
    return (static_cast<uint64_t>(level - kMinLevel) << 58) | ((static_cast<uint64_t>(ti) & kMask) << 29) |
           (static_cast<uint64_t>(tj) & kMask);
}

void GroundCache::level_of(double footprint_m, int &level, double &blend) {
    // log2 split into its exponent, read off the double's bits, and the log2 of its mantissa from
    // a polynomial: libm's log2 and floor were a third of a sample's cost. The blend is a weight,
    // and 1e-4 of a weight is far below what a byte of colour can show.
    if (!(footprint_m > 0.0)) { // zero, negative or NaN: the finest level
        level = kMinLevel;
        blend = 0.0;
        return;
    }
    uint64_t bits = 0;
    std::memcpy(&bits, &footprint_m, sizeof(bits));
    const int exponent = static_cast<int>((bits >> 52) & 0x7FF) - 1023;
    if (exponent < kMinLevel) { // subnormals land here too
        level = kMinLevel;
        blend = 0.0;
        return;
    }
    if (exponent >= kMaxLevel) {
        level = kMaxLevel;
        blend = 0.0;
        return;
    }
    // Mantissa m in [1, 2); log2(m) = log2(1 + x) to 2e-4 (a least-squares quartic in x).
    bits = (bits & 0x000FFFFFFFFFFFFFULL) | 0x3FF0000000000000ULL;
    double m = 0.0;
    std::memcpy(&m, &bits, sizeof(m));
    const double x = m - 1.0;
    level = exponent;
    blend = std::clamp(x * (1.43854793 + x * (-0.67808946 + x * (0.32364631 + x * -0.08429466))), 0.0, 1.0);
}

void GroundCache::build(Tile &tile, int level, int64_t ti, int64_t tj) const {
    const double s = std::ldexp(1.0, level);
    for (int li = 0; li < kTile; ++li) {
        const double n = (static_cast<double>(ti * kTile + li) + 0.5) * s;
        for (int lj = 0; lj < kTile; ++lj) {
            const double e = (static_cast<double>(tj * kTile + lj) + 0.5) * s;
            double c[3];
            // The colour WorldTexture and the light give a pixel exactly this texel's size there.
            scene_->lit_colour(kSurfaceGround, core::Vec3{n, e, 0.0}, s, c);
            uint16_t *out = &tile.rgb[(static_cast<size_t>(li) * kTile + static_cast<size_t>(lj)) * 3];
            for (int k = 0; k < 3; ++k) {
                const double v = c[k] > 0.0 ? c[k] * kScale + 0.5 : 0.0; // NaN to 0, like to_byte
                out[k] = static_cast<uint16_t>(std::min(v, 65535.0));
            }
        }
    }
}

const GroundCache::Tile *GroundCache::acquire(int level, int64_t ti, int64_t tj, uint64_t key,
                                              bool optional) {
    Shard &shard = shard_of(key);
    Tile *tile = nullptr;
    {
        std::lock_guard<std::mutex> lock(shard.mutex);
        auto it = shard.tiles.find(key);
        if (it == shard.tiles.end()) {
            // An optional tile is only made while the frame's budget lasts; the budget is spent here,
            // when it is made, so threads racing for the last of it cannot overspend it by much.
            if (optional && optional_budget_.fetch_sub(1, std::memory_order_relaxed) <= 0) {
                return nullptr;
            }
            it = shard.tiles.emplace(key, std::make_shared<Tile>()).first;
            tile_count_.fetch_add(1, std::memory_order_relaxed);
        }
        tile = it->second.get();
    }
    tile->last_used.store(frame_.load(std::memory_order_relaxed), std::memory_order_relaxed);
    uint8_t state = tile->state.load(std::memory_order_acquire);
    if (state == 2) {
        return tile;
    }
    uint8_t empty = 0;
    if (tile->state.compare_exchange_strong(empty, 1, std::memory_order_acq_rel)) {
        build(*tile, level, ti, tj);
        built_.fetch_add(1, std::memory_order_relaxed);
        tile->state.store(2, std::memory_order_release);
        tile->state.notify_all();
        return tile;
    }
    // Another thread is filling it: a tile takes a fraction of a millisecond, and every texel of
    // it comes out the same whoever computes it, so wait rather than compute it twice.
    while ((state = tile->state.load(std::memory_order_acquire)) != 2) {
        tile->state.wait(state, std::memory_order_acquire);
    }
    return tile;
}

const GroundCache::Tile *GroundCache::Sampler::tile(int level, int64_t ti, int64_t tj, bool optional) {
    const uint64_t key = key_of(level, ti, tj);
    // Fibonacci hashing: neighbouring tiles of both levels land in different slots (an xor of the
    // indices put a tile and its diagonal neighbour in one slot, and they evicted each other).
    Entry &e = memo_[(key * 0x9E3779B97F4A7C15ULL) >> 60];
    // A miss is remembered too (a null tile), so the pixels after it do not each go back to the
    // shard for a tile this frame will not build; a required lookup of it goes back and builds it.
    if (e.key != key || (e.tile == nullptr && !optional)) {
        e.tile = cache_.acquire(level, ti, tj, key, optional);
        e.key = key;
    }
    return e.tile;
}

bool GroundCache::Sampler::bilinear(int level, double north_m, double east_m, bool optional, double out[3]) {
    const double inv = kInverseSize.v[level - kMinLevel];
    // Texel centres sit at half-integer multiples of the texel size.
    const double u = std::clamp(north_m, -kMaxCoordM, kMaxCoordM) * inv - 0.5;
    const double v = std::clamp(east_m, -kMaxCoordM, kMaxCoordM) * inv - 0.5;
    const int64_t i0 = floor_int(u);
    const int64_t j0 = floor_int(v);
    // Integer weights, 1/256 steps, summing to 65536: with 16-bit texels every product and the sum
    // fit 32 bits exactly, and the blend costs integer multiply-adds instead of a dozen
    // int-to-double conversions (which were most of a sample's time).
    const auto wu = static_cast<uint32_t>((u - static_cast<double>(i0)) * 256.0 + 0.5);
    const auto wv = static_cast<uint32_t>((v - static_cast<double>(j0)) * 256.0 + 0.5);
    const uint32_t w[4] = {(256 - wu) * (256 - wv), (256 - wu) * wv, wu * (256 - wv), wu * wv};
    // kTile is a power of two: >> floors (C++20 defines it on negatives) and & is the remainder.
    constexpr int kShift = 5;
    static_assert((1 << kShift) == kTile);
    const int li = static_cast<int>(i0 & (kTile - 1));
    const int lj = static_cast<int>(j0 & (kTile - 1));
    const uint16_t *corners[4];
    if (li < kTile - 1 && lj < kTile - 1) {
        // All four in one tile: the common case.
        const Tile *t = tile(level, i0 >> kShift, j0 >> kShift, optional);
        if (t == nullptr) {
            return false;
        }
        const uint16_t *p00 = &t->rgb[(static_cast<size_t>(li) * kTile + static_cast<size_t>(lj)) * 3];
        corners[0] = p00;
        corners[1] = p00 + 3;
        corners[2] = p00 + kTile * 3;
        corners[3] = p00 + kTile * 3 + 3;
    } else {
        for (int k = 0; k < 4; ++k) {
            const int64_t i = i0 + (k >> 1);
            const int64_t j = j0 + (k & 1);
            const Tile *t = tile(level, i >> kShift, j >> kShift, optional);
            if (t == nullptr) {
                return false;
            }
            corners[k] = &t->rgb[(static_cast<size_t>(i & (kTile - 1)) * kTile +
                                  static_cast<size_t>(j & (kTile - 1))) *
                                 3];
        }
    }
    constexpr double kNorm = kInvScale / 65536.0;
    for (int c = 0; c < 3; ++c) {
        const uint32_t acc =
            w[0] * corners[0][c] + w[1] * corners[1][c] + w[2] * corners[2][c] + w[3] * corners[3][c];
        out[c] = static_cast<double>(acc) * kNorm;
    }
    return true;
}

void GroundCache::Sampler::lit(double north_m, double east_m, double footprint_m, double out[3]) {
    int level = 0;
    double blend = 0.0;
    level_of(footprint_m, level, blend);
    constexpr double kSkip = 1.0 / 256.0; // a level weighted less than this would not change a byte
    if (blend <= kSkip) {
        bilinear(level, north_m, east_m, false, out);
        return;
    }
    if (blend >= 1.0 - kSkip) {
        bilinear(level + 1, north_m, east_m, false, out);
        return;
    }
    // The heavier level always; the lighter one when it is there (or the budget builds it).
    const bool fine_heavier = blend < 0.5;
    double fine[3];
    double coarse[3];
    bilinear(fine_heavier ? level : level + 1, north_m, east_m, false, fine_heavier ? fine : coarse);
    if (!bilinear(fine_heavier ? level + 1 : level, north_m, east_m, true, fine_heavier ? coarse : fine)) {
        const double *only = fine_heavier ? fine : coarse;
        for (int c = 0; c < 3; ++c) {
            out[c] = only[c];
        }
        return;
    }
    for (int c = 0; c < 3; ++c) {
        out[c] = fine[c] + (coarse[c] - fine[c]) * blend;
    }
}

void GroundCache::evict() {
    if (budget_bytes_ == 0) {
        return;
    }
    const size_t count = tile_count_.load(std::memory_order_relaxed);
    if (count * kTileBytes <= budget_bytes_) {
        return;
    }
    // Down to 90 % of the budget, so a cache at its limit does not sort itself every frame.
    const size_t keep = budget_bytes_ / kTileBytes * 9 / 10;
    const uint32_t now = frame_.load(std::memory_order_relaxed);
    std::vector<std::pair<uint32_t, uint64_t>> aged; // (frames since last used, key)
    aged.reserve(count);
    for (Shard &shard : shards_) {
        std::lock_guard<std::mutex> lock(shard.mutex);
        for (const auto &[key, tile] : shard.tiles) {
            aged.emplace_back(now - tile->last_used.load(std::memory_order_relaxed), key);
        }
    }
    if (aged.size() <= keep) {
        return;
    }
    const size_t drop = aged.size() - keep;
    std::nth_element(aged.begin(), aged.begin() + static_cast<std::ptrdiff_t>(drop), aged.end(),
                     [](const auto &a, const auto &b) { return a.first > b.first; });
    size_t dropped = 0;
    for (size_t k = 0; k < drop; ++k) {
        Shard &shard = shard_of(aged[k].second);
        std::lock_guard<std::mutex> lock(shard.mutex);
        dropped += shard.tiles.erase(aged[k].second);
    }
    tile_count_.fetch_sub(dropped, std::memory_order_relaxed);
    evicted_.fetch_add(dropped, std::memory_order_relaxed);
}

GroundCache::Stats GroundCache::stats() const {
    Stats s;
    s.tiles = tile_count_.load(std::memory_order_relaxed);
    s.bytes = s.tiles * kTileBytes;
    s.tiles_built = built_.load(std::memory_order_relaxed);
    s.tiles_evicted = evicted_.load(std::memory_order_relaxed);
    s.budget_bytes = budget_bytes_;
    return s;
}

} // namespace skysim::render
