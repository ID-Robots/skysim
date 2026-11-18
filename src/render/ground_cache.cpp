#include "render/ground_cache.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#if defined(__SSE2__)
#include <emmintrin.h>
#endif

namespace skysim::render {
namespace {

// Lit colour is stored as 15-bit fixed point, four channels a texel (the fourth unused): 0..4 in
// steps of 1/8192, far finer than the 1/255 a frame is quantised to after haze, with room for any
// albedo under the brightest light. Fifteen bits and four channels because a texel is then one
// 64-bit load, and two texels times two 14-bit bilinear weights are one SSE2 multiply-add of
// signed 16-bit pairs (pmaddwd) into four exact 32-bit sums.
constexpr double kScale = 8192.0;
constexpr double kInvScale = 1.0 / kScale;
constexpr int kChannels = 4;

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
    std::atomic<uint64_t> built_seq{0}; // when it was built (GroundCache::build_seq_)
    std::array<uint16_t, static_cast<size_t>(kTile) * kTile * kChannels> rgb{};
};

GroundCache::GroundCache(std::shared_ptr<const RasterScene> scene, size_t budget_bytes)
    : scene_(std::move(scene)), budget_bytes_(budget_bytes) {}

uint64_t GroundCache::key_of(int level, int64_t ti, int64_t tj) {
    constexpr uint64_t kMask = (uint64_t{1} << 29) - 1;
    return (static_cast<uint64_t>(level - kMinLevel) << 58) | ((static_cast<uint64_t>(ti) & kMask) << 29) |
           (static_cast<uint64_t>(tj) & kMask);
}

void GroundCache::unkey(uint64_t key, int &level, int64_t &ti, int64_t &tj) {
    constexpr uint64_t kMask = (uint64_t{1} << 29) - 1;
    level = static_cast<int>(key >> 58) + kMinLevel;
    // Sign-extend the 29-bit indices.
    const auto extend = [](uint64_t v) {
        return static_cast<int64_t>(v & (uint64_t{1} << 28) ? v | ~kMask : v);
    };
    ti = extend((key >> 29) & kMask);
    tj = extend(key & kMask);
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
            uint16_t *out =
                &tile.rgb[(static_cast<size_t>(li) * kTile + static_cast<size_t>(lj)) * kChannels];
            for (int k = 0; k < 3; ++k) {
                const double v = c[k] > 0.0 ? c[k] * kScale + 0.5 : 0.0; // NaN to 0, like to_byte
                out[k] = static_cast<uint16_t>(std::min(v, 32767.0));
            }
            out[3] = 0;
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
                // Put off to a later frame; but not once the frame is sealed: then a pixel between
                // prepare()'s samples asked, and no frame builds an optional tile for that.
                if (sealed_seq_.load(std::memory_order_relaxed) == kUnsealed) {
                    deferred_.store(true, std::memory_order_relaxed);
                }
                return nullptr;
            }
            it = shard.tiles.emplace(key, std::make_shared<Tile>()).first;
            tile_count_.fetch_add(1, std::memory_order_relaxed);
        }
        tile = it->second.get();
    }
    tile->last_used.store(frame_.load(std::memory_order_relaxed), std::memory_order_relaxed);
    if (optional && sealed_seq_.load(std::memory_order_relaxed) != kUnsealed) {
        return optional_ok(*tile) ? tile : nullptr; // sealed: never built, never waited for
    }
    // Ready; or empty, and this thread fills it; or another thread is filling it — a tile takes a
    // fraction of a millisecond, and every texel of it comes out the same whoever computes it, so
    // wait rather than compute it twice. A fill that failed leaves it empty again (fill()), and a
    // waiter then takes it on.
    for (;;) {
        const uint8_t state = tile->state.load(std::memory_order_acquire);
        if (state == 2) {
            return tile;
        }
        if (state == 0) {
            uint8_t empty = 0;
            if (tile->state.compare_exchange_strong(empty, 1, std::memory_order_acq_rel)) {
                fill(*tile, level, ti, tj);
                return tile;
            }
            continue;
        }
        tile->state.wait(1, std::memory_order_acquire);
    }
}

void GroundCache::fill(Tile &tile, int level, int64_t ti, int64_t tj) {
    try {
        build(tile, level, ti, tj);
    } catch (...) {
        tile.state.store(0, std::memory_order_release);
        tile.state.notify_all();
        throw;
    }
    built_.fetch_add(1, std::memory_order_relaxed);
    tile.built_seq.store(build_seq_.fetch_add(1, std::memory_order_relaxed) + 1, std::memory_order_relaxed);
    tile.state.store(2, std::memory_order_release);
    tile.state.notify_all();
}

bool GroundCache::optional_ok(const Tile &tile) const {
    const uint64_t sealed = sealed_seq_.load(std::memory_order_relaxed);
    if (sealed == kUnsealed) {
        return true;
    }
    // Sealed: only a tile ready when the frame was sealed, so a pixel's blend does not depend on
    // whether another band built the tile (as a required one) before or after it looked.
    return tile.state.load(std::memory_order_acquire) == 2 &&
           tile.built_seq.load(std::memory_order_relaxed) <= sealed;
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
    // A tile remembered from a required lookup may still not be one an optional lookup may use.
    return optional && e.tile != nullptr && !cache_.optional_ok(*e.tile) ? nullptr : e.tile;
}

bool GroundCache::Sampler::bilinear(int level, double north_m, double east_m, bool optional, double out[3]) {
    if (!(std::isfinite(north_m) && std::isfinite(east_m))) {
        out[0] = out[1] = out[2] = 0.0; // nowhere: black, rather than an undefined tile index
        return true;
    }
    const double inv = kInverseSize.v[level - kMinLevel];
    // Texel centres sit at half-integer multiples of the texel size.
    const double u = std::clamp(north_m, -kMaxCoordM, kMaxCoordM) * inv - 0.5;
    const double v = std::clamp(east_m, -kMaxCoordM, kMaxCoordM) * inv - 0.5;
    const int64_t i0 = floor_int(u);
    const int64_t j0 = floor_int(v);
    // Integer weights in 1/128 steps, the four summing to 16384: with 15-bit texels every product
    // and their sum fit a signed 32-bit integer exactly, so the SSE2 path and the scalar one give
    // the same bits on every machine.
    const auto wu = static_cast<int32_t>((u - static_cast<double>(i0)) * 128.0 + 0.5);
    const auto wv = static_cast<int32_t>((v - static_cast<double>(j0)) * 128.0 + 0.5);
    const int32_t w[4] = {(128 - wu) * (128 - wv), (128 - wu) * wv, wu * (128 - wv), wu * wv};
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
        const uint16_t *p00 =
            &t->rgb[(static_cast<size_t>(li) * kTile + static_cast<size_t>(lj)) * kChannels];
        corners[0] = p00;
        corners[1] = p00 + kChannels;
        corners[2] = p00 + kTile * kChannels;
        corners[3] = p00 + kTile * kChannels + kChannels;
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
                                 kChannels];
        }
    }
    int32_t acc[4];
#if defined(__SSE2__)
    // Two texels interleaved channel by channel, times their two weights, summed pairwise.
    const auto pair = [](const uint16_t *a, const uint16_t *b, int32_t wa, int32_t wb) {
        const __m128i ab = _mm_unpacklo_epi16(_mm_loadl_epi64(reinterpret_cast<const __m128i *>(a)),
                                              _mm_loadl_epi64(reinterpret_cast<const __m128i *>(b)));
        return _mm_madd_epi16(ab, _mm_set1_epi32(static_cast<int32_t>((static_cast<uint32_t>(wb) << 16) |
                                                                      static_cast<uint32_t>(wa))));
    };
    _mm_storeu_si128(
        reinterpret_cast<__m128i *>(acc),
        _mm_add_epi32(pair(corners[0], corners[1], w[0], w[1]), pair(corners[2], corners[3], w[2], w[3])));
#else
    for (int c = 0; c < 3; ++c) {
        acc[c] = w[0] * corners[0][c] + w[1] * corners[1][c] + w[2] * corners[2][c] + w[3] * corners[3][c];
    }
#endif
    constexpr double kNorm = kInvScale / 16384.0;
    for (int c = 0; c < 3; ++c) {
        out[c] = static_cast<double>(acc[c]) * kNorm;
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

void GroundCache::tiles_for(double north_m, double east_m, double footprint_m,
                            std::vector<uint64_t> &required, std::vector<uint64_t> &optional) {
    if (!(std::isfinite(north_m) && std::isfinite(east_m))) {
        return;
    }
    int level = 0;
    double blend = 0.0;
    level_of(footprint_m, level, blend);
    constexpr double kSkip = 1.0 / 256.0; // as Sampler::lit decides
    const auto add = [&](int lv, std::vector<uint64_t> &out) {
        const double inv = kInverseSize.v[lv - kMinLevel];
        const int64_t i0 = floor_int(std::clamp(north_m, -kMaxCoordM, kMaxCoordM) * inv - 0.5);
        const int64_t j0 = floor_int(std::clamp(east_m, -kMaxCoordM, kMaxCoordM) * inv - 0.5);
        // The bilinear footprint's corners: one tile, or its neighbours across a tile edge.
        for (int64_t i = i0 >> 5; i <= (i0 + 1) >> 5; ++i) {
            for (int64_t j = j0 >> 5; j <= (j0 + 1) >> 5; ++j) {
                out.push_back(key_of(lv, i, j));
            }
        }
    };
    if (blend <= kSkip) {
        add(level, required);
    } else if (blend >= 1.0 - kSkip) {
        add(level + 1, required);
    } else {
        add(blend < 0.5 ? level : level + 1, required);
        add(blend < 0.5 ? level + 1 : level, optional);
    }
}

void GroundCache::prepare(std::vector<uint64_t> &required, std::vector<uint64_t> &optional,
                          core::ThreadPool *pool, bool caller_runs) {
    const auto unique = [](std::vector<uint64_t> &v) {
        std::sort(v.begin(), v.end());
        v.erase(std::unique(v.begin(), v.end()), v.end());
    };
    unique(required);
    unique(optional);
    // The tiles to build now: present-and-ready ones are left alone; missing ones are made here
    // (an optional one while the frame's budget lasts), so the workers below only fill them. The
    // required ones it builds come out of that budget first: a cold frame builds what it must.
    std::vector<std::pair<uint64_t, Tile *>> todo;
    const uint32_t frame = frame_.load(std::memory_order_relaxed);
    const auto take = [&](const std::vector<uint64_t> &keys, bool opt) {
        for (const uint64_t key : keys) {
            Shard &shard = shard_of(key);
            std::lock_guard<std::mutex> lock(shard.mutex);
            auto it = shard.tiles.find(key);
            if (it == shard.tiles.end()) {
                if (opt && optional_budget_.fetch_sub(1, std::memory_order_relaxed) <= 0) {
                    deferred_.store(true, std::memory_order_relaxed);
                    deferred_tiles_.fetch_add(1, std::memory_order_relaxed);
                    continue;
                }
                it = shard.tiles.emplace(key, std::make_shared<Tile>()).first;
                tile_count_.fetch_add(1, std::memory_order_relaxed);
            }
            it->second->last_used.store(frame, std::memory_order_relaxed);
            if (it->second->state.load(std::memory_order_acquire) == 0) {
                todo.emplace_back(key, it->second.get());
            }
        }
    };
    take(required, false);
    // (A frame's tiles: tens of thousands at the very most, an int whatever the budget.)
    optional_budget_.fetch_sub(static_cast<int>(std::min<size_t>(todo.size(), 1u << 20)),
                               std::memory_order_relaxed);
    take(optional, true);
    if (todo.empty()) {
        return;
    }
    std::atomic<size_t> next{0};
    const auto work = [&](size_t, size_t) {
        for (size_t k = next.fetch_add(1, std::memory_order_relaxed); k < todo.size();
             k = next.fetch_add(1, std::memory_order_relaxed)) {
            Tile &tile = *todo[k].second;
            uint8_t empty = 0;
            if (!tile.state.compare_exchange_strong(empty, 1, std::memory_order_acq_rel)) {
                continue; // somebody else is filling it
            }
            int level = 0;
            int64_t ti = 0;
            int64_t tj = 0;
            unkey(todo[k].first, level, ti, tj);
            fill(tile, level, ti, tj);
        }
    };
    if (pool != nullptr && (todo.size() > 1 || !caller_runs)) {
        pool->parallel_for(pool->size() + (caller_runs ? 1 : 0), work, caller_runs);
    } else {
        work(0, 1);
    }
}

void GroundCache::evict() {
    if (budget_bytes_ == 0) {
        return;
    }
    // Down to 90 % of the budget, so a cache at its limit does not sort itself every frame.
    const size_t keep = budget_bytes_ / kTileBytes * 9 / 10;
    const uint32_t now = frame_.load(std::memory_order_relaxed);
    if (deferred_.load(std::memory_order_relaxed)) {
        // The frame put tiles off. Whether a later one from the same place can build them and keep
        // them all: the tiles this frame used (they carry its number) and the ones it put off must
        // fit the cache together, or each frame evicts what the next one needs. Against its whole
        // capacity, not the 90 % evict() trims to: the cache is only trimmed once it holds more than
        // its capacity, and a view that fits it fills (the older tiles go first, and the few of its
        // own that the trim takes are built again by the next frame) and is not evicted from again.
        // Counted only for a frame that put tiles off, so a settled view costs no walk of the cache.
        size_t used = 0;
        for (Shard &shard : shards_) {
            std::lock_guard<std::mutex> lock(shard.mutex);
            for (const auto &entry : shard.tiles) {
                used += entry.second->last_used.load(std::memory_order_relaxed) == now ? 1 : 0;
            }
        }
        refinable_.store(used + deferred_tiles_.load(std::memory_order_relaxed) <= budget_bytes_ / kTileBytes,
                         std::memory_order_relaxed);
    }
    const size_t count = tile_count_.load(std::memory_order_relaxed);
    if (count * kTileBytes <= budget_bytes_) {
        return;
    }
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
