#include "render/resize.h"

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace skysim::render {
namespace {

// Where one destination column or row reads from: two source indices and the weight of the
// second, in 1/256 steps (so a pixel is a sum of four 8-bit products of 16-bit weights, exact in
// 32 bits).
struct Tap {
    int a;
    int b;
    uint32_t w;
};

std::vector<Tap> taps(int src, int dst) {
    std::vector<Tap> out(static_cast<size_t>(dst));
    const double scale = static_cast<double>(src) / static_cast<double>(dst);
    for (int i = 0; i < dst; ++i) {
        const double x = std::clamp((i + 0.5) * scale - 0.5, 0.0, static_cast<double>(src - 1));
        const int a = std::min(static_cast<int>(x), src - 1);
        const int b = std::min(a + 1, src - 1);
        out[static_cast<size_t>(i)] = {a, b, static_cast<uint32_t>((x - a) * 256.0 + 0.5)};
    }
    return out;
}

} // namespace

Image resize_bilinear(const Image &src, int width, int height, core::ThreadPool *pool) {
    Image out;
    if (src.empty() || src.width <= 0 || src.height <= 0 || width <= 0 || height <= 0) {
        return out;
    }
    out.width = width;
    out.height = height;
    out.rgb.resize(static_cast<size_t>(width) * static_cast<size_t>(height) * 3);
    const std::vector<Tap> cols = taps(src.width, width);
    const std::vector<Tap> rows = taps(src.height, height);
    const size_t stride = static_cast<size_t>(src.width) * 3;
    const auto do_rows = [&](int y0, int y1) {
        for (int y = y0; y < y1; ++y) {
            const Tap &r = rows[static_cast<size_t>(y)];
            const uint8_t *top = src.rgb.data() + static_cast<size_t>(r.a) * stride;
            const uint8_t *bottom = src.rgb.data() + static_cast<size_t>(r.b) * stride;
            uint8_t *dst = out.rgb.data() + static_cast<size_t>(y) * static_cast<size_t>(width) * 3;
            const uint32_t wy = r.w;
            for (int x = 0; x < width; ++x) {
                const Tap &c = cols[static_cast<size_t>(x)];
                const uint32_t wx = c.w;
                const uint32_t w00 = (256 - wx) * (256 - wy);
                const uint32_t w01 = wx * (256 - wy);
                const uint32_t w10 = (256 - wx) * wy;
                const uint32_t w11 = wx * wy;
                const size_t a = static_cast<size_t>(c.a) * 3;
                const size_t b = static_cast<size_t>(c.b) * 3;
                for (int k = 0; k < 3; ++k) {
                    const uint32_t v =
                        w00 * top[a + k] + w01 * top[b + k] + w10 * bottom[a + k] + w11 * bottom[b + k];
                    dst[static_cast<size_t>(x) * 3 + static_cast<size_t>(k)] =
                        static_cast<uint8_t>((v + 32768) >> 16);
                }
            }
        }
    };
    if (pool == nullptr) {
        do_rows(0, height);
        return out;
    }
    // Rows in blocks of 16, taken as threads come free.
    constexpr int kBlock = 16;
    const int blocks = (height + kBlock - 1) / kBlock;
    std::atomic<int> next{0};
    pool->parallel_for(pool->size() + 1, [&](size_t, size_t) {
        for (int b = next.fetch_add(1, std::memory_order_relaxed); b < blocks;
             b = next.fetch_add(1, std::memory_order_relaxed)) {
            do_rows(b * kBlock, std::min(height, (b + 1) * kBlock));
        }
    });
    return out;
}

} // namespace skysim::render
