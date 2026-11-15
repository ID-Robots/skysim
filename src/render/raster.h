#pragma once

#include "core/thread_pool.h"
#include "core/world.h"
#include "render/camera.h"
#include "render/image.h"
#include "render/scene.h"

#include <cstdint>
#include <vector>

namespace skysim::render {

// A triangle already projected into screen space, ready to fill.
struct ScreenTriangle {
    // Pixel coordinates, and 1/depth for perspective-correct depth.
    float x[3];
    float y[3];
    float inv_w[3];
    uint32_t tri; // scene triangle this came from (a near-plane clip can yield two)
    int min_y;
    int max_y;
};

// Draws the world by filling triangles instead of chasing a ray per pixel.
//
// Two passes per horizontal band of the screen. The first fills triangles into a depth and
// triangle-id buffer — no shading, so overdraw costs a compare and two stores. The second
// shades each pixel once: it intersects the pixel's ray with the plane of the triangle that
// won (or with the ground plane, or sees sky), so the world point it shades is exact to the
// micrometre rather than interpolated in float, and colours it from the scene's world-space
// texture and light. That exactness is what makes a building corner the same colour in
// every photo of it.
//
// Threading: each band's pixels and buffers are touched by exactly one thread, so there are
// no atomics and no locks in the inner loops, and the picture is identical whatever the
// thread count. Bands are dealt round-robin to the workers so sky-heavy and city-heavy bands
// share the load evenly.
class Rasterizer {
  public:
    explicit Rasterizer(const CameraConfig &cfg) : cfg_(cfg) {}

    // `surface`, when given, receives one entry per pixel: the scene triangle index, or
    // kSurfaceGround / kSurfaceSky. caller_runs = false keeps the calling thread out of the
    // work (see ThreadPool::parallel_for).
    Image render(const RasterScene &scene, const Camera &camera, core::ThreadPool *pool,
                 std::vector<int32_t> *surface = nullptr, bool caller_runs = true) const;

  private:
    // Project, clip against the near plane and cull, producing screen triangles.
    void transform(const RasterScene &scene, const Camera &camera, std::vector<ScreenTriangle> &out) const;

    CameraConfig cfg_;
};

} // namespace skysim::render
