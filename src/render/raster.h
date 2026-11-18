#pragma once

#include "core/thread_pool.h"
#include "core/world.h"
#include "render/camera.h"
#include "render/dock_marker.h"
#include "render/ground_cache.h"
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
// thread count. Workers take the next band as they finish one, so a thread that drew sky is
// not left idle while another is still on a band of grass (bands of the ground cost several
// times what bands of sky do, and a fixed deal left a quarter of a 4-thread frame idle).
class Rasterizer {
  public:
    explicit Rasterizer(const CameraConfig &cfg) : cfg_(cfg) {}

    // `surface`, when given, receives one entry per pixel: the scene triangle index, or
    // kSurfaceGround / kSurfaceSky. caller_runs = false keeps the calling thread out of the
    // work (see ThreadPool::parallel_for). `markers`, when given, are the dock markers lying on
    // the ground (render/dock_marker.h); a marker's pixels are ground in `surface`. `ground`,
    // when given, is where the lit ground plane's colour is read from instead of computed per
    // pixel (render/ground_cache.h; the pilot camera's, TASK-2661): it must be built on `scene`.
    Image render(const RasterScene &scene, const Camera &camera, core::ThreadPool *pool,
                 std::vector<int32_t> *surface = nullptr, bool caller_runs = true,
                 const GroundMarkers *markers = nullptr, GroundCache *ground = nullptr) const;

  private:
    // Project, clip against the near plane and cull, producing screen triangles.
    void transform(const RasterScene &scene, const Camera &camera, std::vector<ScreenTriangle> &out) const;

    CameraConfig cfg_;
};

} // namespace skysim::render
