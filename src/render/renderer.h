#pragma once

#include "core/thread_pool.h"
#include "core/world.h"
#include "render/camera.h"
#include "render/image.h"

#include <cstdint>
#include <vector>

namespace skysim::render {

// Draws the world the way the physics already knows it.
//
// NOT the production renderer — Rasterizer is, and is ~17x faster. This is kept
// deliberately, as the reference the rasteriser is checked against: it asks the
// collision world directly, one ray per pixel, so it cannot disagree with the
// geometry a vehicle will actually hit. tests/test_render.cpp holds both to the
// same facts. Delete it only along with that comparison.
//
// This is a ray caster rather than a rasteriser, and that is the whole trick:
// skysim already holds every building as collision geometry, and already
// answers "what does this ray hit" fast enough to fly rangefinders off. Casting
// one ray per pixel reuses that exact geometry, so the picture cannot disagree
// with what the vehicle will crash into — no second copy of the world to build,
// load or keep in step.
//
// The cost is per-pixel rather than per-triangle, which is why the resolution is
// deliberately small and the rows are spread across the pool.
class Renderer {
  public:
    explicit Renderer(const CameraConfig &cfg) : cfg_(cfg) {}

    const CameraConfig &config() const { return cfg_; }

    // Render the view from `vehicle_id`'s airframe. The vehicle's own collision
    // box is excluded, or every ray would hit it at zero range.
    Image render(const core::World &world, uint32_t vehicle_id, const core::Vec3 &position_ned,
                 const core::Quat &quat_ned_frd, core::ThreadPool *pool = nullptr) const;

  private:
    void render_rows(const core::World &world, const Camera &camera, uint32_t vehicle_id, Image &image, int row_begin,
                     int row_end) const;

    CameraConfig cfg_;
};

} // namespace skysim::render
