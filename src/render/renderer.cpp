#include "render/renderer.h"

#include "render/shading.h"

#include <algorithm>
#include <cmath>

namespace skysim::render {
namespace {

using core::Vec3;

} // namespace

Image Renderer::render(const core::World &world, uint32_t vehicle_id, const core::Vec3 &position_ned,
                       const core::Quat &quat_ned_frd, core::ThreadPool *pool) const {
    Image image;
    image.width = cfg_.width;
    image.height = cfg_.height;
    image.rgb.assign(static_cast<size_t>(cfg_.width) * cfg_.height * 3, 0);

    const Camera camera(cfg_, position_ned, quat_ned_frd);

    if (pool != nullptr) {
        pool->parallel_for(static_cast<size_t>(cfg_.height), [&](size_t begin, size_t end) {
            render_rows(world, camera, vehicle_id, image, static_cast<int>(begin), static_cast<int>(end));
        });
    } else {
        render_rows(world, camera, vehicle_id, image, 0, cfg_.height);
    }

    return image;
}

void Renderer::render_rows(const core::World &world, const Camera &camera, uint32_t vehicle_id, Image &image,
                           int row_begin, int row_end) const {
    // Height of the camera above the ground plane. NED z is down, so a camera
    // 60 m up has z = -60.
    const double height = -camera.position()[2];

    for (int y = row_begin; y < row_end; ++y) {
        for (int x = 0; x < cfg_.width; ++x) {
            const Vec3 dir = camera.ray(x, y);

            // Where this ray meets the ground plane, analytically. Most rays in a
            // downward-looking camera end there, and casting the full draw
            // distance through a city to discover that is most of the frame time:
            // the cast only has to reach as far as the ground.
            double ground_dist = -1.0;
            if (dir[2] > 1e-6 && height > 0.0) {
                ground_dist = height / dir[2];
            }

            double cast_dist = cfg_.max_range_m;
            if (ground_dist > 0.0 && ground_dist < cast_dist) {
                cast_dist = ground_dist;
            }

            core::World::RayHit hit = world.raycast_surface(camera.position(), dir, cast_dist, vehicle_id);

            // Nothing solid within the shortened ray, but the ray still meets the
            // ground at the end of it.
            if (!hit.hit && ground_dist > 0.0 && ground_dist <= cfg_.max_range_m) {
                hit.hit = true;
                hit.distance = ground_dist;
                hit.normal_ned = {0.0, 0.0, -1.0}; // ground faces up; NED up is -z
            }

            double colour[3];
            if (!hit.hit) {
                sky_colour(dir, colour);
            } else {
                const double *base = hit.is_ground ? kGround : kBuilding;
                const double light = shade_for_normal(hit.normal_ned);
                const double haze = haze_at(hit.distance, cfg_.max_range_m);
                for (int c = 0; c < 3; ++c) {
                    colour[c] = mix(base[c] * light, kSkyHorizon[c], haze);
                }
            }

            const size_t i = (static_cast<size_t>(y) * cfg_.width + x) * 3;
            for (int c = 0; c < 3; ++c) {
                image.rgb[i + c] = to_byte(colour[c]);
            }
        }
    }
}

} // namespace skysim::render
