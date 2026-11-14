#include "render/renderer.h"

#include "render/shading.h"

#include <algorithm>
#include <cmath>

namespace skysim::render {
namespace {

using core::Vec3;

} // namespace

Image Renderer::render(const core::World &world, uint32_t vehicle_id, const core::Vec3 &position_ned,
                       const core::Quat &quat_ned_frd, core::ThreadPool *pool,
                       std::vector<int32_t> *surface) const {
    Image image;
    image.width = cfg_.width;
    image.height = cfg_.height;
    image.rgb.assign(static_cast<size_t>(cfg_.width) * cfg_.height * 3, 0);
    if (surface != nullptr) {
        surface->assign(static_cast<size_t>(cfg_.width) * cfg_.height, kSurfaceSky);
    }

    const Camera camera(cfg_, position_ned, quat_ned_frd);

    if (pool != nullptr) {
        pool->parallel_for(static_cast<size_t>(cfg_.height), [&](size_t begin, size_t end) {
            render_rows(world, camera, vehicle_id, image, surface, static_cast<int>(begin),
                        static_cast<int>(end));
        });
    } else {
        render_rows(world, camera, vehicle_id, image, surface, 0, cfg_.height);
    }

    return image;
}

void Renderer::render_rows(const core::World &world, const Camera &camera, uint32_t vehicle_id, Image &image,
                           std::vector<int32_t> *surface, int row_begin, int row_end) const {
    // Height of the camera above the ground plane. NED z is down, so a camera
    // 60 m up has z = -60.
    const double height = -camera.position()[2];
    static const RasterScene kEmpty = [] {
        RasterScene s;
        s.build({});
        return s;
    }();
    const RasterScene &scene = appearance_ != nullptr ? *appearance_ : kEmpty;
    const Vec3 sun = sun_direction();

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
            //
            // Past the draw distance too: buildings stop there, the ground does not (it is
            // hazed instead), exactly as the rasteriser draws it.
            if (!hit.hit && ground_dist > 0.0) {
                hit.hit = true;
                hit.distance = ground_dist;
                hit.normal_ned = {0.0, 0.0, -1.0}; // ground faces up; NED up is -z
                hit.is_ground = true;              // it is the ground, so say so, or it draws grey
            }

            double colour[3];
            int32_t what = kSurfaceSky;
            if (!hit.hit) {
                sky_colour(dir, colour);
            } else {
                // Ground beyond 20 km is shaded as if it were there, as the rasteriser does: it is
                // fully filtered and hazed long before.
                const double d = hit.is_ground ? std::min(hit.distance, 20000.0) : hit.distance;
                Vec3 p{camera.position()[0] + dir[0] * d, camera.position()[1] + dir[1] * d,
                       camera.position()[2] + dir[2] * d};
                const double footprint = camera.footprint(dir, d, hit.normal_ned);
                const int tri = hit.is_ground ? -1 : scene.find_triangle(p);
                what = hit.is_ground ? kSurfaceGround : (tri >= 0 ? tri : kSurfaceUnknown);
                if (hit.is_ground) {
                    p[2] = 0.0;
                    scene.lit_colour(-1, p, footprint, colour);
                } else if (tri >= 0) {
                    scene.lit_colour(tri, p, footprint, colour);
                } else {
                    // A building the appearance scene does not know: plain, sun-lit grey.
                    const double lambert = std::fabs(hit.normal_ned[0] * sun[0] + hit.normal_ned[1] * sun[1] +
                                                     hit.normal_ned[2] * sun[2]);
                    for (int c = 0; c < 3; ++c) {
                        colour[c] = 0.6 * (kSkyLight[c] + kSunLight[c] * lambert);
                    }
                }
                const double haze = haze_at(hit.distance, cfg_.max_range_m, cfg_.haze_start);
                for (int c = 0; c < 3; ++c) {
                    colour[c] = mix(colour[c], kSkyHorizon[c], haze);
                }
            }

            if (surface != nullptr) {
                (*surface)[static_cast<size_t>(y) * cfg_.width + static_cast<size_t>(x)] = what;
            }
            const size_t i = (static_cast<size_t>(y) * cfg_.width + x) * 3;
            for (int c = 0; c < 3; ++c) {
                image.rgb[i + c] = to_byte(colour[c]);
            }
        }
    }
}

} // namespace skysim::render
