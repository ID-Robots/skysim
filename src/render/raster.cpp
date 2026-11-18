#include "render/raster.h"

#include "render/shading.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <optional>

namespace skysim::render {
namespace {

using core::Vec3;

Vec3 sub(const Vec3 &a, const Vec3 &b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }

double dot(const Vec3 &a, const Vec3 &b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

// Near plane, metres in front of the camera. Geometry closer than this is clipped off.
constexpr double kNear = 0.25;

// Ground further than this is shaded as if it were this far: it is fully filtered and
// fully hazed long before, and a texture lookup at astronomic coordinates is not defined.
constexpr double kMaxGroundM = 20000.0;

struct CamVertex {
    double x, y, z; // camera space: right, down, forward
};

// One band of rows, owned by one thread.
struct Band {
    int y0;
    int y1;
    std::vector<float> depth;
    std::vector<int32_t> ids;
};

void fill(const std::vector<ScreenTriangle> &tris, int width, Band &band) {
    for (const ScreenTriangle &t : tris) {
        if (t.max_y < band.y0 || t.min_y >= band.y1) {
            continue;
        }
        const float area = (t.x[1] - t.x[0]) * (t.y[2] - t.y[0]) - (t.x[2] - t.x[0]) * (t.y[1] - t.y[0]);
        if (std::abs(area) < 1e-9f) {
            continue;
        }
        const float inv_area = 1.0f / area;
        const int ys = std::max(band.y0, t.min_y);
        const int ye = std::min(band.y1 - 1, t.max_y);
        const int xs = std::max(0, static_cast<int>(std::floor(std::min({t.x[0], t.x[1], t.x[2]}))));
        const int xe = std::min(width - 1, static_cast<int>(std::ceil(std::max({t.x[0], t.x[1], t.x[2]}))));
        for (int y = ys; y <= ye; ++y) {
            const float py = static_cast<float>(y) + 0.5f;
            for (int x = xs; x <= xe; ++x) {
                const float px = static_cast<float>(x) + 0.5f;
                const float w0 = ((t.x[1] - px) * (t.y[2] - py) - (t.x[2] - px) * (t.y[1] - py)) * inv_area;
                const float w1 = ((t.x[2] - px) * (t.y[0] - py) - (t.x[0] - px) * (t.y[2] - py)) * inv_area;
                const float w2 = 1.0f - w0 - w1;
                if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f) {
                    continue;
                }
                const float inv_w = w0 * t.inv_w[0] + w1 * t.inv_w[1] + w2 * t.inv_w[2];
                if (inv_w <= 0.0f) {
                    continue;
                }
                const float z = 1.0f / inv_w;
                const size_t i =
                    static_cast<size_t>(y - band.y0) * static_cast<size_t>(width) + static_cast<size_t>(x);
                // Ties go to the lower triangle index, so the winner never depends on the
                // order triangles arrive in.
                if (z < band.depth[i] || (z == band.depth[i] && static_cast<int32_t>(t.tri) < band.ids[i])) {
                    band.depth[i] = z;
                    band.ids[i] = static_cast<int32_t>(t.tri);
                }
            }
        }
    }
}

} // namespace

Image Rasterizer::render(const RasterScene &scene, const Camera &camera, core::ThreadPool *pool,
                         std::vector<int32_t> *surface, bool caller_runs, const GroundMarkers *markers,
                         GroundCache *ground_cache) const {
    Image image;
    std::vector<ScreenTriangle> tris;
    MarkerView marker_view;
    // Setup: the frame buffers and the pass over the whole world that picks what is in shot.
    // With caller_runs = false it runs on a pool worker like the rest, so the caller's thread
    // (an HTTP worker at normal priority) does none of the work.
    const auto setup = [&] {
        image.width = cfg_.width;
        image.height = cfg_.height;
        image.rgb.assign(static_cast<size_t>(cfg_.width) * cfg_.height * 3, 0);
        if (surface != nullptr) {
            surface->assign(static_cast<size_t>(cfg_.width) * cfg_.height, kSurfaceSky);
        }
        transform(scene, camera, tris);
        // The dock markers in shot, once a frame (a marker's texture is built here the first
        // time one is, so on a worker too).
        marker_view = MarkerView(markers, camera);
        if (ground_cache != nullptr) {
            ground_cache->begin_frame();
        }
    };
    if (pool != nullptr && !caller_runs) {
        pool->parallel_for(1, [&](size_t, size_t) { setup(); }, false);
    } else {
        setup();
    }

    const Vec3 eye = camera.position();
    const auto &normals = scene.normals();
    const auto &triangles = scene.triangles();

    // The pilot camera's ground fast path (with a ground cache, TASK-2661): a pixel no triangle
    // covers and that sees the ground plane is the commonest pixel of a camera looking down, and
    // for it the ray, the hit point and the footprint are closed forms of the pixel's position.
    // The ray, un-normalised, is forward + right * cx + down * cy (tangent-plane coordinates),
    // so its forward component is 1, and the ground (plane normal n = down, the camera h above
    // it) is hit at h / r_z along it. The footprint is Camera::footprint's for that plane, which
    // works out to (h / focal) * r_z^-1.5. The same numbers as the general path below to within
    // rounding; the stills, which have no cache, keep the general path exactly.
    const bool fast_ground = ground_cache != nullptr && eye[2] < 0.0;
    const double height_m = -eye[2];
    const double footprint_k = height_m / camera.focal_px();
    const Vec3 &fwd = camera.forward();
    const Vec3 &right = camera.right();
    const Vec3 &down = camera.down();
    const double tan_x = camera.tan_half_x();
    const double step_x = 2.0 / cfg_.width * tan_x;

    const auto shade_band = [&](Band &band, GroundCache::Sampler *sampler) {
        for (int y = band.y0; y < band.y1; ++y) {
            const double cy = ((y + 0.5) / cfg_.height * 2.0 - 1.0) * camera.tan_half_y();
            const double row[3] = {fwd[0] + down[0] * cy, fwd[1] + down[1] * cy, fwd[2] + down[2] * cy};
            for (int x = 0; x < cfg_.width; ++x) {
                const size_t bi = static_cast<size_t>(y - band.y0) * cfg_.width + static_cast<size_t>(x);
                if (fast_ground && sampler != nullptr && band.ids[bi] < 0) {
                    const double cx = (x + 0.5) * step_x - tan_x;
                    const double rx = row[0] + right[0] * cx;
                    const double ry = row[1] + right[1] * cx;
                    const double rz = row[2] + right[2] * cx;
                    const double rn = std::sqrt(rx * rx + ry * ry + rz * rz);
                    if (rz > 1e-9 * rn) {
                        const double inv = 1.0 / rz;
                        const double along = height_m * inv;
                        const double distance = along * rn;
                        if (distance <= kMaxGroundM) {
                            const double north = eye[0] + rx * along;
                            const double east = eye[1] + ry * along;
                            const double footprint = std::min(footprint_k * inv * std::sqrt(inv), 1e3);
                            double marker = 0.0;
                            const double covered =
                                marker_view.empty()
                                    ? 0.0
                                    : marker_view.albedo(x, y, north, east, footprint, marker);
                            if (!(covered > 0.0)) {
                                double colour[3];
                                sampler->lit(north, east, footprint, colour);
                                const double haze = haze_at(distance, cfg_.max_range_m, cfg_.haze_start);
                                const size_t pi =
                                    (static_cast<size_t>(y) * cfg_.width + static_cast<size_t>(x)) * 3;
                                for (int c = 0; c < 3; ++c) {
                                    image.rgb[pi + static_cast<size_t>(c)] =
                                        to_byte(mix(colour[c], kSkyHorizon[c], haze));
                                }
                                if (surface != nullptr) {
                                    (*surface)[static_cast<size_t>(y) * cfg_.width + static_cast<size_t>(x)] =
                                        kSurfaceGround;
                                }
                                continue;
                            }
                        }
                    }
                }
                const Vec3 dir = camera.ray(x, y);
                int32_t id = band.ids[bi];
                double t = -1.0;
                // The ground plane, intersected on its own: geometry below it (a basement in
                // a scanned tile, say) is underground and must lose to it, however the fill
                // went. Ties go to the mesh — a face lying on the ground is drawn as itself.
                const double ground_t = dir[2] > 1e-9 && eye[2] < 0.0 ? -eye[2] / dir[2] : -1.0;
                if (id >= 0) {
                    // Exact hit on the winning triangle's plane.
                    const Vec3 &n = normals[static_cast<size_t>(id)];
                    const double denom = dot(n, dir);
                    if (std::fabs(denom) > 1e-9) {
                        t = dot(n, sub(triangles[static_cast<size_t>(id)].a, eye)) / denom;
                    }
                    if (!(t > 0.0)) { // grazing: fall back to the filled depth
                        t = static_cast<double>(band.depth[bi]) / std::max(dot(dir, camera.forward()), 1e-9);
                    }
                    if (ground_t > 0.0 && ground_t < t * (1.0 - 1e-9)) {
                        id = kSurfaceGround;
                        t = ground_t;
                    }
                } else if (ground_t > 0.0) {
                    id = kSurfaceGround;
                    t = ground_t;
                } else {
                    id = kSurfaceSky;
                }

                double colour[3];
                if (id == kSurfaceSky) {
                    sky_colour(dir, colour);
                } else {
                    const double tt = id == kSurfaceGround ? std::min(t, kMaxGroundM) : t;
                    Vec3 p{eye[0] + dir[0] * tt, eye[1] + dir[1] * tt, eye[2] + dir[2] * tt};
                    if (id == kSurfaceGround) {
                        p[2] = 0.0; // on the plane, not a rounding error off it
                    }
                    const Vec3 normal =
                        id == kSurfaceGround ? Vec3{0.0, 0.0, -1.0} : normals[static_cast<size_t>(id)];
                    const double footprint = camera.footprint(dir, tt, normal);
                    double marker = 0.0;
                    // On the ground plane or on a ground mesh lying on it (a scanned tile's): a
                    // dock marker lies on either. A raised face hides it.
                    const double covered = !marker_view.empty() && scene.ground_level(id)
                                               ? marker_view.albedo(x, y, p[0], p[1], footprint, marker)
                                               : 0.0;
                    if (covered > 0.0) {
                        // A dock marker's plate: its albedo, blended with the ground's along the
                        // plate's edge, lit like the ground it lies on.
                        double ground[3] = {marker, marker, marker};
                        if (covered < 1.0) {
                            scene.albedo(id, p, footprint, ground);
                        }
                        for (int c = 0; c < 3; ++c) {
                            colour[c] = mix(ground[c], marker, covered);
                        }
                        scene.light(id, p, colour);
                    } else if (sampler != nullptr && id == kSurfaceGround) {
                        sampler->lit(p[0], p[1], footprint, colour);
                    } else {
                        scene.lit_colour(id, p, footprint, colour);
                    }
                    const double haze = haze_at(t, cfg_.max_range_m, cfg_.haze_start);
                    for (int c = 0; c < 3; ++c) {
                        colour[c] = mix(colour[c], kSkyHorizon[c], haze);
                    }
                }
                const size_t pi = (static_cast<size_t>(y) * cfg_.width + static_cast<size_t>(x)) * 3;
                for (int c = 0; c < 3; ++c) {
                    image.rgb[pi + static_cast<size_t>(c)] = to_byte(colour[c]);
                }
                if (surface != nullptr) {
                    (*surface)[static_cast<size_t>(y) * cfg_.width + static_cast<size_t>(x)] = id;
                }
            }
        }
    };

    // Eight rows a band, at most 256 bands: enough to balance a small pilot frame across the
    // workers, few enough that each band's scan of the screen-triangle list stays cheap.
    const int bands = pool != nullptr ? std::clamp(cfg_.height / 8, 1, 256) : 1;
    const auto do_band = [&](int b) {
        Band band;
        band.y0 = cfg_.height * b / bands;
        band.y1 = cfg_.height * (b + 1) / bands;
        if (band.y0 >= band.y1) {
            return;
        }
        const size_t n = static_cast<size_t>(cfg_.width) * static_cast<size_t>(band.y1 - band.y0);
        band.depth.assign(n, std::numeric_limits<float>::infinity());
        band.ids.assign(n, kSurfaceGround);
        fill(tris, cfg_.width, band);
        std::optional<GroundCache::Sampler> sampler;
        if (ground_cache != nullptr) {
            sampler.emplace(*ground_cache);
        }
        shade_band(band, sampler ? &*sampler : nullptr);
    };

    if (pool != nullptr) {
        const size_t workers = pool->size() + (caller_runs ? 1 : 0);
        std::atomic<int> next_band{0};
        pool->parallel_for(
            workers,
            [&](size_t begin, size_t end) {
                for (size_t w = begin; w < end; ++w) {
                    for (int b = next_band.fetch_add(1, std::memory_order_relaxed); b < bands;
                         b = next_band.fetch_add(1, std::memory_order_relaxed)) {
                        do_band(b);
                    }
                }
            },
            caller_runs);
    } else {
        do_band(0);
    }
    return image;
}

void Rasterizer::transform(const RasterScene &scene, const Camera &camera, std::vector<ScreenTriangle> &out) const {
    const Vec3 eye = camera.position();
    const Vec3 &fwd = camera.forward();
    const Vec3 &right = camera.right();
    const Vec3 &down = camera.down();
    const double half_w = cfg_.width * 0.5;
    const double half_h = cfg_.height * 0.5;
    const double focal = camera.focal_px();

    const auto &tris = scene.triangles();
    out.clear();
    out.reserve(1024);

    // Read straight off the packed centroid arrays: this reject runs over the whole world
    // every frame, so it is the one loop where the memory layout matters more than the
    // arithmetic.
    const float *cx = scene.centroid_x().data();
    const float *cy = scene.centroid_y().data();
    const float *cz = scene.centroid_z().data();
    const float *radius = scene.radius().data();
    const auto eye_x = static_cast<float>(eye[0]);
    const auto eye_y = static_cast<float>(eye[1]);
    const auto eye_z = static_cast<float>(eye[2]);
    const auto range_f = static_cast<float>(cfg_.max_range_m);

    const auto emit = [&](const CamVertex *v, uint32_t index) {
        ScreenTriangle st{};
        for (int k = 0; k < 3; ++k) {
            st.x[k] = static_cast<float>(half_w + v[k].x * focal / v[k].z);
            st.y[k] = static_cast<float>(half_h + v[k].y * focal / v[k].z);
            st.inv_w[k] = static_cast<float>(1.0 / v[k].z);
        }
        const float min_x = std::min({st.x[0], st.x[1], st.x[2]});
        const float max_x = std::max({st.x[0], st.x[1], st.x[2]});
        const float min_y = std::min({st.y[0], st.y[1], st.y[2]});
        const float max_y = std::max({st.y[0], st.y[1], st.y[2]});
        if (max_x < 0 || min_x >= cfg_.width || max_y < 0 || min_y >= cfg_.height) {
            return; // off screen
        }
        st.tri = index;
        st.min_y = std::max(0, static_cast<int>(std::floor(min_y)));
        st.max_y = std::min(cfg_.height - 1, static_cast<int>(std::ceil(max_y)));
        out.push_back(st);
    };

    for (size_t i = 0; i < tris.size(); ++i) {
        const float dx = cx[i] - eye_x;
        const float dy = cy[i] - eye_y;
        const float dz = cz[i] - eye_z;
        // Against the triangle's near edge, not its middle: see RasterScene::radius().
        const float limit = range_f + radius[i];
        if (dx * dx + dy * dy + dz * dz > limit * limit) {
            continue;
        }

        const auto &t = tris[i];
        CamVertex v[3];
        int in_front = 0;
        for (int k = 0; k < 3; ++k) {
            const Vec3 &p = k == 0 ? t.a : (k == 1 ? t.b : t.c);
            const Vec3 d = sub(p, eye);
            v[k] = {dot(d, right), dot(d, down), dot(d, fwd)};
            in_front += v[k].z >= kNear ? 1 : 0;
        }
        if (in_front == 3) {
            emit(v, static_cast<uint32_t>(i));
            continue;
        }
        if (in_front == 0) {
            continue;
        }
        // Clip against the near plane (Sutherland-Hodgman), leaving a triangle or a quad.
        // Dropping the whole triangle instead, as this used to, punched a hole in any wall
        // or roof that reached behind the camera — and in an oblique survey frame that is
        // most of the roofs along the bottom edge.
        CamVertex poly[4];
        int count = 0;
        for (int k = 0; k < 3; ++k) {
            const CamVertex &a = v[k];
            const CamVertex &b = v[(k + 1) % 3];
            const bool a_in = a.z >= kNear;
            const bool b_in = b.z >= kNear;
            if (a_in) {
                poly[count++] = a;
            }
            if (a_in != b_in) {
                const double s = (kNear - a.z) / (b.z - a.z);
                poly[count++] = {a.x + (b.x - a.x) * s, a.y + (b.y - a.y) * s, kNear};
            }
        }
        emit(poly, static_cast<uint32_t>(i));
        if (count == 4) {
            const CamVertex second[3] = {poly[0], poly[2], poly[3]};
            emit(second, static_cast<uint32_t>(i));
        }
    }
}

} // namespace skysim::render
