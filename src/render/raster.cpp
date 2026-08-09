#include "render/raster.h"

#include "render/shading.h"

#include <algorithm>
#include <cmath>

namespace skysim::render {
namespace {

using core::Vec3;

Vec3 sub(const Vec3 &a, const Vec3 &b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }

Vec3 cross(const Vec3 &a, const Vec3 &b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

double dot(const Vec3 &a, const Vec3 &b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

Vec3 normalize(const Vec3 &v) {
    const double n = std::sqrt(dot(v, v));
    return n > 0.0 ? Vec3{v[0] / n, v[1] / n, v[2] / n} : Vec3{0.0, 0.0, -1.0};
}

} // namespace

void RasterScene::build(std::vector<core::World::Triangle> triangles) {
    tris_ = std::move(triangles);
    normals_.clear();
    normals_.reserve(tris_.size());
    cx_.clear();
    cy_.clear();
    cz_.clear();
    radius_.clear();
    cx_.reserve(tris_.size());
    cy_.reserve(tris_.size());
    cz_.reserve(tris_.size());
    radius_.reserve(tris_.size());
    for (const auto &t : tris_) {
        normals_.push_back(normalize(cross(sub(t.b, t.a), sub(t.c, t.a))));
        const core::Vec3 c{(t.a[0] + t.b[0] + t.c[0]) / 3.0, (t.a[1] + t.b[1] + t.c[1]) / 3.0,
                           (t.a[2] + t.b[2] + t.c[2]) / 3.0};
        cx_.push_back(static_cast<float>(c[0]));
        cy_.push_back(static_cast<float>(c[1]));
        cz_.push_back(static_cast<float>(c[2]));
        const Vec3 ra = sub(t.a, c), rb = sub(t.b, c), rc = sub(t.c, c);
        radius_.push_back(
            static_cast<float>(std::sqrt(std::max({dot(ra, ra), dot(rb, rb), dot(rc, rc)}))));
    }
}

Image Rasterizer::render(const RasterScene &scene, const Camera &camera, core::ThreadPool *pool) const {
    Image image;
    image.width = cfg_.width;
    image.height = cfg_.height;
    image.rgb.assign(static_cast<size_t>(cfg_.width) * cfg_.height * 3, 0);

    std::vector<ScreenTriangle> tris;
    transform(scene, camera, tris);

    // One band per worker. Each band's pixels belong to exactly one thread, so
    // the depth buffer needs no synchronisation.
    const int bands = pool != nullptr ? std::max(1, std::min(cfg_.height, 16)) : 1;
    const auto do_band = [&](int band) {
        const int y0 = cfg_.height * band / bands;
        const int y1 = cfg_.height * (band + 1) / bands;
        if (y0 >= y1) {
            return;
        }
        draw_background(camera, image, y0, y1);
        std::vector<float> depth(static_cast<size_t>(cfg_.width) * (y1 - y0), cfg_.max_range_m);
        fill_band(tris, image, depth, y0, y1);
    };

    if (pool != nullptr) {
        pool->parallel_for(static_cast<size_t>(bands), [&](size_t begin, size_t end) {
            for (size_t b = begin; b < end; ++b) {
                do_band(static_cast<int>(b));
            }
        });
    } else {
        do_band(0);
    }

    return image;
}

void Rasterizer::transform(const RasterScene &scene, const Camera &camera, std::vector<ScreenTriangle> &out) const {
    const Vec3 eye = camera.position();
    // Camera basis in world space, taken from the rays the camera already knows
    // how to make: centre ray is forward, and the frame is completed from the
    // horizontal and vertical edges of the image.
    const Vec3 fwd = camera.ray(cfg_.width / 2, cfg_.height / 2);
    const Vec3 right = normalize(sub(camera.ray(cfg_.width - 1, cfg_.height / 2), camera.ray(0, cfg_.height / 2)));
    const Vec3 down = normalize(sub(camera.ray(cfg_.width / 2, cfg_.height - 1), camera.ray(cfg_.width / 2, 0)));

    const double half_w = cfg_.width * 0.5;
    const double half_h = cfg_.height * 0.5;
    const double tan_half_x = std::tan((cfg_.fov_deg * M_PI / 180.0) * 0.5);
    const double focal = half_w / tan_half_x;

    const double range_sq = cfg_.max_range_m * cfg_.max_range_m;
    const auto &tris = scene.triangles();
    const auto &normals = scene.normals();
    out.clear();
    out.reserve(1024);

    // Read straight off the packed centroid arrays: this reject runs over the
    // whole world every frame, so it is the one loop where the memory layout
    // matters more than the arithmetic.
    const float *cx = scene.centroid_x().data();
    const float *cy = scene.centroid_y().data();
    const float *cz = scene.centroid_z().data();
    const float *radius = scene.radius().data();
    const auto eye_x = static_cast<float>(eye[0]);
    const auto eye_y = static_cast<float>(eye[1]);
    const auto eye_z = static_cast<float>(eye[2]);
    const auto range_f = static_cast<float>(std::sqrt(range_sq));

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
        ScreenTriangle st{};
        bool behind = false;
        for (int k = 0; k < 3; ++k) {
            const Vec3 &p = k == 0 ? t.a : (k == 1 ? t.b : t.c);
            const Vec3 v = sub(p, eye);
            const double z = dot(v, fwd);
            if (z <= 0.25) { // near plane; a proper clip is not worth it at this scale
                behind = true;
                break;
            }
            st.x[k] = static_cast<float>(half_w + dot(v, right) * focal / z);
            st.y[k] = static_cast<float>(half_h + dot(v, down) * focal / z);
            st.inv_w[k] = static_cast<float>(1.0 / z);
        }
        if (behind) {
            continue;
        }

        // Off-screen reject.
        const float min_x = std::min({st.x[0], st.x[1], st.x[2]});
        const float max_x = std::max({st.x[0], st.x[1], st.x[2]});
        const float min_y = std::min({st.y[0], st.y[1], st.y[2]});
        const float max_y = std::max({st.y[0], st.y[1], st.y[2]});
        if (max_x < 0 || min_x >= cfg_.width || max_y < 0 || min_y >= cfg_.height) {
            continue;
        }

        const Vec3 &n = normals[i];
        // Two-sided: a tile's winding is not guaranteed to face the camera, and a
        // building lit from the inside out looks like a hole in the world.
        st.shade = static_cast<float>(shade_for_normal(n));
        st.is_ground = t.is_ground;
        st.min_y = std::max(0, static_cast<int>(std::floor(min_y)));
        st.max_y = std::min(cfg_.height - 1, static_cast<int>(std::ceil(max_y)));
        out.push_back(st);
    }
}

void Rasterizer::draw_background(const Camera &camera, Image &image, int y0, int y1) const {
    for (int y = y0; y < y1; ++y) {
        for (int x = 0; x < cfg_.width; ++x) {
            const Vec3 dir = camera.ray(x, y);

            // Below the horizon the background is ground haze, so terrain the
            // triangles do not cover does not read as sky.
            const bool below = dir[2] > 0.0;
            const double *far = below ? kGround : kSkyZenith;
            // Below the horizon this is only seen past the draw distance, so it
            // sits close to the haze colour and lets the drawn ground stand out.
            const double up = std::clamp(-dir[2], 0.0, 1.0);
            const double blend = below ? 0.72 : std::pow(up, 0.65);
            const size_t i = (static_cast<size_t>(y) * cfg_.width + x) * 3;
            for (int c = 0; c < 3; ++c) {
                image.rgb[i + c] = to_byte(mix(kSkyHorizon[c], far[c], blend));
            }
        }
    }
}

void Rasterizer::fill_band(const std::vector<ScreenTriangle> &tris, Image &image, std::vector<float> &depth,
                           int y0, int y1) const {
    // Per-axis tangent offsets, so a pixel's camera-space depth can be turned
    // into its radial distance with two adds and a multiply. Squared, because
    // the only consumer squares them again.
    const double tan_x = std::tan((cfg_.fov_deg * M_PI / 180.0) * 0.5);
    const double tan_y = tan_x * cfg_.height / cfg_.width;
    const double max_range_sq = cfg_.max_range_m * cfg_.max_range_m;

    std::vector<double> kx(static_cast<size_t>(cfg_.width));
    for (int x = 0; x < cfg_.width; ++x) {
        const double sx = ((x + 0.5) / cfg_.width) * 2.0 - 1.0;
        kx[static_cast<size_t>(x)] = (sx * tan_x) * (sx * tan_x);
    }

    for (const ScreenTriangle &t : tris) {
        if (t.max_y < y0 || t.min_y >= y1) {
            continue; // not in this band
        }

        const float area = (t.x[1] - t.x[0]) * (t.y[2] - t.y[0]) - (t.x[2] - t.x[0]) * (t.y[1] - t.y[0]);
        if (std::abs(area) < 1e-6f) {
            continue;
        }
        const float inv_area = 1.0f / area;

        const int ys = std::max(y0, t.min_y);
        const int ye = std::min(y1 - 1, t.max_y);
        const int xs = std::max(0, static_cast<int>(std::floor(std::min({t.x[0], t.x[1], t.x[2]}))));
        const int xe = std::min(cfg_.width - 1, static_cast<int>(std::ceil(std::max({t.x[0], t.x[1], t.x[2]}))));

        for (int y = ys; y <= ye; ++y) {
            const float py = y + 0.5f;
            const double sy = 1.0 - ((y + 0.5) / cfg_.height) * 2.0;
            const double ky_row = (sy * tan_y) * (sy * tan_y);
            for (int x = xs; x <= xe; ++x) {
                const float px = x + 0.5f;

                // Barycentric coverage test.
                const float w0 = ((t.x[1] - px) * (t.y[2] - py) - (t.x[2] - px) * (t.y[1] - py)) * inv_area;
                const float w1 = ((t.x[2] - px) * (t.y[0] - py) - (t.x[0] - px) * (t.y[2] - py)) * inv_area;
                const float w2 = 1.0f - w0 - w1;
                if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f) {
                    continue;
                }

                // Perspective-correct depth.
                const float inv_w = w0 * t.inv_w[0] + w1 * t.inv_w[1] + w2 * t.inv_w[2];
                if (inv_w <= 0.0f) {
                    continue;
                }
                const float z = 1.0f / inv_w;

                const size_t di = static_cast<size_t>(y - y0) * cfg_.width + x;
                if (z >= depth[di]) {
                    continue;
                }
                depth[di] = z;

                const double *base = t.is_ground ? kGround : kBuilding;
                // Haze grows with distance but never fully erases a surface:
                // capped, so the far edge of the draw distance reads as far away
                // rather than as blank sky.
                //
                // Against radial distance, not z. z is depth along the camera's
                // forward axis, and at the edge of a 78 degree frame that is
                // ~25% short of how far the surface actually is — enough that the
                // same wall hazed differently here than in the ray caster, which
                // shading.h says must not happen. Squared throughout, so the
                // conversion costs a multiply instead of a square root per pixel.
                const double radial_sq = static_cast<double>(z) * z * (1.0 + kx[x] + ky_row);
                const double haze = haze_at_sq(radial_sq, max_range_sq);

                const size_t pi = (static_cast<size_t>(y) * cfg_.width + x) * 3;
                for (int c = 0; c < 3; ++c) {
                    const double lit = base[c] * t.shade;
                    image.rgb[pi + c] = to_byte(mix(lit, kSkyHorizon[c], haze));
                }
            }
        }
    }
}

} // namespace skysim::render
