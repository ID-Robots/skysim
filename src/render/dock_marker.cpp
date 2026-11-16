#include "render/dock_marker.h"

// The marker image is a PNG committed with the source and compiled in (CMakeLists.txt), so it is
// decoded from memory: PNG only, no stdio. Static, so a test that brings its own stb_image (a JPEG
// decoder, tests/test_still.cpp) links against this library without two of every symbol.
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_STATIC
#define STBI_ONLY_PNG
#define STBI_NO_STDIO
#include <stb_image.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <utility>

namespace skysim::render {

// The image's bytes, generated from assets/markers/ at configure time (CMakeLists.txt).
extern const unsigned char kDockMarkerPng[];
extern const size_t kDockMarkerPngSize;

namespace {

// The part of [x - w/2, x + w/2] inside [lo, hi], as a fraction of w: a box-filtered edge.
double coverage(double x, double lo, double hi, double w) {
    const double c = (std::min(hi, x + w * 0.5) - std::max(lo, x - w * 0.5)) / w;
    return c > 0.0 ? (c < 1.0 ? c : 1.0) : 0.0;
}

// How each destination texel of a block `d` texels wide starting at `origin` (continuous texel
// coordinates) draws on a source of `n` texels stretched over it: per destination texel, the
// source texels it overlaps and by how much (in destination texels; what is left of the texel
// lies outside the block).
struct Tap {
    int src;
    double weight;
};
std::vector<std::vector<Tap>> resample_taps(double origin, double d, int n, int &first) {
    first = static_cast<int>(std::floor(origin));
    const int last = static_cast<int>(std::ceil(origin + d)) - 1;
    const double scale = n / d; // source texels per destination texel
    std::vector<std::vector<Tap>> taps;
    for (int k = first; k <= last; ++k) {
        std::vector<Tap> row;
        const double a = std::max(static_cast<double>(k), origin);
        const double b = std::min(static_cast<double>(k + 1), origin + d);
        if (b > a) {
            const double sa = (a - origin) * scale;
            const double sb = (b - origin) * scale;
            const int i0 = std::clamp(static_cast<int>(std::floor(sa)), 0, n - 1);
            const int i1 = std::clamp(static_cast<int>(std::ceil(sb)) - 1, 0, n - 1);
            for (int i = i0; i <= i1; ++i) {
                const double w = (std::min(sb, i + 1.0) - std::max(sa, static_cast<double>(i))) / scale;
                if (w > 0.0) {
                    row.push_back({i, w});
                }
            }
        }
        taps.push_back(std::move(row));
    }
    return taps;
}

} // namespace

const uint8_t *dock_marker_png() { return kDockMarkerPng; }
size_t dock_marker_png_size() { return kDockMarkerPngSize; }

std::shared_ptr<const MarkerTexture> MarkerTexture::build(const uint8_t *grey, int width, int height,
                                                          const DockMarkerSpec &spec, std::string &error) {
    constexpr int kCells = DockMarkerSpec::kCells;
    error = spec.invalid();
    if (!error.empty()) {
        return nullptr;
    }
    if (grey == nullptr || width != height || width < kCells * 4 || width > 16384) {
        error = "the marker image must be square, 36 to 16384 px";
        return nullptr;
    }
    const auto at = [&](int x, int y) {
        return grey[static_cast<size_t>(y) * width + static_cast<size_t>(x)];
    };
    const auto dark = [&](int x, int y) {
        return x >= 0 && y >= 0 && x < width && y < height && at(x, y) < 128;
    };

    // The outer marker is the bounding box of everything dark: its black border is solid.
    int x0 = width;
    int x1 = -1;
    int y0 = height;
    int y1 = -1;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (at(x, y) < 128) {
                x0 = std::min(x0, x);
                x1 = std::max(x1, x);
                y0 = std::min(y0, y);
                y1 = std::max(y1, y);
            }
        }
    }
    const int side = x1 - x0 + 1;
    if (x1 < 0 || side != y1 - y0 + 1 || side < kCells * 4) {
        error = "the marker image has no square black marker at least 36 px across";
        return nullptr;
    }

    // The generator's own arithmetic (generate_e_aruco_marker): a cell is side // 9; the inner
    // marker is int(cell * 0.8) px on a white square one cell across in the centre cell (row and
    // column 4), (cell - inner) // 2 px in from its edges.
    const int cell = side / kCells;
    const int inner = static_cast<int>(cell * 0.8);
    const int border = (cell - inner) / 2;
    const int cx0 = x0 + 4 * cell;
    const int cy0 = y0 + 4 * cell;
    const int ix0 = cx0 + border;
    const int iy0 = cy0 + border;
    const int mid = inner / 2;
    // What makes it an e-ArUco rather than a plain ArUco: an inner marker whose black square has
    // a white ring around it.
    const bool nested = border >= 1 && dark(ix0, iy0) && dark(ix0 + inner - 1, iy0) &&
                        dark(ix0, iy0 + inner - 1) && dark(ix0 + inner - 1, iy0 + inner - 1) &&
                        !dark(ix0 - 1, iy0 + mid) && !dark(ix0 + inner, iy0 + mid) &&
                        !dark(ix0 + mid, iy0 - 1) && !dark(ix0 + mid, iy0 + inner);
    if (!nested) {
        error =
            "the marker image has no inner marker on a white cell at its centre (not a generator e-ArUco)";
        return nullptr;
    }

    auto tex = std::shared_ptr<MarkerTexture>(new MarkerTexture());
    tex->spec_ = spec;
    const double ppm = side / spec.outer_size_m;
    tex->texels_per_m_ = ppm;
    tex->centre_x_ = x0 + side * 0.5;
    tex->centre_y_ = y0 + side * 0.5;
    const double icx = ix0 + inner * 0.5;
    const double icy = iy0 + inner * 0.5;
    DockLayout &l = tex->layout_;
    l.outer_size_m = spec.outer_size_m;
    l.inner_size_m = spec.inner_size_m;
    l.cell_size_m = cell / ppm;
    l.plate_u_min = -tex->centre_x_ / ppm;
    l.plate_u_max = (width - tex->centre_x_) / ppm;
    l.plate_v_min = -tex->centre_y_ / ppm;
    l.plate_v_max = (height - tex->centre_y_) / ppm;
    l.inner_u_m = (icx - tex->centre_x_) / ppm;
    l.inner_v_m = (icy - tex->centre_y_) / ppm;

    // The inner marker at its configured size, about its own centre, must stay on its white cell.
    const double d = spec.inner_size_m * ppm;
    const double bx0 = icx - d * 0.5;
    const double by0 = icy - d * 0.5;
    if (bx0 < cx0 || by0 < cy0 || bx0 + d > cx0 + cell || by0 + d > cy0 + cell) {
        error = "the inner marker does not fit on its white cell at that size";
        return nullptr;
    }

    // Level 0: the image on white, padded to a power of two so every mip level halves exactly.
    int n = 1;
    while (n < width) {
        n *= 2;
    }
    std::vector<uint8_t> base(static_cast<size_t>(n) * n, 255);
    for (int y = 0; y < height; ++y) {
        std::copy_n(grey + static_cast<size_t>(y) * width, width,
                    base.begin() + static_cast<ptrdiff_t>(y) * n);
    }
    // The centre cell redrawn: white, with the inner marker stretched over [bx0, bx0 + d) by
    // exact area weights (what a texel of the stretched picture integrates).
    for (int y = cy0; y < cy0 + cell; ++y) {
        std::fill_n(base.begin() + static_cast<ptrdiff_t>(y) * n + cx0, cell, uint8_t{255});
    }
    int fx = 0;
    int fy = 0;
    const auto tx = resample_taps(bx0, d, inner, fx);
    const auto ty = resample_taps(by0, d, inner, fy);
    for (size_t j = 0; j < ty.size(); ++j) {
        double wy_sum = 0.0;
        for (const Tap &b : ty[j]) {
            wy_sum += b.weight;
        }
        for (size_t i = 0; i < tx.size(); ++i) {
            double sum = 0.0;
            double wx_sum = 0.0;
            for (const Tap &a : tx[i]) {
                wx_sum += a.weight;
                for (const Tap &b : ty[j]) {
                    sum += a.weight * b.weight * at(ix0 + a.src, iy0 + b.src);
                }
            }
            sum += (1.0 - wx_sum * wy_sum) * 255.0; // the part of the texel off the block is white
            const int X = fx + static_cast<int>(i);
            const int Y = fy + static_cast<int>(j);
            base[static_cast<size_t>(Y) * n + static_cast<size_t>(X)] =
                static_cast<uint8_t>(std::clamp(std::lround(sum), 0L, 255L));
        }
    }

    tex->sizes_.push_back(n);
    tex->levels_.push_back(std::move(base));
    while (tex->sizes_.back() > 1) {
        const int s = tex->sizes_.back();
        const int h = s / 2;
        const std::vector<uint8_t> &src = tex->levels_.back();
        std::vector<uint8_t> next(static_cast<size_t>(h) * h);
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < h; ++x) {
                const size_t a = static_cast<size_t>(2 * y) * s + static_cast<size_t>(2 * x);
                next[static_cast<size_t>(y) * h + x] =
                    static_cast<uint8_t>((src[a] + src[a + 1] + src[a + s] + src[a + s + 1] + 2) / 4);
            }
        }
        tex->sizes_.push_back(h);
        tex->levels_.push_back(std::move(next));
    }
    return tex;
}

std::shared_ptr<const MarkerTexture> MarkerTexture::from_png(const uint8_t *png, size_t size,
                                                             const DockMarkerSpec &spec, std::string &error) {
    if (png == nullptr || size == 0 || size > 0x7fffffff) {
        error = "no marker image";
        return nullptr;
    }
    int w = 0;
    int h = 0;
    int channels = 0;
    stbi_uc *pixels = stbi_load_from_memory(png, static_cast<int>(size), &w, &h, &channels, 1);
    if (pixels == nullptr) {
        error = std::string("the marker image is not a readable PNG: ") + stbi_failure_reason();
        return nullptr;
    }
    auto tex = build(pixels, w, h, spec, error);
    stbi_image_free(pixels);
    return tex;
}

uint8_t MarkerTexture::texel(int x, int y) const {
    const int n = size();
    if (x < 0 || y < 0 || x >= n || y >= n) {
        return 255;
    }
    return levels_[0][static_cast<size_t>(y) * n + static_cast<size_t>(x)];
}

double MarkerTexture::box(size_t level, double x, double y, double width) const {
    const int n = sizes_[level];
    const std::vector<uint8_t> &t = levels_[level];
    const double h = width * 0.5;
    const double ax = x - h;
    const double bx = x + h;
    const double ay = y - h;
    const double by = y + h;
    const int i0 = std::max(static_cast<int>(std::floor(ax)), 0);
    const int i1 = std::min(static_cast<int>(std::floor(bx)), n - 1);
    const int j0 = std::max(static_cast<int>(std::floor(ay)), 0);
    const int j1 = std::min(static_cast<int>(std::floor(by)), n - 1);
    double sum = 0.0;
    double area = 0.0;
    for (int j = j0; j <= j1; ++j) {
        const double wy = std::min(by, j + 1.0) - std::max(ay, static_cast<double>(j));
        if (wy <= 0.0) {
            continue;
        }
        for (int i = i0; i <= i1; ++i) {
            const double wx = std::min(bx, i + 1.0) - std::max(ax, static_cast<double>(i));
            if (wx <= 0.0) {
                continue;
            }
            sum += wx * wy * t[static_cast<size_t>(j) * n + static_cast<size_t>(i)];
            area += wx * wy;
        }
    }
    // Whatever of the box lies off the texture is the white the image is padded with.
    const double total = width * width;
    return (sum + std::max(total - area, 0.0) * 255.0) / total;
}

double MarkerTexture::sample(double u_m, double v_m, double footprint_m) const {
    const double x = centre_x_ + u_m * texels_per_m_;
    const double y = centre_y_ + v_m * texels_per_m_;
    if (!std::isfinite(x) || !std::isfinite(y)) {
        return 1.0;
    }
    // The footprint in level-0 texels, and the finest level whose texels are no smaller than a
    // pixel: the box then spans 1 to 2 of its texels (under 1 on level 0, close up).
    double w = footprint_m * texels_per_m_;
    w = w > 1e-3 ? w : 1e-3;
    const int top = static_cast<int>(levels_.size()) - 1;
    const int level = w >= 1.0 ? std::min(static_cast<int>(std::floor(std::log2(w))), top) : 0;
    const double s = std::ldexp(1.0, -level);
    return box(static_cast<size_t>(level), x * s, y * s, w * s) / 255.0;
}

std::shared_ptr<const MarkerTexture> dock_marker_texture(const DockMarkerSpec &spec) {
    static std::mutex mutex;
    static std::map<std::pair<double, double>, std::shared_ptr<const MarkerTexture>> cache;
    std::lock_guard<std::mutex> lock(mutex);
    const std::pair<double, double> key{spec.outer_size_m, spec.inner_size_m};
    const auto it = cache.find(key);
    if (it != cache.end()) {
        return it->second;
    }
    std::string error;
    const auto t0 = std::chrono::steady_clock::now();
    auto tex = MarkerTexture::from_png(kDockMarkerPng, kDockMarkerPngSize, spec, error);
    if (tex) {
        // Once per process (per size): the moment a camera first had a marker in shot, or the
        // ground truth was first asked for. Never at startup, never while nobody looks.
        size_t bytes = 0;
        for (size_t l = 0; l < tex->levels(); ++l) {
            const size_t side = static_cast<size_t>(tex->size()) >> l;
            bytes += side * side;
        }
        std::printf("skysim: dock marker texture built (%dx%d, %zu levels, %.1f MB) in %.0f ms\n",
                    tex->size(), tex->size(), tex->levels(), static_cast<double>(bytes) / 1e6,
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
        std::fflush(stdout);
    } else {
        // Said once per size, and cached as a failure so a bad image is not decoded every frame.
        std::fprintf(stderr, "skysim: dock marker %.3f/%.3f m is not drawn: %s\n", spec.outer_size_m,
                     spec.inner_size_m, error.c_str());
    }
    cache.emplace(key, tex);
    return tex;
}

MarkerView::MarkerView(const GroundMarkers *markers, const Camera &camera) {
    if (markers == nullptr || markers->empty()) {
        return;
    }
    const Vec3 &eye = camera.position();
    if (!(eye[2] < 0.0)) {
        return; // at or below the ground plane: no ground in shot
    }
    const CameraConfig &cfg = camera.config();
    for (const GroundMarker &m : *markers) {
        // Out of draw range before any texture is touched: a marker a kilometre away is a fraction
        // of a pixel, and a frame that sees none must not build the texture.
        const double dn = m.pose.north_m - eye[0];
        const double de = m.pose.east_m - eye[1];
        const double reach = cfg.max_range_m + m.spec.outer_size_m * 2.0;
        if (dn * dn + de * de + eye[2] * eye[2] > reach * reach) {
            continue;
        }
        std::shared_ptr<const MarkerTexture> tex = m.texture ? m.texture : dock_marker_texture(m.spec);
        if (!tex) {
            continue;
        }
        const DockLayout &l = tex->layout();
        const double us[4] = {l.plate_u_min, l.plate_u_max, l.plate_u_max, l.plate_u_min};
        const double vs[4] = {l.plate_v_min, l.plate_v_min, l.plate_v_max, l.plate_v_max};
        double min_x = 1e18;
        double max_x = -1e18;
        double min_y = 1e18;
        double max_y = -1e18;
        int in_front = 0;
        for (int k = 0; k < 4; ++k) {
            const auto ne = dock_to_ned(m.pose, us[k], vs[k]);
            double px = 0.0;
            double py = 0.0;
            if (camera.project({ne[0], ne[1], 0.0}, px, py)) {
                ++in_front;
                min_x = std::min(min_x, px);
                max_x = std::max(max_x, px);
                min_y = std::min(min_y, py);
                max_y = std::max(max_y, py);
            }
        }
        if (in_front == 0) {
            continue; // all of it behind the camera
        }
        Visible v{tex.get(), m.pose, 0, cfg.width - 1, 0, cfg.height - 1};
        if (in_front == 4) {
            // The plate's corners all in front: its picture lies inside their box (perspective
            // keeps a flat quad convex). Two pixels of slack for the anti-aliased edge.
            if (max_x < -2.0 || min_x > cfg.width + 2.0 || max_y < -2.0 || min_y > cfg.height + 2.0) {
                continue; // off screen
            }
            // Clamped to the screen before the casts: a corner just in front of the camera
            // projects arbitrarily far out, past what an int holds.
            const auto px = [](double at, int size) { return std::clamp(at, -4.0, size + 4.0); };
            v.x0 = std::max(static_cast<int>(std::floor(px(min_x, cfg.width))) - 2, 0);
            v.x1 = std::min(static_cast<int>(std::ceil(px(max_x, cfg.width))) + 2, cfg.width - 1);
            v.y0 = std::max(static_cast<int>(std::floor(px(min_y, cfg.height))) - 2, 0);
            v.y1 = std::min(static_cast<int>(std::ceil(px(max_y, cfg.height))) + 2, cfg.height - 1);
        }
        // Some corners behind the camera (a plate under a low, tilted camera): any pixel may show it.
        visible_.push_back(v);
        textures_.push_back(std::move(tex));
    }
}

double MarkerView::albedo(int x, int y, double north_m, double east_m, double footprint_m,
                          double &albedo) const {
    const double fp = footprint_m > 1e-6 ? footprint_m : 1e-6;
    for (const Visible &v : visible_) {
        if (x < v.x0 || x > v.x1 || y < v.y0 || y > v.y1) {
            continue;
        }
        const auto uv = ned_to_dock(v.pose, north_m, east_m);
        const DockLayout &l = v.texture->layout();
        const double cu = coverage(uv[0], l.plate_u_min, l.plate_u_max, fp);
        if (cu <= 0.0) {
            continue;
        }
        const double cv = coverage(uv[1], l.plate_v_min, l.plate_v_max, fp);
        if (cv <= 0.0) {
            continue;
        }
        albedo = kMarkerBlack + (kMarkerWhite - kMarkerBlack) * v.texture->sample(uv[0], uv[1], fp);
        return cu * cv;
    }
    return 0.0;
}

void DockBoard::add(uint32_t vehicle_id, double home_north_m, double home_east_m) {
    std::lock_guard<std::mutex> lock(mutex_);
    Entry e;
    e.home = DockPose{home_north_m, home_east_m, 0.0};
    e.pose = e.home;
    e.at_home = true;
    entries_[vehicle_id] = e;
    publish_locked();
}

void DockBoard::remove(uint32_t vehicle_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (entries_.erase(vehicle_id) != 0) {
        publish_locked();
    }
}

std::optional<DockBoard::Entry> DockBoard::get(uint32_t vehicle_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = entries_.find(vehicle_id);
    if (it == entries_.end()) {
        return std::nullopt;
    }
    return it->second;
}

std::optional<DockBoard::Entry> DockBoard::set(uint32_t vehicle_id, const DockPose &pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = entries_.find(vehicle_id);
    if (it == entries_.end()) {
        return std::nullopt;
    }
    it->second.pose = pose;
    it->second.at_home = false;
    publish_locked();
    return it->second;
}

std::optional<DockBoard::Entry> DockBoard::update(uint32_t vehicle_id, const DockPatch &patch) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = entries_.find(vehicle_id);
    if (it == entries_.end()) {
        return std::nullopt;
    }
    if (patch.empty()) {
        return it->second;
    }
    DockPose &p = it->second.pose;
    p.north_m = patch.north_m.value_or(p.north_m);
    p.east_m = patch.east_m.value_or(p.east_m);
    p.yaw_rad = patch.yaw_rad.value_or(p.yaw_rad);
    it->second.at_home = false;
    publish_locked();
    return it->second;
}

std::optional<DockBoard::Entry> DockBoard::reset(uint32_t vehicle_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = entries_.find(vehicle_id);
    if (it == entries_.end()) {
        return std::nullopt;
    }
    it->second.pose = it->second.home;
    it->second.at_home = true;
    publish_locked();
    return it->second;
}

std::shared_ptr<const GroundMarkers> DockBoard::markers() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return markers_;
}

void DockBoard::publish_locked() {
    auto list = std::make_shared<GroundMarkers>();
    list->reserve(entries_.size());
    for (const auto &[id, e] : entries_) {
        list->push_back(GroundMarker{id, e.pose, spec_, nullptr});
    }
    markers_ = std::move(list);
}

} // namespace skysim::render
