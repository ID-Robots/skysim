#include "render/scene.h"

#include "render/shading.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

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

const Vec3 &vertex(const core::World::Triangle &t, int k) { return k == 0 ? t.a : (k == 1 ? t.b : t.c); }

class UnionFind {
  public:
    explicit UnionFind(size_t n) : parent_(n) {
        for (size_t i = 0; i < n; ++i) {
            parent_[i] = static_cast<uint32_t>(i);
        }
    }
    uint32_t find(uint32_t x) {
        while (parent_[x] != x) {
            parent_[x] = parent_[parent_[x]];
            x = parent_[x];
        }
        return x;
    }
    // The smaller index becomes the root, so the result does not depend on union order.
    void unite(uint32_t a, uint32_t b) {
        a = find(a);
        b = find(b);
        if (a != b) {
            parent_[std::max(a, b)] = std::min(a, b);
        }
    }

  private:
    std::vector<uint32_t> parent_;
};

// Vertices are matched at millimetre resolution: tiles store floats, and two tiles' copies of
// a shared corner are not guaranteed to be bit-identical after the basis change.
struct VertexKey {
    int64_t x, y, z;
    bool operator==(const VertexKey &o) const { return x == o.x && y == o.y && z == o.z; }
    bool operator<(const VertexKey &o) const { return x != o.x ? x < o.x : (y != o.y ? y < o.y : z < o.z); }
};

struct VertexKeyHash {
    size_t operator()(const VertexKey &k) const {
        return static_cast<size_t>(hash_cell(k.x, k.y, static_cast<uint32_t>(k.z) ^ 0xA5A5A5A5U));
    }
};

VertexKey quantise(const Vec3 &v) {
    return {std::llround(v[0] * 1000.0), std::llround(v[1] * 1000.0), std::llround(v[2] * 1000.0)};
}

} // namespace

void RasterScene::build(std::vector<core::World::Triangle> triangles, uint32_t texture_seed) {
    tris_.clear();
    tris_.reserve(triangles.size());
    for (const auto &t : triangles) {
        if (!t.is_ground) {
            tris_.push_back(t);
        }
    }
    std::vector<core::World::Triangle>().swap(triangles);
    texture_ = WorldTexture(texture_seed);

    normals_.clear();
    cx_.clear();
    cy_.clear();
    cz_.clear();
    radius_.clear();
    normals_.reserve(tris_.size());
    cx_.reserve(tris_.size());
    cy_.reserve(tris_.size());
    cz_.reserve(tris_.size());
    radius_.reserve(tris_.size());
    for (const auto &t : tris_) {
        normals_.push_back(normalize(cross(sub(t.b, t.a), sub(t.c, t.a))));
        const Vec3 c{(t.a[0] + t.b[0] + t.c[0]) / 3.0, (t.a[1] + t.b[1] + t.c[1]) / 3.0,
                     (t.a[2] + t.b[2] + t.c[2]) / 3.0};
        cx_.push_back(static_cast<float>(c[0]));
        cy_.push_back(static_cast<float>(c[1]));
        cz_.push_back(static_cast<float>(c[2]));
        const Vec3 ra = sub(t.a, c), rb = sub(t.b, c), rc = sub(t.c, c);
        radius_.push_back(static_cast<float>(std::sqrt(std::max({dot(ra, ra), dot(rb, rb), dot(rc, rc)}))));
    }
    build_identity();
    build_grid();
}

void RasterScene::build_identity() {
    const size_t n = tris_.size();
    tri_building_.assign(n, 0);
    tri_frame_.assign(n, 0);
    tri_outline_.assign(n, 0);
    tri_altitude_.assign(n, {0.0F, 0.0F, 0.0F});
    frames_.clear();
    building_count_ = 0;
    if (n == 0) {
        return;
    }

    // Shared corners, by id.
    std::unordered_map<VertexKey, uint32_t, VertexKeyHash> ids;
    ids.reserve(n * 2);
    std::vector<VertexKey> keys;
    std::vector<std::array<uint32_t, 3>> tv(n);
    for (size_t i = 0; i < n; ++i) {
        for (int k = 0; k < 3; ++k) {
            const VertexKey key = quantise(vertex(tris_[i], k));
            const auto [it, inserted] = ids.try_emplace(key, static_cast<uint32_t>(keys.size()));
            if (inserted) {
                keys.push_back(key);
            }
            tv[i][static_cast<size_t>(k)] = it->second;
        }
    }

    // A building is everything connected through shared corners.
    UnionFind buildings(n);
    std::vector<uint32_t> first_tri(keys.size(), std::numeric_limits<uint32_t>::max());
    for (size_t i = 0; i < n; ++i) {
        for (const uint32_t v : tv[i]) {
            if (first_tri[v] == std::numeric_limits<uint32_t>::max()) {
                first_tri[v] = static_cast<uint32_t>(i);
            } else {
                buildings.unite(static_cast<uint32_t>(i), first_tri[v]);
            }
        }
    }

    // A face is everything connected through shared edges and lying in one plane; an edge no
    // coplanar neighbour shares is part of its face's outline.
    struct EdgeRef {
        uint64_t key;
        uint32_t tri;
        uint32_t k; // the vertex opposite this edge
    };
    std::vector<EdgeRef> edges;
    edges.reserve(n * 3);
    for (size_t i = 0; i < n; ++i) {
        for (uint32_t k = 0; k < 3; ++k) {
            const uint32_t a = tv[i][(k + 1) % 3];
            const uint32_t b = tv[i][(k + 2) % 3];
            const uint64_t key = (static_cast<uint64_t>(std::min(a, b)) << 32) | std::max(a, b);
            edges.push_back({key, static_cast<uint32_t>(i), k});
        }
    }
    std::sort(edges.begin(), edges.end(), [](const EdgeRef &x, const EdgeRef &y) {
        return x.key != y.key ? x.key < y.key : x.tri < y.tri;
    });
    const auto coplanar = [this](uint32_t i, uint32_t j) {
        return dot(normals_[i], normals_[j]) > 0.9999 &&
               std::fabs(dot(normals_[i], sub(tris_[j].a, tris_[i].a))) < 1e-3;
    };
    UnionFind faces(n);
    std::vector<uint8_t> interior(n, 0);
    for (size_t r0 = 0; r0 < edges.size();) {
        size_t r1 = r0 + 1;
        while (r1 < edges.size() && edges[r1].key == edges[r0].key) {
            ++r1;
        }
        for (size_t p = r0; p < r1; ++p) {
            for (size_t q = p + 1; q < r1; ++q) {
                if (coplanar(edges[p].tri, edges[q].tri)) {
                    faces.unite(edges[p].tri, edges[q].tri);
                    interior[edges[p].tri] |= static_cast<uint8_t>(1U << edges[p].k);
                    interior[edges[q].tri] |= static_cast<uint8_t>(1U << edges[q].k);
                }
            }
        }
        r0 = r1;
    }

    // Building keys from each building's lowest corner, so a building keeps its colours
    // whatever order its tiles were loaded in.
    std::vector<VertexKey> lowest(n, VertexKey{std::numeric_limits<int64_t>::max(), 0, 0});
    for (size_t i = 0; i < n; ++i) {
        const uint32_t root = buildings.find(static_cast<uint32_t>(i));
        for (const uint32_t v : tv[i]) {
            if (keys[v] < lowest[root]) {
                lowest[root] = keys[v];
            }
        }
    }
    for (size_t i = 0; i < n; ++i) {
        const uint32_t root = buildings.find(static_cast<uint32_t>(i));
        if (root == i) {
            ++building_count_;
        }
        const VertexKey &k = lowest[root];
        tri_building_[i] = hash32(hash_cell(k.x, k.y, 0x5EEDU) ^ static_cast<uint32_t>(k.z));
        tri_outline_[i] = static_cast<uint8_t>(~interior[i] & 7U);
    }

    // One frame per face: what kind of surface, and where it sits.
    constexpr double kBig = std::numeric_limits<double>::max();
    std::vector<uint32_t> frame_of_root(n, std::numeric_limits<uint32_t>::max());
    for (size_t i = 0; i < n; ++i) {
        const uint32_t root = faces.find(static_cast<uint32_t>(i));
        if (frame_of_root[root] == std::numeric_limits<uint32_t>::max()) {
            frame_of_root[root] = static_cast<uint32_t>(frames_.size());
            SurfaceFrame f;
            const Vec3 &nr = normals_[root];
            const double horiz = std::hypot(nr[0], nr[1]);
            f.kind = std::fabs(nr[2]) > 0.7 ? SurfaceFrame::Kind::kRoof : SurfaceFrame::Kind::kWall;
            if (horiz > 1e-9) {
                f.tangent_n = -nr[1] / horiz;
                f.tangent_e = nr[0] / horiz;
            }
            f.u_min = f.n_min = f.e_min = f.z_top = kBig;
            f.u_max = f.n_max = f.e_max = f.z_bottom = -kBig;
            frames_.push_back(f);
        }
        tri_frame_[i] = frame_of_root[root];
        SurfaceFrame &f = frames_[tri_frame_[i]];
        for (int k = 0; k < 3; ++k) {
            const Vec3 &v = vertex(tris_[i], k);
            const double u = v[0] * f.tangent_n + v[1] * f.tangent_e;
            f.u_min = std::min(f.u_min, u);
            f.u_max = std::max(f.u_max, u);
            f.z_top = std::min(f.z_top, v[2]);
            f.z_bottom = std::max(f.z_bottom, v[2]);
            f.n_min = std::min(f.n_min, v[0]);
            f.n_max = std::max(f.n_max, v[0]);
            f.e_min = std::min(f.e_min, v[1]);
            f.e_max = std::max(f.e_max, v[1]);
        }
    }
    // A horizontal face at ground level is a building's underside (or a slab nobody sees from
    // above), whichever way its winding happens to point.
    for (SurfaceFrame &f : frames_) {
        if (f.kind == SurfaceFrame::Kind::kRoof && -f.z_top < 0.05) {
            f.kind = SurfaceFrame::Kind::kUnderside;
        }
    }

    for (size_t i = 0; i < n; ++i) {
        const auto &t = tris_[i];
        const double area2 =
            std::sqrt(dot(cross(sub(t.b, t.a), sub(t.c, t.a)), cross(sub(t.b, t.a), sub(t.c, t.a))));
        for (int k = 0; k < 3; ++k) {
            const Vec3 &p = vertex(t, (k + 1) % 3);
            const Vec3 &q = vertex(t, (k + 2) % 3);
            const double len = std::sqrt(dot(sub(q, p), sub(q, p)));
            tri_altitude_[i][static_cast<size_t>(k)] = len > 0.0 ? static_cast<float>(area2 / len) : 0.0F;
        }
    }
}

void RasterScene::build_grid() {
    grid_ = Grid{};
    std::vector<uint32_t> casters;
    double n0 = std::numeric_limits<double>::max(), n1 = -n0, e0 = n0, e1 = -n0;
    for (size_t i = 0; i < tris_.size(); ++i) {
        const auto &t = tris_[i];
        // Geometry at ground level casts no shadow and holds no visible surface.
        if (-std::min({t.a[2], t.b[2], t.c[2]}) < 0.05) {
            continue;
        }
        casters.push_back(static_cast<uint32_t>(i));
        n0 = std::min({n0, t.a[0], t.b[0], t.c[0]});
        n1 = std::max({n1, t.a[0], t.b[0], t.c[0]});
        e0 = std::min({e0, t.a[1], t.b[1], t.c[1]});
        e1 = std::max({e1, t.a[1], t.b[1], t.c[1]});
    }
    if (casters.empty()) {
        return;
    }

    Grid g;
    g.origin_n = n0 - 1.0;
    g.origin_e = e0 - 1.0;
    const double ext_n = n1 - n0 + 2.0;
    const double ext_e = e1 - e0 + 2.0;
    // 4 m cells, coarsened so a city of any size stays under about a million cells.
    g.cell = std::max(4.0, std::sqrt(ext_n * ext_e / 1.0e6));
    g.rows = static_cast<int64_t>(ext_n / g.cell) + 1;
    g.cols = static_cast<int64_t>(ext_e / g.cell) + 1;
    const auto cells = static_cast<size_t>(g.rows * g.cols);
    g.start.assign(cells + 1, 0);
    g.top.assign(cells, 0.0F);

    const auto range = [&g](double lo, double hi, double origin, int64_t limit, int64_t &a, int64_t &b) {
        a = std::clamp(static_cast<int64_t>(std::floor((lo - origin) / g.cell)), int64_t{0}, limit - 1);
        b = std::clamp(static_cast<int64_t>(std::floor((hi - origin) / g.cell)), int64_t{0}, limit - 1);
    };
    for (int pass = 0; pass < 2; ++pass) {
        std::vector<uint32_t> cursor;
        if (pass == 1) {
            for (size_t c = 0; c < cells; ++c) {
                g.start[c + 1] += g.start[c];
            }
            g.items.resize(g.start[cells]);
            cursor.assign(g.start.begin(), g.start.end() - 1);
        }
        for (const uint32_t i : casters) {
            const auto &t = tris_[i];
            int64_t r0 = 0, r1 = 0, c0 = 0, c1 = 0;
            range(std::min({t.a[0], t.b[0], t.c[0]}), std::max({t.a[0], t.b[0], t.c[0]}), g.origin_n, g.rows,
                  r0, r1);
            range(std::min({t.a[1], t.b[1], t.c[1]}), std::max({t.a[1], t.b[1], t.c[1]}), g.origin_e, g.cols,
                  c0, c1);
            const auto top = static_cast<float>(-std::min({t.a[2], t.b[2], t.c[2]}));
            for (int64_t r = r0; r <= r1; ++r) {
                for (int64_t c = c0; c <= c1; ++c) {
                    const auto cell = static_cast<size_t>(r * g.cols + c);
                    if (pass == 0) {
                        ++g.start[cell + 1];
                        g.top[cell] = std::max(g.top[cell], top);
                    } else {
                        g.items[cursor[cell]++] = i;
                    }
                }
            }
            g.max_height = std::max(g.max_height, static_cast<double>(top));
        }
    }
    grid_ = std::move(g);
}

bool RasterScene::ray_hits(size_t tri, const Vec3 &o, const Vec3 &d, double t_min) const {
    // Moller-Trumbore.
    const auto &t = tris_[tri];
    const Vec3 e1 = sub(t.b, t.a);
    const Vec3 e2 = sub(t.c, t.a);
    const Vec3 pv = cross(d, e2);
    const double det = dot(e1, pv);
    if (std::fabs(det) < 1e-12) {
        return false; // parallel, which includes every triangle coplanar with the start
    }
    const double inv = 1.0 / det;
    const Vec3 tv = sub(o, t.a);
    const double u = dot(tv, pv) * inv;
    if (u < 0.0 || u > 1.0) {
        return false;
    }
    const Vec3 qv = cross(tv, e1);
    const double v = dot(d, qv) * inv;
    if (v < 0.0 || u + v > 1.0) {
        return false;
    }
    return dot(e2, qv) * inv > t_min;
}

bool RasterScene::in_shadow(const Vec3 &p, int tri) const {
    const Grid &g = grid_;
    const double h0 = -p[2];
    if (g.items.empty() || h0 >= g.max_height) {
        return false;
    }
    const Vec3 s = sun_direction();
    const double rise = -s[2]; // metres up per metre along the ray
    // Past this the ray is above everything that could block it.
    const double t_end = (g.max_height - h0) / rise;

    // Walk the grid cells under the ray (Amanatides & Woo), from the start until t_end.
    const double gx = (p[0] - g.origin_n) / g.cell;
    const double gy = (p[1] - g.origin_e) / g.cell;
    const double dx = s[0] / g.cell;
    const double dy = s[1] / g.cell;
    double t0 = 0.0;
    double t1 = t_end;
    const auto clip = [&t0, &t1](double pos, double dir, double limit) {
        if (std::fabs(dir) < 1e-15) {
            return pos >= 0.0 && pos < limit;
        }
        double ta = (0.0 - pos) / dir;
        double tb = (limit - pos) / dir;
        if (ta > tb) {
            std::swap(ta, tb);
        }
        t0 = std::max(t0, ta);
        t1 = std::min(t1, tb);
        return t0 <= t1;
    };
    if (!clip(gx, dx, static_cast<double>(g.rows)) || !clip(gy, dy, static_cast<double>(g.cols))) {
        return false;
    }
    int64_t ix = std::clamp(static_cast<int64_t>(std::floor(gx + dx * t0)), int64_t{0}, g.rows - 1);
    int64_t iy = std::clamp(static_cast<int64_t>(std::floor(gy + dy * t0)), int64_t{0}, g.cols - 1);
    const int64_t step_x = dx > 0.0 ? 1 : -1;
    const int64_t step_y = dy > 0.0 ? 1 : -1;
    constexpr double kInf = std::numeric_limits<double>::infinity();
    double next_x = dx != 0.0 ? (static_cast<double>(ix + (dx > 0.0 ? 1 : 0)) - gx) / dx : kInf;
    double next_y = dy != 0.0 ? (static_cast<double>(iy + (dy > 0.0 ? 1 : 0)) - gy) / dy : kInf;
    const double delta_x = dx != 0.0 ? std::fabs(1.0 / dx) : kInf;
    const double delta_y = dy != 0.0 ? std::fabs(1.0 / dy) : kInf;

    double t_cell = t0;
    while (t_cell <= t1) {
        const auto cell = static_cast<size_t>(ix * g.cols + iy);
        // The ray only rises, so its lowest point in a cell is where it enters; a cell whose
        // tallest geometry is below that cannot block it.
        if (g.start[cell] != g.start[cell + 1] && h0 + rise * t_cell <= g.top[cell]) {
            for (uint32_t k = g.start[cell]; k < g.start[cell + 1]; ++k) {
                const uint32_t other = g.items[k];
                if (static_cast<int>(other) != tri && ray_hits(other, p, s, 1e-3)) {
                    return true;
                }
            }
        }
        if (next_x < next_y) {
            t_cell = next_x;
            next_x += delta_x;
            ix += step_x;
            if (ix < 0 || ix >= g.rows) {
                break;
            }
        } else {
            t_cell = next_y;
            next_y += delta_y;
            iy += step_y;
            if (iy < 0 || iy >= g.cols) {
                break;
            }
        }
    }
    return false;
}

void RasterScene::albedo(int tri, const Vec3 &p, double footprint_m, double out[3]) const {
    if (tri < 0) {
        texture_.ground(p[0], p[1], footprint_m, out);
        return;
    }
    const auto i = static_cast<size_t>(tri);
    BuildingPoint b;
    b.building = tri_building_[i];
    b.frame = &frames_[tri_frame_[i]];
    if (b.frame->kind == SurfaceFrame::Kind::kRoof) {
        b.edge_distance_m = edge_distance(i, p);
    }
    texture_.building(p, b, footprint_m, out);
}

void RasterScene::lit_colour(int tri, const Vec3 &p, double footprint_m, double out[3]) const {
    albedo(tri, p, footprint_m, out);
    const Vec3 n = tri >= 0 ? normals_[static_cast<size_t>(tri)] : Vec3{0.0, 0.0, -1.0};
    // Two-sided: a face turned away from the sun is not lit by it, but that is the shadow
    // ray's job — it starts on the face and runs into the building's own far side. Asking
    // the winding instead would darken any tile whose winding is off.
    const double lambert = std::fabs(dot(n, sun_direction()));
    const bool sunlit = lambert > 0.0 && !in_shadow(p, tri);
    for (int c = 0; c < 3; ++c) {
        out[c] *= kSkyLight[c] + (sunlit ? kSunLight[c] * lambert : 0.0);
    }
}

double RasterScene::edge_distance(size_t tri, const Vec3 &p) const {
    const auto &t = tris_[tri];
    const Vec3 &n = normals_[tri];
    const double area2 = dot(n, cross(sub(t.b, t.a), sub(t.c, t.a)));
    if (!(std::fabs(area2) > 1e-12)) {
        return 1e9;
    }
    const double l0 = dot(n, cross(sub(t.b, p), sub(t.c, p))) / area2;
    const double l1 = dot(n, cross(sub(t.c, p), sub(t.a, p))) / area2;
    const double l[3] = {l0, l1, 1.0 - l0 - l1};
    double best = 1e9;
    for (int k = 0; k < 3; ++k) {
        if ((tri_outline_[tri] & (1U << k)) != 0U) {
            best = std::min(best, std::max(l[k], 0.0) *
                                      static_cast<double>(tri_altitude_[tri][static_cast<size_t>(k)]));
        }
    }
    return best;
}

int RasterScene::find_triangle(const Vec3 &p, double tolerance_m) const {
    const Grid &g = grid_;
    if (g.items.empty()) {
        return -1;
    }
    const auto ix = static_cast<int64_t>(std::floor((p[0] - g.origin_n) / g.cell));
    const auto iy = static_cast<int64_t>(std::floor((p[1] - g.origin_e) / g.cell));
    if (ix < 0 || ix >= g.rows || iy < 0 || iy >= g.cols) {
        return -1;
    }
    const auto cell = static_cast<size_t>(ix * g.cols + iy);
    int best = -1;
    double best_d = tolerance_m;
    for (uint32_t k = g.start[cell]; k < g.start[cell + 1]; ++k) {
        const uint32_t i = g.items[k];
        const auto &t = tris_[i];
        const Vec3 &n = normals_[i];
        const double d = std::fabs(dot(n, sub(p, t.a)));
        if (d > best_d) {
            continue;
        }
        const double area2 = dot(n, cross(sub(t.b, t.a), sub(t.c, t.a)));
        if (!(area2 > 1e-12)) {
            continue;
        }
        const double l0 = dot(n, cross(sub(t.b, p), sub(t.c, p))) / area2;
        const double l1 = dot(n, cross(sub(t.c, p), sub(t.a, p))) / area2;
        const double l[3] = {l0, l1, 1.0 - l0 - l1};
        bool inside = true;
        for (int j = 0; j < 3; ++j) {
            inside = inside &&
                     l[j] * static_cast<double>(tri_altitude_[i][static_cast<size_t>(j)]) >= -tolerance_m;
        }
        if (inside) {
            best = static_cast<int>(i);
            best_d = d;
        }
    }
    return best;
}

std::shared_ptr<const RasterScene> load_scene(const std::string &tiles_dir, uint32_t texture_seed,
                                              size_t *tiles_loaded) {
    core::WorldConfig wcfg;
    // Never stepped: this world exists only to parse the tiles into triangles.
    wcfg.dt_s = 1.0 / 800.0;
    wcfg.worker_threads = 0;
    auto world = std::make_unique<core::World>(wcfg);
    const size_t tiles = tiles_dir.empty() ? 0 : world->load_tiles(tiles_dir);
    if (tiles_loaded != nullptr) {
        *tiles_loaded = tiles;
    }
    auto scene = std::make_shared<RasterScene>();
    scene->build(world->collect_static_triangles(), texture_seed);
    return scene;
}

} // namespace skysim::render
