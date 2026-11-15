#include "render/texture.h"

#include <algorithm>
#include <cmath>

namespace skysim::render {
namespace {

// Layers of the texture, each with its own noise so they do not line up with each other.
enum Layer : uint32_t {
    kCoverWarpN = 1,
    kCoverWarpE,
    kCover,
    kGrass,
    kGrassHue,
    kDry,
    kGravel,
    kSoil,
    kPaving,
    kShrub,
    kShrubLeaf,
    kAsphalt,
    kAsphaltPatch,
    kWear,
    kWalk,
    kWall,
    kBrick,
    kStain,
    kRoof,
    kUnit,
    kGrain,
    kWindow,
    kBuilding,
    kLayerCount, // must stay <= 32 (WorldTexture::salts_)
};
static_assert(kLayerCount <= 32);

double unit(uint32_t h) { return static_cast<double>(h >> 8) * (1.0 / 16777216.0); }

double lerp(double a, double b, double t) { return a + (b - a) * t; }

void set(double out[3], double r, double g, double b) {
    out[0] = r;
    out[1] = g;
    out[2] = b;
}

void scale(double out[3], double k) {
    out[0] *= k;
    out[1] *= k;
    out[2] *= k;
}

void blend(double out[3], const double c[3], double t) {
    if (t <= 0.0) {
        return;
    }
    for (int i = 0; i < 3; ++i) {
        out[i] = lerp(out[i], c[i], std::min(t, 1.0));
    }
}

// Fraction of a pixel on the positive side of an edge `x_m` away from its centre.
double step_coverage(double x_m, double fp) { return std::clamp(x_m / fp + 0.5, 0.0, 1.0); }

// A rectangle centred at the origin, box-filtered by the pixel.
double rect_coverage(double x, double y, double half_x, double half_y, double fp) {
    return band_coverage(x, half_x, fp) * band_coverage(y, half_y, fp);
}

// How much of a per-cell variation survives at this footprint: a pixel that spans several
// cells sees their average, not one of them.
double cell_detail(double cell_m, double fp) { return std::clamp(1.5 - fp / cell_m, 0.0, 1.0); }

// Signed distance to the nearest road centre line on one axis.
double road_offset(double c) {
    const double k = std::round((c - WorldTexture::kRoadOffsetM) / WorldTexture::kRoadPitchM);
    return c - WorldTexture::kRoadOffsetM - k * WorldTexture::kRoadPitchM;
}

} // namespace

uint32_t hash32(uint32_t x) {
    // lowbias32 (C. Wellons): full avalanche, three multiplies.
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}

uint32_t hash_cell(int64_t x, int64_t y, uint32_t salt) {
    return hash32(hash32((salt * 0x9E3779B9U) ^ static_cast<uint32_t>(x)) ^ static_cast<uint32_t>(y));
}

double value_noise(double x, double y, uint32_t salt) {
    const double fx = std::floor(x);
    const double fy = std::floor(y);
    const auto ix = static_cast<int64_t>(fx);
    const auto iy = static_cast<int64_t>(fy);
    const double tx = x - fx;
    const double ty = y - fy;
    const double sx = tx * tx * (3.0 - 2.0 * tx);
    const double sy = ty * ty * (3.0 - 2.0 * ty);
    // hash_cell, with the column hash shared by the two corners in each column.
    const uint32_t base = salt * 0x9E3779B9U;
    const uint32_t h0 = hash32(base ^ static_cast<uint32_t>(ix));
    const uint32_t h1 = hash32(base ^ static_cast<uint32_t>(ix + 1));
    const auto y0 = static_cast<uint32_t>(iy);
    const auto y1 = static_cast<uint32_t>(iy + 1);
    const double a = unit(hash32(h0 ^ y0));
    const double b = unit(hash32(h1 ^ y0));
    const double c = unit(hash32(h0 ^ y1));
    const double d = unit(hash32(h1 ^ y1));
    return lerp(lerp(a, b, sx), lerp(c, d, sx), sy);
}

double fbm(double x, double y, double wavelength_m, int octaves, double gain, double footprint_m,
           uint32_t salt) {
    const double fp = std::max(footprint_m, 1e-6);
    double sum = 0.0;
    double norm = 0.0;
    double amp = 1.0;
    double wl = wavelength_m;
    for (int i = 0; i < octaves; ++i) {
        // Fully present at four pixels per wavelength, gone at two (Nyquist): finer than
        // that a sensor sees the average, so that is what this octave contributes.
        const double w = std::min((wl / fp - 2.0) * 0.5, 1.0);
        if (w <= 0.0) {
            // This octave and every finer one sits below the pixel.
            const double rest = amp * (1.0 - std::pow(gain, octaves - i)) / (1.0 - gain);
            sum += 0.5 * rest;
            norm += rest;
            break;
        }
        // Each octave shifted off the others' lattice so their grid lines never coincide.
        const double inv = 1.0 / wl;
        const double v =
            value_noise(x * inv + 0.3183 * i, y * inv - 0.6180 * i, salt + static_cast<uint32_t>(i));
        sum += amp * (0.5 + w * (v - 0.5));
        norm += amp;
        amp *= gain;
        wl *= 0.5;
    }
    return norm > 0.0 ? sum / norm : 0.5;
}

double band_coverage(double distance_m, double half_width_m, double footprint_m) {
    const double d = std::fabs(distance_m);
    const double fp = std::max(footprint_m, 1e-6);
    const double lo = std::max(d - fp * 0.5, -half_width_m);
    const double hi = std::min(d + fp * 0.5, half_width_m);
    return hi > lo ? (hi - lo) / fp : 0.0;
}

double pulse_coverage(double x_m, double period_m, double on_m, double footprint_m) {
    const double centre = on_m * 0.5;
    double d = x_m - period_m * std::floor(x_m / period_m) - centre;
    d -= period_m * std::round(d / period_m);
    const double single = band_coverage(d, on_m * 0.5, footprint_m);
    // Once a pixel spans neighbouring stripes as well, it tends to the stripes' duty cycle.
    const double t = std::clamp((footprint_m / period_m - 0.5) / 1.5, 0.0, 1.0);
    return lerp(single, on_m / period_m, t);
}

WorldTexture::WorldTexture(uint32_t seed) : seed_(seed) {
    for (uint32_t layer = 0; layer < 32; ++layer) {
        salts_[layer] = hash32(seed_ * 0x632BE5ABU + layer * 0x85EBCA6BU);
    }
}

// --- ground ---------------------------------------------------------------------------------

void WorldTexture::ground(double n, double e, double footprint_m, double out[3]) const {
    const double fp = std::max(footprint_m, 1e-4);
    const double sn = road_offset(n); // across an east-west road
    const double se = road_offset(e); // across a north-south road
    const double dn = std::fabs(sn);
    const double de = std::fabs(se);
    const double kerb_edge = kRoadHalfWidthM + kKerbM;
    const double walk_edge = kerb_edge + kSidewalkM;

    // Nested zones, outermost first, each blended in by how much of the pixel it covers, so
    // a kerb is a clean edge at any distance rather than a staircase.
    const double c_walk = std::max(step_coverage(walk_edge - dn, fp), step_coverage(walk_edge - de, fp));
    const double c_kerb = std::max(step_coverage(kerb_edge - dn, fp), step_coverage(kerb_edge - de, fp));
    const double c_road =
        std::max(step_coverage(kRoadHalfWidthM - dn, fp), step_coverage(kRoadHalfWidthM - de, fp));

    set(out, 0.0, 0.0, 0.0);
    if (c_walk < 1.0) {
        cover(n, e, fp, out);
    }
    double layer[3];
    if (c_walk > 0.0 && c_kerb < 1.0) {
        sidewalk(n, e, fp, layer);
        blend(out, layer, c_walk);
    }
    if (c_kerb > 0.0 && c_road < 1.0) {
        set(layer, 0.67, 0.66, 0.63);
        scale(layer, 0.85 + 0.3 * fbm(n, e, 0.6, 5, 0.6, fp, salt(kWalk) + 7));
        blend(out, layer, c_kerb);
    }
    if (c_road > 0.0) {
        asphalt(n, e, sn, se, fp, layer);
        blend(out, layer, c_road);
    }
}

void WorldTexture::cover(double n, double e, double fp, double out[3]) const {
    // Land cover: patches of grass, dry grass, gravel, soil and paving, each a cell of a
    // jittered 40 m Voronoi diagram, its borders roughened by a warp so fields are not
    // polygons. A border is a clean edge, anti-aliased by the pixel footprint.
    const double wn = n + 36.0 * (fbm(n, e, 40.0, 2, 0.5, fp, salt(kCoverWarpN)) - 0.5);
    const double we = e + 36.0 * (fbm(n, e, 40.0, 2, 0.5, fp, salt(kCoverWarpE)) - 0.5);
    constexpr double kPatch = 40.0;
    const auto ci = static_cast<int64_t>(std::floor(wn / kPatch));
    const auto cj = static_cast<int64_t>(std::floor(we / kPatch));
    double d1 = 1e300;
    double d2 = 1e300;
    double p1[2] = {0.0, 0.0};
    double p2[2] = {0.0, 0.0};
    uint32_t h1 = 0;
    uint32_t h2 = 0;
    for (int64_t i = ci - 1; i <= ci + 1; ++i) {
        for (int64_t j = cj - 1; j <= cj + 1; ++j) {
            // One hash per cell: its low and high halves place the point.
            const uint32_t h = hash_cell(i, j, salt(kCover));
            const double pn =
                (static_cast<double>(i) + 0.15 + 0.7 * static_cast<double>(h & 0xFFFFU) / 65536.0) * kPatch;
            const double pe =
                (static_cast<double>(j) + 0.15 + 0.7 * static_cast<double>(h >> 16) / 65536.0) * kPatch;
            const double d = (wn - pn) * (wn - pn) + (we - pe) * (we - pe);
            if (d < d1) {
                d2 = d1;
                p2[0] = p1[0];
                p2[1] = p1[1];
                h2 = h1;
                d1 = d;
                p1[0] = pn;
                p1[1] = pe;
                h1 = h;
            } else if (d < d2) {
                d2 = d;
                p2[0] = pn;
                p2[1] = pe;
                h2 = h;
            }
        }
    }
    // Grass is the commonest cover, then paving; gravel and soil are the scattered rest.
    const auto cover_class = [](uint32_t h) {
        const double u = unit(hash32(h));
        return u < 0.38 ? 0 : (u < 0.52 ? 1 : (u < 0.64 ? 2 : (u < 0.74 ? 3 : 4)));
    };
    const int cls = cover_class(h1);
    const int other = cover_class(h2);
    // Distance to the border (the bisector of the two nearest points).
    const double sep = std::hypot(p2[0] - p1[0], p2[1] - p1[1]);
    const double border = sep > 0.0 ? (d2 - d1) / (2.0 * sep) : 1e9;
    // The share of the pixel on the far side of the border: a half at the border itself.
    const double t = other != cls ? step_coverage(-border, std::max(0.5, fp)) : 0.0;

    const auto material = [&](int k, double col[3]) {
        switch (k) {
        case 0: { // grass, with drier and lusher patches
            set(col, 0.29, 0.40, 0.17);
            const double hue = fbm(n, e, 6.0, 3, 0.5, fp, salt(kGrassHue));
            const double yellow[3] = {0.42, 0.46, 0.21};
            blend(col, yellow, std::clamp((hue - 0.42) * 3.0, 0.0, 1.0));
            scale(col, 0.55 + 0.9 * fbm(n, e, 3.0, 9, 0.62, fp, salt(kGrass)));
            break;
        }
        case 1: // dry grass
            set(col, 0.55, 0.51, 0.31);
            scale(col, 0.6 + 0.8 * fbm(n, e, 3.0, 9, 0.62, fp, salt(kDry)));
            break;
        case 2: // gravel: strong centimetre-scale speckle
            set(col, 0.57, 0.55, 0.51);
            scale(col, 0.45 + 1.1 * fbm(n, e, 0.35, 6, 0.8, fp, salt(kGravel)));
            break;
        case 3: // bare soil
            set(col, 0.46, 0.36, 0.25);
            scale(col, 0.6 + 0.8 * fbm(n, e, 2.0, 8, 0.6, fp, salt(kSoil)));
            break;
        default: { // paved plaza: 0.6 m slabs, each its own shade, with joints
            constexpr double kSlab = 0.6;
            const uint32_t h = hash_cell(static_cast<int64_t>(std::floor(n / kSlab)),
                                         static_cast<int64_t>(std::floor(e / kSlab)), salt(kPaving));
            set(col, 0.62, 0.60, 0.56);
            scale(col, 1.0 + 0.2 * (unit(h) - 0.5) * cell_detail(kSlab, fp));
            scale(col, 0.8 + 0.4 * fbm(n, e, 0.3, 5, 0.6, fp, salt(kPaving) + 11));
            const double joint = std::max(pulse_coverage(n + 0.006, kSlab, 0.012, fp),
                                          pulse_coverage(e + 0.006, kSlab, 0.012, fp));
            scale(col, 1.0 - 0.45 * joint);
            break;
        }
        }
    };

    material(cls, out);
    if (t > 0.0) {
        double col[3];
        material(other, col);
        blend(out, col, t);
    }

    // Shrubs and small trees on the green: a canopy disc per 7 m cell at most, darker and
    // leafy, brighter at the crown.
    if (cls <= 1) {
        constexpr double kCell = 7.0;
        const auto si = static_cast<int64_t>(std::floor(n / kCell));
        const auto sj = static_cast<int64_t>(std::floor(e / kCell));
        const uint32_t h = hash_cell(si, sj, salt(kShrub));
        if (unit(h) < 0.35) {
            const double r = 0.9 + 1.8 * unit(hash32(h ^ 1U));
            const double cn = static_cast<double>(si) * kCell + r + (kCell - 2.0 * r) * unit(hash32(h ^ 2U));
            const double ce = static_cast<double>(sj) * kCell + r + (kCell - 2.0 * r) * unit(hash32(h ^ 3U));
            const double dist = std::hypot(n - cn, e - ce);
            const double r_eff = r * (0.8 + 0.4 * fbm(n, e, 0.9, 2, 0.5, fp, salt(kShrubLeaf) + 3));
            const double cov = step_coverage(r_eff - dist, fp);
            if (cov > 0.0) {
                double leaf[3] = {0.13, 0.24, 0.08};
                scale(leaf, 0.5 + fbm(n, e, 0.5, 5, 0.65, fp, salt(kShrubLeaf)));
                scale(leaf, 0.75 + 0.5 * std::clamp(1.0 - dist / r, 0.0, 1.0));
                blend(out, leaf, cov);
            }
        }
    }
}

void WorldTexture::sidewalk(double n, double e, double fp, double out[3]) const {
    constexpr double kSlab = 0.5;
    const uint32_t h = hash_cell(static_cast<int64_t>(std::floor(n / kSlab)),
                                 static_cast<int64_t>(std::floor(e / kSlab)), salt(kWalk));
    set(out, 0.58, 0.56, 0.53);
    scale(out, 1.0 + 0.16 * (unit(h) - 0.5) * cell_detail(kSlab, fp));
    scale(out, 0.8 + 0.4 * fbm(n, e, 0.25, 5, 0.65, fp, salt(kWalk) + 3));
    const double joint =
        std::max(pulse_coverage(n + 0.005, kSlab, 0.01, fp), pulse_coverage(e + 0.005, kSlab, 0.01, fp));
    scale(out, 1.0 - 0.4 * joint);
}

void WorldTexture::asphalt(double n, double e, double sn, double se, double fp, double out[3]) const {
    set(out, 0.21, 0.21, 0.22);
    scale(out, 0.7 + 0.6 * fbm(n, e, 0.4, 6, 0.7, fp, salt(kAsphalt)));
    scale(out, 0.85 + 0.3 * fbm(n, e, 14.0, 3, 0.5, fp, salt(kAsphaltPatch)));

    const double R = kRoadHalfWidthM;
    const double walk_edge = R + kKerbM + kSidewalkM;
    const bool ew = std::fabs(sn) < R; // on an east-west carriageway
    const bool ns = std::fabs(se) < R;

    // Paint for one carriageway: `across` from its centre line, `along` it, `cross` the
    // distance from the centre line of the road it meets.
    const auto marks = [&](double across, double along, double cross) {
        double m = 0.0;
        // Zebra crossing where the other road's sidewalk meets this carriageway.
        const double zebra_zone = band_coverage(cross - (R + 0.5 + (kKerbM + kSidewalkM - 0.75) * 0.5),
                                                (kKerbM + kSidewalkM - 0.75) * 0.5, fp);
        if (zebra_zone > 0.0) {
            m = std::max(m, zebra_zone * pulse_coverage(across + R, 1.0, 0.5, fp) *
                                band_coverage(across, R - 0.3, fp));
        }
        // Stop line just before it.
        m = std::max(m,
                     band_coverage(cross - (walk_edge + 0.7), 0.2, fp) * band_coverage(across, R - 0.3, fp));
        // Lane lines along the rest of the block.
        const double lines_zone = step_coverage(cross - (walk_edge + 1.4), fp);
        if (lines_zone > 0.0) {
            const double centre = band_coverage(across, 0.075, fp) * pulse_coverage(along, 9.0, 3.0, fp);
            const double edge = band_coverage(std::fabs(across) - (R - 0.35), 0.06, fp);
            m = std::max(m, lines_zone * std::max(centre, edge));
        }
        return m;
    };

    double paint = 0.0;
    double hole_n = 0.0;
    double hole_e = 0.0;
    if (ew && ns) {
        // Junction box: no lines, one manhole off-centre.
        hole_n = sn - 2.5;
        hole_e = se - 2.5;
    } else if (ew) {
        paint = marks(sn, e, std::fabs(se));
        hole_n = sn - 1.75; // lane centre
        hole_e = e - 30.0 * std::round((e - 15.0) / 30.0) - 15.0;
    } else {
        paint = marks(se, n, std::fabs(sn));
        hole_e = se - 1.75;
        hole_n = n - 30.0 * std::round((n - 15.0) / 30.0) - 15.0;
    }
    if (paint > 0.0) {
        const double white[3] = {0.86, 0.86, 0.82};
        blend(out, white, paint * (0.55 + 0.45 * fbm(n, e, 3.0, 3, 0.5, fp, salt(kWear))));
    }

    // Manhole: iron disc with a darker ring.
    const double dist = std::hypot(hole_n, hole_e);
    if (dist < 0.5 + fp) {
        double iron[3] = {0.17, 0.16, 0.15};
        scale(iron, 0.8 + 0.4 * fbm(n, e, 0.08, 3, 0.6, fp, salt(kAsphalt) + 9));
        blend(out, iron, step_coverage(0.32 - dist, fp));
        const double ring[3] = {0.08, 0.08, 0.08};
        blend(out, ring, band_coverage(dist - 0.27, 0.02, fp));
    }
}

// --- buildings -------------------------------------------------------------------------------

void WorldTexture::building(const core::Vec3 &p, const BuildingPoint &b, double footprint_m,
                            double out[3]) const {
    const double fp = std::max(footprint_m, 1e-4);
    const uint32_t h = hash32(b.building ^ salt(kBuilding));
    if (b.frame == nullptr || b.frame->kind == SurfaceFrame::Kind::kUnderside) {
        set(out, 0.30, 0.30, 0.30);
        return;
    }
    double gx = 0.0;
    double gy = 0.0;
    if (b.frame->kind == SurfaceFrame::Kind::kRoof) {
        roof(p, b, h, fp, out);
        gx = p[0];
        gy = p[1];
    } else {
        wall(p, b, h, fp, out);
        gx = p[0] * b.frame->tangent_n + p[1] * b.frame->tangent_e;
        gy = -p[2];
    }
    // Fine grain on every building surface: dust, weathering, texture of the material.
    scale(out, 0.9 + 0.2 * fbm(gx, gy, 0.25, 5, 0.7, fp, salt(kGrain)));
    for (int i = 0; i < 3; ++i) {
        out[i] = std::clamp(out[i], 0.0, 1.0);
    }
}

void WorldTexture::wall(const core::Vec3 &p, const BuildingPoint &b, uint32_t h, double fp,
                        double out[3]) const {
    const SurfaceFrame &f = *b.frame;
    const double u = p[0] * f.tangent_n + p[1] * f.tangent_e;
    const double u_rel = u - f.u_min;
    const double width = f.u_max - f.u_min;
    const double height = f.z_bottom - p[2]; // above the bottom of this face
    const double face_h = f.z_bottom - f.z_top;

    static constexpr double kPalette[9][3] = {
        {0.84, 0.78, 0.64}, // cream
        {0.76, 0.68, 0.54}, // beige
        {0.72, 0.72, 0.70}, // light grey
        {0.80, 0.64, 0.40}, // ochre
        {0.82, 0.68, 0.63}, // pale pink
        {0.88, 0.87, 0.84}, // white
        {0.58, 0.32, 0.24}, // brick
        {0.62, 0.62, 0.60}, // concrete panels
        {0.38, 0.39, 0.41}, // dark cladding
    };
    const uint32_t style = h % 9U;
    const double tint = 0.9 + 0.2 * unit(hash32(h ^ 0x51U));
    for (int i = 0; i < 3; ++i) {
        out[i] =
            kPalette[style][i] * tint * (0.97 + 0.06 * unit(hash32(h ^ (0x52U + static_cast<uint32_t>(i)))));
    }

    const uint32_t wall_salt = salt(kWall) ^ h;
    switch (style) {
    case 6: { // brick: 24 x 7.5 cm, stretcher bond, each brick its own shade, light mortar
        const double row = std::floor(height / 0.075);
        const double shift = std::fmod(row, 2.0) != 0.0 ? 0.12 : 0.0;
        const double col = std::floor((u + shift) / 0.24);
        const uint32_t bh = hash_cell(static_cast<int64_t>(row), static_cast<int64_t>(col), salt(kBrick) ^ h);
        scale(out, 1.0 + 0.3 * (unit(bh) - 0.5) * cell_detail(0.075, fp));
        const double mortar =
            std::max(pulse_coverage(height, 0.075, 0.01, fp), pulse_coverage(u + shift, 0.24, 0.01, fp));
        const double grey[3] = {0.74, 0.71, 0.66};
        blend(out, grey, mortar);
        break;
    }
    case 7: { // concrete panels with joints and rain streaks
        const double joint = std::max(pulse_coverage(u_rel + 0.012, 1.5, 0.024, fp),
                                      pulse_coverage(height + 0.012, 3.0, 0.024, fp));
        scale(out, 1.0 - 0.45 * joint);
        const double streak = value_noise(u / 0.7, height / 14.0, salt(kStain) ^ h);
        scale(out, 0.85 + 0.3 * lerp(0.5, streak, cell_detail(0.35, fp)));
        break;
    }
    case 8: { // cladding: vertical seams every half metre
        scale(out, 1.0 + 0.5 * pulse_coverage(u_rel + 0.01, 0.5, 0.02, fp));
        break;
    }
    default: // render: plaster with a little relief
        scale(out, 0.82 + 0.36 * fbm(u, height, 1.5, 8, 0.6, fp, wall_salt));
        break;
    }

    // Windows, on any wall big enough for a storey of them.
    if (face_h >= 2.5 && width >= 1.6) {
        const double floor_h = 3.0 + 0.6 * unit(hash32(h ^ 0x11U));
        const double ground_h = 3.6 + 0.6 * unit(hash32(h ^ 0x12U));
        const double bay_target = 2.6 + 1.2 * unit(hash32(h ^ 0x13U));
        const double bays = std::max(1.0, std::floor(width / bay_target));
        const double bay = width / bays;
        const double bay_idx = std::clamp(std::floor(u_rel / bay), 0.0, bays - 1.0);
        const double x = u_rel - (bay_idx + 0.5) * bay;
        const double half_w = bay * (0.22 + 0.12 * unit(hash32(h ^ 0x14U)));
        const auto face_key = static_cast<int64_t>(std::llround(f.u_min * 10.0 + f.tangent_n * 7.0));

        const auto glass_colour = [&](double storey, double c[3]) {
            const uint32_t wh = hash_cell(static_cast<int64_t>(storey) * 4096 + static_cast<int64_t>(bay_idx),
                                          face_key, salt(kWindow) ^ h);
            const double refl = 0.35 * unit(wh);
            set(c, 0.07 + refl * 0.50, 0.09 + refl * 0.56, 0.12 + refl * 0.64);
            if (unit(hash32(wh)) < 0.3) { // curtains drawn
                set(c, 0.60, 0.55, 0.47);
                scale(c, 0.75 + 0.5 * unit(hash32(wh ^ 5U)));
            }
        };
        const auto window = [&](double y, double sill, double win_h, double storey) {
            const double yc = y - (sill + win_h * 0.5);
            const double frame_cov = rect_coverage(x, yc, half_w + 0.07, win_h * 0.5 + 0.07, fp);
            if (frame_cov <= 0.0) {
                const double sill_cov = rect_coverage(x, y - (sill - 0.12), half_w + 0.12, 0.05, fp);
                const double stone[3] = {0.80, 0.78, 0.73};
                blend(out, stone, sill_cov);
                return;
            }
            const double frame[3] = {0.86, 0.85, 0.82};
            blend(out, frame, frame_cov);
            double glass[3];
            glass_colour(storey, glass);
            blend(out, glass, rect_coverage(x, yc, half_w, win_h * 0.5, fp));
        };

        if (height >= ground_h && height <= face_h - 0.8) {
            const double storey = std::floor((height - ground_h) / floor_h);
            const double y = height - ground_h - storey * floor_h;
            const double sill = 0.85;
            const double win_h = 1.45 + 0.3 * unit(hash32(h ^ 0x15U));
            if (ground_h + storey * floor_h + sill + win_h <= face_h - 0.5) {
                window(y, sill, win_h, storey + 1.0);
            }
        } else if (height < ground_h - 0.4) {
            if (unit(hash32(h ^ 0x16U)) < 0.5) { // shop fronts
                const double frame[3] = {0.20, 0.20, 0.21};
                const double y0 = 0.45;
                const double y1 = ground_h - 0.7;
                const double yc = height - (y0 + y1) * 0.5;
                blend(out, frame, rect_coverage(x, yc, bay * 0.42, (y1 - y0) * 0.5 + 0.08, fp));
                double glass[3];
                glass_colour(0.0, glass);
                scale(glass, 1.4);
                blend(out, glass, rect_coverage(x, yc, bay * 0.4, (y1 - y0) * 0.5, fp));
            } else if (std::fmod(bay_idx, 3.0) == 0.0) { // a door
                const double door[3] = {0.30, 0.20, 0.12};
                blend(out, door, rect_coverage(x, height - 1.1, 0.55, 1.1, fp));
            } else {
                window(height, 1.0, 1.4, 0.0);
            }
        }
    }

    // Plinth, cornice and (on some buildings) quoins: the lines that make a facade's
    // outline findable from any angle.
    const double stone[3] = {0.42, 0.40, 0.38};
    blend(out, stone, step_coverage(0.45 - height, fp));
    double cornice[3] = {out[0] * 0.85 + 0.15, out[1] * 0.85 + 0.15, out[2] * 0.85 + 0.15};
    blend(out, cornice, step_coverage(height - (face_h - 0.4), fp));
    scale(out, 1.0 - 0.4 * band_coverage(height - (face_h - 0.45), 0.03, fp));
    if (unit(hash32(h ^ 0x17U)) < 0.5) {
        const double edge = std::min(u_rel, width - u_rel);
        const double quoin =
            step_coverage(0.3 - edge, fp) * (0.5 + 0.5 * pulse_coverage(height, 0.6, 0.3, fp));
        scale(out, 1.0 + 0.15 * quoin);
    }
}

void WorldTexture::roof(const core::Vec3 &p, const BuildingPoint &b, uint32_t h, double fp,
                        double out[3]) const {
    const SurfaceFrame &f = *b.frame;
    const double n = p[0];
    const double e = p[1];
    const uint32_t type = (h >> 8) % 5U;
    const bool along_n = ((h >> 12) & 1U) != 0U;
    const double a = along_n ? n : e; // across the seams
    const double c = along_n ? e : n;
    const uint32_t roof_salt = salt(kRoof) ^ h;

    // Per building, a colour from its roof type's palette — what tells one roof from the
    // next in an orthophoto, where every roof is seen square-on from above.
    static constexpr double kPalette[5][3][3] = {
        {{0.29, 0.29, 0.31}, {0.34, 0.30, 0.27}, {0.30, 0.33, 0.33}}, // membrane: grey, brown, slate
        {{0.53, 0.51, 0.48}, {0.58, 0.54, 0.47}, {0.58, 0.50, 0.47}}, // gravel: grey, beige, pink
        {{0.67, 0.66, 0.63}, {0.71, 0.67, 0.59}, {0.62, 0.63, 0.60}}, // concrete
        {{0.62, 0.33, 0.22}, {0.49, 0.31, 0.23}, {0.67, 0.41, 0.25}}, // clay: red, brown, orange
        {{0.36, 0.46, 0.38}, {0.50, 0.27, 0.25}, {0.44, 0.47, 0.51}}, // metal: green, maroon, blue-grey
    };
    const double *base = kPalette[type][hash32(h ^ 0x23U) % 3U];
    set(out, base[0], base[1], base[2]);
    switch (type) {
    case 0: // bitumen membrane: overlapping strips, tar stains
        scale(out, 1.0 + 0.25 * pulse_coverage(a, 1.0, 0.02, fp));
        scale(out, 0.8 + 0.4 * fbm(n, e, 9.0, 4, 0.55, fp, roof_salt));
        break;
    case 1: // gravel ballast
        scale(out, 0.5 + 1.0 * fbm(n, e, 0.3, 6, 0.8, fp, roof_salt));
        break;
    case 2: // concrete slabs
        scale(out, 1.0 - 0.3 * std::max(pulse_coverage(n + 0.01, 2.0, 0.02, fp),
                                        pulse_coverage(e + 0.01, 2.0, 0.02, fp)));
        scale(out, 0.8 + 0.4 * fbm(n, e, 7.0, 4, 0.55, fp, roof_salt));
        break;
    case 3: { // clay tiles: staggered rows, every tile its own shade
        const double row = std::floor(a / 0.32);
        const double shift = std::fmod(row, 2.0) != 0.0 ? 0.12 : 0.0;
        const uint32_t th = hash_cell(static_cast<int64_t>(row),
                                      static_cast<int64_t>(std::floor((c + shift) / 0.24)), roof_salt);
        scale(out, 1.0 + 0.24 * (unit(th) - 0.5) * cell_detail(0.24, fp));
        scale(out, 1.0 - 0.35 * pulse_coverage(a, 0.32, 0.03, fp));
        break;
    }
    default: // standing-seam metal
        scale(out,
              1.0 + 0.3 * pulse_coverage(a, 0.4, 0.05, fp) - 0.25 * pulse_coverage(a - 0.06, 0.4, 0.04, fp));
        scale(out, 0.9 + 0.2 * fbm(n, e, 5.0, 3, 0.5, fp, roof_salt));
        break;
    }
    scale(out, 0.88 + 0.24 * unit(hash32(h ^ 0x21U)));

    // Rooftop plant: at most one unit per 6 m cell, wholly inside the roof's footprint.
    const double parapet = 0.35 + 0.3 * unit(hash32(h ^ 0x22U));
    {
        constexpr double kCell = 6.0;
        const auto si = static_cast<int64_t>(std::floor(n / kCell));
        const auto sj = static_cast<int64_t>(std::floor(e / kCell));
        const uint32_t hu = hash_cell(si, sj, salt(kUnit) ^ h);
        if (unit(hu) < 0.45) {
            const double hx = 0.6 + 1.0 * unit(hash32(hu ^ 1U));
            const double hy = 0.5 + 0.9 * unit(hash32(hu ^ 2U));
            const double cn =
                static_cast<double>(si) * kCell + hx + 0.3 + (kCell - 2.0 * hx - 0.6) * unit(hash32(hu ^ 3U));
            const double ce =
                static_cast<double>(sj) * kCell + hy + 0.3 + (kCell - 2.0 * hy - 0.6) * unit(hash32(hu ^ 4U));
            const double margin = parapet + 0.4;
            if (cn - hx > f.n_min + margin && cn + hx < f.n_max - margin && ce - hy > f.e_min + margin &&
                ce + hy < f.e_max - margin) {
                // Painted shade on the side away from the sun (it sits to the south-west).
                const double shade_cov = rect_coverage(n - cn - 0.25, e - ce - 0.2, hx, hy, fp);
                scale(out, 1.0 - 0.45 * shade_cov);
                const double cov = rect_coverage(n - cn, e - ce, hx, hy, fp);
                if (cov > 0.0) {
                    double unit_col[3];
                    switch (hash32(hu ^ 5U) % 3U) {
                    case 0: { // air handler with a fan
                        set(unit_col, 0.76, 0.76, 0.74);
                        const double fan = std::hypot(n - cn, e - ce);
                        scale(unit_col, 1.0 - 0.6 * step_coverage(0.45 * std::min(hx, hy) - fan, fp));
                        break;
                    }
                    case 1: { // skylight: dark glass in a light frame
                        set(unit_col, 0.18, 0.21, 0.25);
                        const double glass = rect_coverage(n - cn, e - ce, hx - 0.06, hy - 0.06, fp);
                        const double frame[3] = {0.70, 0.70, 0.68};
                        blend(unit_col, frame, (cov - glass) / cov);
                        break;
                    }
                    default: // solar panels
                        set(unit_col, 0.13, 0.15, 0.21);
                        scale(unit_col, 1.0 + 1.5 * std::max(pulse_coverage(n - cn, 1.0, 0.02, fp),
                                                             pulse_coverage(e - ce, 0.55, 0.02, fp)));
                        break;
                    }
                    blend(out, unit_col, cov);
                }
            }
        }
    }

    // Parapet round the edge: a light coping with a shadow line inside it and a dark line
    // where it meets the wall.
    if (b.edge_distance_m < parapet + 0.4) {
        const double d = b.edge_distance_m;
        double coping[3] = {0.74, 0.73, 0.70};
        scale(coping, 0.9 + 0.2 * fbm(n, e, 0.5, 4, 0.6, fp, roof_salt + 5));
        scale(out, 1.0 - 0.3 * band_coverage(d - (parapet + 0.08), 0.08, fp));
        blend(out, coping, step_coverage(parapet - d, fp));
        scale(out, 1.0 - 0.5 * band_coverage(d - 0.02, 0.02, fp));
    }
}

} // namespace skysim::render
