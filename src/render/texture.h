#pragma once

#include "core/frames.h"

#include <cstdint>

namespace skysim::render {

// What the world is made of, before any light falls on it.
//
// Photogrammetry finds the same point in two photos by the texture around it, so a world of
// flat-coloured boxes on a flat-coloured plane cannot be reconstructed at all: every patch of
// roof looks like every other. Everything here is a pure function of a WORLD position (plus
// a per-building key and the face a point lies on), so a point has the same colour from every
// camera, every vehicle and every run with the same seed — which is the property structure
// from motion needs, and the one tests/test_still.cpp holds this to.
//
// Detail runs from tens of metres (land cover, stains) down to about a centimetre (gravel,
// asphalt grain, brick). Every pattern is band-limited by the pixel footprint it is sampled
// at: octaves finer than about two pixels fade to their average and stripes narrower than a
// pixel blend to their coverage, which is what a real lens and sensor do. Point-sampling a
// pattern finer than the pixel instead would alias into noise that differs between two
// photos of the same spot — the opposite of what a matcher wants.

// How a point on a building sits on its face, measured once per face at scene build.
struct SurfaceFrame {
    enum class Kind : uint8_t { kWall, kRoof, kUnderside };
    Kind kind{Kind::kWall};
    // Walls: horizontal unit tangent along the face (NED north, east), and the face's extent
    // along it in world metres (u = dot(p, tangent)).
    double tangent_n{1.0};
    double tangent_e{0.0};
    double u_min{0.0};
    double u_max{0.0};
    // NED z extent. z_top is the higher point (more negative), z_bottom the lower.
    double z_top{0.0};
    double z_bottom{0.0};
    // Roofs: footprint bounding box, NED metres.
    double n_min{0.0};
    double n_max{0.0};
    double e_min{0.0};
    double e_max{0.0};
};

// Per-pixel facts the building texture needs beyond the world position.
struct BuildingPoint {
    uint32_t building{0}; // stable per-building key (RasterScene)
    const SurfaceFrame *frame{nullptr};
    double edge_distance_m{1e9}; // roofs: distance to the roof outline
};

class WorldTexture {
  public:
    explicit WorldTexture(uint32_t seed = 1);

    uint32_t seed() const { return seed_; }

    // Ground albedo at a point on the ground plane (NED north/east, metres), filtered for a
    // pixel `footprint_m` across. Roads every 100 m on a grid offset 50 m from the origin —
    // the street grid tools/cooker/pretile.py lays its demo city on — with kerbs, sidewalks,
    // lane markings, crossings and manholes; land cover (grass, dry grass, gravel, soil,
    // paving, shrubs) between them.
    void ground(double north, double east, double footprint_m, double out[3]) const;

    // Building albedo: per-building wall and roof material and colour, windows, plinth,
    // cornice, parapet and rooftop plant, and fine grain.
    void building(const core::Vec3 &p, const BuildingPoint &b, double footprint_m, double out[3]) const;

    // Road layout, exposed so tests can find a road and a field without guessing.
    static constexpr double kRoadPitchM = 100.0;
    static constexpr double kRoadOffsetM = 50.0;
    static constexpr double kRoadHalfWidthM = 5.5;
    static constexpr double kKerbM = 0.25;
    static constexpr double kSidewalkM = 2.5;

  private:
    void cover(double n, double e, double fp, double out[3]) const;
    void asphalt(double n, double e, double dn_signed, double de_signed, double fp, double out[3]) const;
    void sidewalk(double n, double e, double fp, double out[3]) const;
    uint32_t salt(uint32_t layer) const { return salts_[layer]; }
    void wall(const core::Vec3 &p, const BuildingPoint &b, uint32_t h, double fp, double out[3]) const;
    void roof(const core::Vec3 &p, const BuildingPoint &b, uint32_t h, double fp, double out[3]) const;

    uint32_t seed_;
    // One noise salt per texture layer, derived from the seed once rather than per pixel.
    uint32_t salts_[32];
};

// --- primitives, exposed for the tests -----------------------------------------------------

// A well-mixed 32-bit hash (deterministic across platforms and builds).
uint32_t hash32(uint32_t x);
uint32_t hash_cell(int64_t x, int64_t y, uint32_t salt);

// Value noise in [0, 1] on a unit lattice, smoothstep-interpolated.
double value_noise(double x, double y, uint32_t salt);

// Band-limited fractal noise in [0, 1], mean 0.5: `octaves` layers starting at
// `wavelength_m` and halving, each faded out as it approaches the pixel footprint.
double fbm(double x, double y, double wavelength_m, int octaves, double gain, double footprint_m,
           uint32_t salt);

// Fraction of a pixel `footprint_m` wide, centred `distance_m` from a band's centre line,
// that the band (half-width `half_width_m`) covers. A box-filtered stripe.
double band_coverage(double distance_m, double half_width_m, double footprint_m);

// The same for a stripe repeating every `period_m`, `on_m` long, starting at 0.
double pulse_coverage(double x_m, double period_m, double on_m, double footprint_m);

} // namespace skysim::render
