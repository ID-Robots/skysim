#pragma once

#include "core/frames.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace skysim::render {

// What the world looks like, in one place.
//
// Two renderers draw this world — the rasteriser that ships, and the ray caster
// the tests use as a reference — and a picture that changes depending on which
// one drew it is worse than either. These constants were duplicated in both
// files, in different types, kept equal by a comment saying they were equal.
// They had already drifted.

//: Late afternoon, high and off to one side, so vertical faces of a building are
//: lit differently from each other and the block reads as solid rather than flat.
//: Points from the surface towards the sun.
inline constexpr core::Vec3 kSunDirNed{-0.45, -0.35, -0.82};

inline constexpr double kSkyHorizon[3] = {0.72, 0.80, 0.90};
inline constexpr double kSkyZenith[3] = {0.25, 0.47, 0.80};
inline constexpr double kGround[3] = {0.35, 0.42, 0.27};
inline constexpr double kBuilding[3] = {0.62, 0.60, 0.57};

//: Light a surface gets regardless of which way it faces.
inline constexpr double kAmbient = 0.42;

//: Haze never fully erases a surface, so the far edge of the draw distance reads
//: as far away rather than as blank sky.
inline constexpr double kMaxHaze = 0.82;

inline uint8_t to_byte(double v) { return static_cast<uint8_t>(std::clamp(v, 0.0, 1.0) * 255.0 + 0.5); }

inline double mix(double a, double b, double t) { return a + (b - a) * t; }

//: How lit a surface is. Two-sided: a tile's winding is not guaranteed to face
//: the camera, and a building lit from the inside out looks like a hole.
inline double shade_for_normal(const core::Vec3 &n) {
    const double lambert =
        std::abs(n[0] * kSunDirNed[0] + n[1] * kSunDirNed[1] + n[2] * kSunDirNed[2]);
    return kAmbient + (1.0 - kAmbient) * std::clamp(lambert, 0.0, 1.0);
}

//: Sky colour for a ray direction. NED z is down, so a ray at the sky has z < 0.
//: Biased towards the horizon colour so the band near eye level stays pale, which
//: is what makes a horizon read as one.
inline void sky_colour(const core::Vec3 &dir, double out[3]) {
    const double up = std::clamp(-dir[2], 0.0, 1.0);
    const double t = std::pow(up, 0.65);
    for (int c = 0; c < 3; ++c) {
        out[c] = mix(kSkyHorizon[c], kSkyZenith[c], t);
    }
}

//: How much haze sits between the camera and something this far away.
inline double haze_at(double distance_m, double max_range_m) {
    const double fog = std::clamp(distance_m / max_range_m, 0.0, 1.0);
    return std::min(fog * fog, kMaxHaze);
}

} // namespace skysim::render
