#pragma once

#include "core/frames.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace skysim::render {

// What the light and the air look like, in one place.
//
// Two renderers draw this world — the rasteriser that ships, and the ray caster the tests use
// as a reference — and a picture that changes depending on which one drew it is worse than
// either. What surfaces are made of lives in texture.h; how they are lit and hazed lives here.

//: Late afternoon, high and off to the south-west, so vertical faces of a building are lit
//: differently from each other and every building throws a shadow to the north-east.
//: Points from the surface towards the sun (not unit length; see sun_direction()).
inline constexpr core::Vec3 kSunDirNed{-0.45, -0.35, -0.82};

inline core::Vec3 sun_direction() {
    const double n = std::sqrt(kSunDirNed[0] * kSunDirNed[0] + kSunDirNed[1] * kSunDirNed[1] +
                               kSunDirNed[2] * kSunDirNed[2]);
    return {kSunDirNed[0] / n, kSunDirNed[1] / n, kSunDirNed[2] / n};
}

inline constexpr double kSkyHorizon[3] = {0.72, 0.80, 0.90};
inline constexpr double kSkyZenith[3] = {0.25, 0.47, 0.80};

//: Light every surface gets from the sky, shadowed or not. Bluish, as open shade is.
inline constexpr double kSkyLight[3] = {0.36, 0.38, 0.44};
//: Direct sun, scaled by how squarely a surface faces it. Warm, and together with the sky
//: light it makes a sunlit field read as neutral rather than tinted.
inline constexpr double kSunLight[3] = {0.78, 0.75, 0.70};

//: Haze never fully erases a surface, so the far edge of the draw distance reads as far away
//: rather than as blank sky.
inline constexpr double kMaxHaze = 0.82;

// NaN maps to 0: std::clamp would pass it through to a cast that is undefined for it.
inline uint8_t to_byte(double v) {
    const double c = v > 0.0 ? (v < 1.0 ? v : 1.0) : 0.0;
    return static_cast<uint8_t>(c * 255.0 + 0.5);
}

inline double mix(double a, double b, double t) { return a + (b - a) * t; }

//: Sky colour for a ray direction. NED z is down, so a ray at the sky has z < 0. Biased
//: towards the horizon colour so the band near eye level stays pale, which is what makes a
//: horizon read as one.
inline void sky_colour(const core::Vec3 &dir, double out[3]) {
    const double up = std::clamp(-dir[2], 0.0, 1.0);
    const double t = std::pow(up, 0.65);
    for (int c = 0; c < 3; ++c) {
        out[c] = mix(kSkyHorizon[c], kSkyZenith[c], t);
    }
}

//: How much haze sits between the camera and something `distance_m` away, for a camera whose
//: draw distance is `max_range_m` and whose haze starts `start` of the way out (0..1).
//: Quadratic from the start to the full distance, capped at kMaxHaze.
inline double haze_at(double distance_m, double max_range_m, double start = 0.0) {
    // A non-positive range would divide by zero, and the NaN survives clamp and min all the
    // way to a cast that is undefined behaviour. Nothing is visible at zero range anyway, so
    // everything is as hazy as it gets.
    if (!(max_range_m > 0.0)) {
        return kMaxHaze;
    }
    const double s = std::clamp(start, 0.0, 0.99);
    const double x = std::clamp((distance_m / max_range_m - s) / (1.0 - s), 0.0, 1.0);
    return std::min(x * x, kMaxHaze);
}

} // namespace skysim::render
