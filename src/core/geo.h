#pragma once
// A world NED position as latitude / longitude / altitude, the way ArduPilot does it.
//
// The world's NED origin is the SITL home. ArduPilot turns the NED position skysim sends into
// the GPS fix it reports — and so into the lat/lon in CAMERA_FEEDBACK, which is what a survey
// photo gets geotagged with — via Location::offset(): a flat-earth step on a sphere of
// RADIUS_OF_EARTH = 6 378 100 m (LOCATION_SCALING_FACTOR_INV = 89.83204953368922 1e-7 deg per
// metre), with the east step scaled by cos() of the latitude halfway along it
// (AP_Common/Location.cpp, Location::offset and longitude_scale). Doing the same here keeps a
// skysim frame's pose headers on the same coordinates as the autopilot's own geotag, rather
// than a few decimetres off over a survey block because of a different earth radius.
//
// Header-only and dependency-free: the control plane, the renderer tools and the tests all
// need it, and none of them should pull in another library for twelve lines of arithmetic.
#include <algorithm>
#include <cmath>

namespace skysim::core {

struct GeoOrigin {
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double alt_amsl_m = 0.0; // height of the NED origin (ground at home) above mean sea level
};

struct GeoPoint {
    double lat_deg = 0.0;
    double lon_deg = 0.0;
    double alt_amsl_m = 0.0;
    double alt_rel_m = 0.0; // above the origin, i.e. -down
};

// 1e-7 degrees per metre on ArduPilot's sphere.
inline constexpr double kArduLocationScalingFactorInv = 89.83204953368922;

inline GeoPoint geo_from_ned(const GeoOrigin &origin, double north_m, double east_m, double down_m) {
    constexpr double kDegToRad = M_PI / 180.0;
    const double dlat_deg = north_m * kArduLocationScalingFactorInv * 1e-7;
    // ArduPilot's longitude_scale(): cos of the mid latitude, never below 0.01 so a pole
    // does not divide by zero.
    const double scale = std::max(std::cos((origin.lat_deg + dlat_deg * 0.5) * kDegToRad), 0.01);
    GeoPoint p;
    p.lat_deg = std::clamp(origin.lat_deg + dlat_deg, -90.0, 90.0);
    // wrap_longitude(): stay in (-180, 180]. A remainder, not a loop, so no input can spin it.
    const double lon =
        std::remainder(origin.lon_deg + east_m * kArduLocationScalingFactorInv * 1e-7 / scale, 360.0);
    p.lon_deg = lon == -180.0 ? 180.0 : lon;
    p.alt_rel_m = -down_m;
    p.alt_amsl_m = origin.alt_amsl_m - down_m;
    return p;
}

// North and east of the origin, metres, for a latitude / longitude: the inverse of
// geo_from_ned, and what ArduPilot's Location::get_distance_NE computes (the east step scaled by
// the cosine of the mid latitude, the longitude difference wrapped to (-180, 180]).
struct NorthEast {
    double north_m = 0.0;
    double east_m = 0.0;
};

inline NorthEast ned_from_geo(const GeoOrigin &origin, double lat_deg, double lon_deg) {
    constexpr double kDegToRad = M_PI / 180.0;
    const double scale = std::max(std::cos((origin.lat_deg + lat_deg) * 0.5 * kDegToRad), 0.01);
    NorthEast ne;
    ne.north_m = (lat_deg - origin.lat_deg) / (kArduLocationScalingFactorInv * 1e-7);
    ne.east_m =
        std::remainder(lon_deg - origin.lon_deg, 360.0) * scale / (kArduLocationScalingFactorInv * 1e-7);
    return ne;
}

} // namespace skysim::core
