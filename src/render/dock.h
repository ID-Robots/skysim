#pragma once
// Observer 1's dock marker (TASK-2562): what it is and where a vehicle's lies, shared by the
// renderer (which draws it, render/dock_marker.h) and the control plane (which publishes its
// ground truth). Header-only and free of renderer types, like still_request.h, so the HTTP
// layer can describe a dock without linking the renderer.
//
// The marker is a nested e-ArUco: cv2.aruco DICT_7X7_250 ID 228 as the outer marker and ID 11
// inside its centre cell, as skyhub_core's utils/generate_e_aruco_marker.py draws it
// (assets/markers/README.md). It lies flat on the ground (NED z = 0).
//
// Marker-plane coordinates are metres from the outer marker's centre: u towards the image's
// right edge, v towards its bottom edge, as the marker image is laid out. A dock's yaw is the
// direction the image's top edge faces, clockwise from north: at yaw 0 the image lies upright
// with north up, so its top edge faces north and its right edge east.
#include <array>
#include <cmath>
#include <optional>
#include <string>

namespace skysim::render {

struct DockMarkerSpec {
    static constexpr const char *kDictionary = "DICT_7X7_250";
    static constexpr int kOuterId = 228;
    static constexpr int kInnerId = 11;
    // A 7x7 ArUco marker is 9 cells across: the 7x7 bits and a one-cell black border.
    static constexpr int kCells = 9;
    // The real Observer 1 dock (Yanko, 15.11) and the inner marker's size from Observer 1's
    // flight log of 16.11.
    static constexpr double kDefaultOuterSizeM = 1.0;
    static constexpr double kDefaultInnerSizeM = 0.09;
    static constexpr double kMinOuterSizeM = 0.1;
    static constexpr double kMaxOuterSizeM = 10.0;

    double outer_size_m{kDefaultOuterSizeM}; // side of the outer marker's black square
    double inner_size_m{kDefaultInnerSizeM}; // side of the inner marker's black square

    // Empty when the sizes can be drawn, else what is wrong. The inner marker sits on a white
    // square one cell across, so it must be smaller than a cell.
    std::string invalid() const {
        if (!(outer_size_m >= kMinOuterSizeM && outer_size_m <= kMaxOuterSizeM)) {
            return "dock marker size must be between 0.1 and 10 m";
        }
        if (!(inner_size_m > 0.0 && inner_size_m < outer_size_m / kCells)) {
            return "dock inner marker size must be above 0 and below one cell of the outer marker (size / 9)";
        }
        return {};
    }
};

// Where a vehicle's marker lies: the outer marker's centre on the ground, and its yaw.
struct DockPose {
    double north_m{0.0};
    double east_m{0.0};
    double yaw_rad{0.0}; // the image's top edge, clockwise from north
};

// A change to a dock's pose: the fields given replace the dock's, the rest stay. Applied in one
// step under the dock board's lock (DockBoard::update), so two partial changes cannot undo each
// other.
struct DockPatch {
    std::optional<double> north_m;
    std::optional<double> east_m;
    std::optional<double> yaw_rad;

    bool empty() const { return !north_m && !east_m && !yaw_rad; }
};

// The marker as drawn, in marker-plane metres: measured off the image once it is loaded
// (MarkerTexture), at the sizes it is drawn at.
struct DockLayout {
    double outer_size_m{0.0};
    double inner_size_m{0.0};
    double cell_size_m{0.0};
    // The white plate around the outer marker — the image's own border — as a u/v box.
    double plate_u_min{0.0};
    double plate_u_max{0.0};
    double plate_v_min{0.0};
    double plate_v_max{0.0};
    // The inner marker's centre. The generator lays it out in whole pixels, so it is a fraction
    // of a millimetre off the outer marker's centre; this is where it really is.
    double inner_u_m{0.0};
    double inner_v_m{0.0};
};

// A marker-plane point in NED (north, east).
inline std::array<double, 2> dock_to_ned(const DockPose &pose, double u, double v) {
    const double s = std::sin(pose.yaw_rad);
    const double c = std::cos(pose.yaw_rad);
    // Right is (-sin, cos) in (north, east), up is (cos, sin), and v runs down.
    return {pose.north_m - u * s - v * c, pose.east_m + u * c - v * s};
}

// A ground point (north, east) in marker-plane coordinates (u, v); the inverse of dock_to_ned.
inline std::array<double, 2> ned_to_dock(const DockPose &pose, double north_m, double east_m) {
    const double s = std::sin(pose.yaw_rad);
    const double c = std::cos(pose.yaw_rad);
    const double dn = north_m - pose.north_m;
    const double de = east_m - pose.east_m;
    return {-dn * s + de * c, -(dn * c + de * s)};
}

// The corners of a square of side `size_m` centred at (u, v) on the marker, in NED, in the order
// cv2.aruco reports a marker's corners: the image's top-left, top-right, bottom-right,
// bottom-left.
inline std::array<std::array<double, 2>, 4> dock_corners_ned(const DockPose &pose, double u, double v,
                                                             double size_m) {
    const double h = size_m * 0.5;
    return {dock_to_ned(pose, u - h, v - h), dock_to_ned(pose, u + h, v - h), dock_to_ned(pose, u + h, v + h),
            dock_to_ned(pose, u - h, v + h)};
}

} // namespace skysim::render
