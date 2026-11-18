#pragma once
// What a vehicle's pilot camera draws: its picture size and frame rate (TASK-2661).
//
// Every vehicle draws the fleet's camera (--camera-size, --camera-fps) unless it was given its own
// (POST /vehicles "camera", POST /instances/<n>/camera), and switches to the landing camera
// (--camera-landing-*) while it is in the air close to its dock, so the full-size pictures the dock
// marker needs are only drawn where a landing can use them. The consumer sees one stream whose
// picture size changes there; nothing else about it does.
#include <cstdint>
#include <optional>
#include <string>

namespace skysim::render {

struct CameraMode {
    int width{0};
    int height{0};
    double fps{0.0};
    // The H.264 bitrate the vehicle's RTSP stream runs at while this mode is drawn; 0 keeps the
    // stream's own (POST /instances/<n>/camera/encoder, default 1200). A 1920x1080 marker at 30 m
    // has 4 px cells, and 1200 kbit/s blurs one out of a frame now and then (docs/DOCK_MARKER.md).
    int bitrate_kbps{0};

    bool operator==(const CameraMode &) const = default;
};

// The bounds a mode set through the control plane is held to. 3840x2160 is the largest picture a
// still is drawn at too, and 60 fps is twice what any consumer of the pilot camera reads.
inline constexpr int kMinCameraSide = 16;
inline constexpr int kMaxCameraWidth = 3840;
inline constexpr int kMaxCameraHeight = 2160;
inline constexpr double kMinCameraFps = 0.5;
inline constexpr double kMaxCameraFps = 60.0;
inline constexpr int kMinCameraBitrateKbps = 64; // the encoder route's own bounds
inline constexpr int kMaxCameraBitrateKbps = 8000;

// A change to a mode: the fields given replace the mode's, the rest stay.
struct CameraModePatch {
    std::optional<int> width;
    std::optional<int> height;
    std::optional<double> fps;
    std::optional<int> bitrate_kbps;

    bool empty() const { return !width && !height && !fps && !bitrate_kbps; }
};

// The given fields of `patch` against the bounds above. False with `error` naming the first one
// out of them. (A field the patch does not give is not checked: the fleet's own settings, from
// the command line, may lie outside what the API accepts.)
inline bool check_mode_patch(const CameraModePatch &patch, std::string *error) {
    const auto fail = [error](const char *why) {
        if (error != nullptr) {
            *error = why;
        }
        return false;
    };
    if (patch.width && (*patch.width < kMinCameraSide || *patch.width > kMaxCameraWidth)) {
        return fail("width must be 16..3840");
    }
    if (patch.height && (*patch.height < kMinCameraSide || *patch.height > kMaxCameraHeight)) {
        return fail("height must be 16..2160");
    }
    if (patch.fps && !(*patch.fps >= kMinCameraFps && *patch.fps <= kMaxCameraFps)) {
        return fail("fps must be 0.5..60");
    }
    if (patch.bitrate_kbps && *patch.bitrate_kbps != 0 &&
        (*patch.bitrate_kbps < kMinCameraBitrateKbps || *patch.bitrate_kbps > kMaxCameraBitrateKbps)) {
        return fail("bitrate_kbps must be 0 (the stream's own) or 64..8000");
    }
    return true;
}

// A change to one vehicle's camera: POST /vehicles "camera", POST and DELETE
// /instances/<n>/camera (RenderService::configure).
struct CameraSettingsPatch {
    bool reset{false}; // back to the fleet's settings first (both modes)
    CameraModePatch normal;
    CameraModePatch landing;
    std::optional<bool> landing_enabled;

    bool empty() const { return !reset && normal.empty() && landing.empty() && !landing_enabled; }
};

// One vehicle's camera, as GET /instances/<n>/camera reports it.
struct VehicleCameraState {
    CameraMode normal;
    bool normal_own{false}; // its own, not the fleet's
    // The landing camera: available when the fleet has one or the vehicle was given one, and not
    // turned off for the vehicle.
    bool landing_available{false};
    bool landing_enabled{true};
    CameraMode landing;
    bool landing_own{false};
    double landing_radius_m{0.0};
    double landing_height_m{0.0};
    CameraMode now; // what it draws now
    bool landing_active{false};
    bool watched{false};      // somebody asked for its pictures within the demand window
    double fps{0.0};          // frames it got per second, over the last couple of seconds
    double render_ms{0.0};    // its last drawn frame
    uint64_t frames_drawn{0}; // since it spawned
    uint64_t frames_reused{0};
};

} // namespace skysim::render
