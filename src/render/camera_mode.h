#pragma once
// What a vehicle's pilot camera draws, and how often (TASK-2661).
//
// A vehicle's picture has one size, the fleet's (--camera-size) unless it was given its own: what
// camera.jpg, camera.mjpg, the RTSP stream and the intrinsics have, from the first frame to the
// last (H.264 crops an odd width or height down to even, at the right and bottom edge). What changes is how
// it is drawn. A mode is the size the picture is drawn at — scaled up to the picture size when smaller — its
// frame rate and its stream's bitrate. A vehicle draws its normal mode (--camera-draw-size, --camera-fps),
// the landing mode (--camera-landing-*) while it is in the air near its dock, where the dock marker needs
// every pixel, and its normal mode at the rest rate (--camera-rest-fps) while it sits still on the ground. So
// a consumer sees the same endpoints, formats and picture size throughout: OpenCV's FFmpeg capture, for one,
// does not follow a picture that changes size mid-stream (it keeps returning the last frame of the old size).
#include <charconv>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace skysim::render {

struct CameraMode {
    // The size it is drawn at. Scaled to the vehicle's picture size when different (0: that size).
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
inline constexpr double kMinCameraFps = 1.0; // the H.264 stream's own floor (video/video_service.h)
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

// "WxH" as two whole numbers, exactly: nothing before, between or after them but the 'x' (a NUL
// inside a JSON string included), and each within int (a number too long is refused, not wrapped
// into range as sscanf's %d would).
inline bool parse_wxh(std::string_view text, int &width, int &height) {
    const char *end = text.data() + text.size();
    const auto w = std::from_chars(text.data(), end, width);
    if (w.ec != std::errc() || w.ptr == end || *w.ptr != 'x') {
        return false;
    }
    const auto h = std::from_chars(w.ptr + 1, end, height);
    return h.ec == std::errc() && h.ptr == end;
}

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
        return fail("fps must be 1..60");
    }
    if (patch.bitrate_kbps && *patch.bitrate_kbps != 0 &&
        (*patch.bitrate_kbps < kMinCameraBitrateKbps || *patch.bitrate_kbps > kMaxCameraBitrateKbps)) {
        return fail("bitrate_kbps must be 0 (the stream's own) or 64..8000");
    }
    return true;
}

// Whether a draw size has the picture's aspect ratio, the one rule the command line, a spawn's
// "camera" and POST .../camera all hold a draw size to: its height is within half a pixel of the
// picture's share of its width, or its width within half a pixel of the picture's share of its
// height (what a size given by one side alone is rounded to, whichever side that is, so one
// always passes: 33 high on a 1080x1920 picture is 19 wide, 18.56 rounded), or its width over its
// height is within 1 % of the picture's. A size of 0 is not a size.
inline bool same_aspect_ratio(int width, int height, int picture_width, int picture_height) {
    if (width <= 0 || height <= 0 || picture_width <= 0 || picture_height <= 0) {
        return false;
    }
    const double share = static_cast<double>(width) * picture_height / picture_width;
    if (std::fabs(static_cast<double>(height) - share) <= 0.5 + 1e-9) {
        return true;
    }
    const double across = static_cast<double>(height) * picture_width / picture_height;
    if (std::fabs(static_cast<double>(width) - across) <= 0.5 + 1e-9) {
        return true;
    }
    const double picture = static_cast<double>(picture_width) / picture_height;
    return std::fabs(static_cast<double>(width) / height - picture) <= 0.01 * picture;
}

// A change to one vehicle's camera: POST /vehicles "camera", POST and DELETE
// /instances/<n>/camera (RenderService::configure).
struct CameraSettingsPatch {
    bool reset{false}; // back to the fleet's settings first (both modes and the picture size)
    // The picture size. Best given at spawn: changing it on a vehicle somebody is reading changes
    // the size of the stream they are reading.
    std::optional<int> picture_width;
    std::optional<int> picture_height;
    CameraModePatch normal;
    CameraModePatch landing;
    std::optional<bool> landing_enabled;

    bool empty() const {
        return !reset && !picture_width && !picture_height && normal.empty() && landing.empty() &&
               !landing_enabled;
    }
};

// The picture size of a patch against the bounds, as check_mode_patch does a mode's.
inline bool check_picture_patch(const CameraSettingsPatch &patch, std::string *error) {
    CameraModePatch size;
    size.width = patch.picture_width;
    size.height = patch.picture_height;
    return check_mode_patch(size, error);
}

// One vehicle's camera, as GET /instances/<n>/camera reports it.
struct VehicleCameraState {
    int picture_width{0}; // what every frame of it is
    int picture_height{0};
    bool picture_own{false};
    // How many times the vehicle's picture size has changed since it spawned (0: never). Each JPEG
    // camera.jpg and camera.mjpg serve carries the revision it was drawn for, and the intrinsics and
    // GET .../camera name the current one, so a reader can tell the frame its calibration is for.
    // It counts within one vehicle id: a replacement at the same instance starts again from 0, so
    // the frames and the intrinsics name the vehicle too, and only the pair names one picture.
    uint64_t picture_revision{0};
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
    CameraMode now; // what it draws now (sizes resolved: never 0)
    bool landing_active{false};
    // At rest on the ground and drawn at the rest rate (rest_fps, 0 = no rest rate).
    bool resting{false};
    double rest_fps{0.0};
    bool watched{false};      // somebody asked for its pictures within the demand window
    double fps{0.0};          // frames it got per second, over the last couple of seconds
    double render_ms{0.0};    // its last drawn frame
    uint64_t frames_drawn{0}; // since it spawned
    uint64_t frames_reused{0};
};

} // namespace skysim::render
