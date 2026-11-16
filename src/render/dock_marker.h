#pragma once
// Observer 1's e-ArUco dock marker on the ground, for the pilot camera and the survey stills
// (TASK-2562): the texture it is drawn from, the markers in the world, and what a frame sees of
// them. Geometry and sizes: render/dock.h.
//
// The marker is a decal on the ground plane, not geometry: the rasteriser already intersects
// the ground analytically per pixel, and a ground pixel that lands on a marker's plate takes its
// albedo from the marker instead of the ground texture, then is lit like any other ground. So a
// marker costs nothing while no camera draws a frame, and a frame that does not see one pays a
// bounding-box test per ground pixel per marker in shot.
#include "render/camera.h"
#include "render/dock.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace skysim::render {

// Albedo of the marker's black and white, a matte print: white 0.80 lit by the scene's sun and
// sky (render/shading.h) is about 205 of 255 and black about 8 — well clear of clipping, and a
// contrast a detector's threshold has no trouble with.
inline constexpr double kMarkerBlack = 0.03;
inline constexpr double kMarkerWhite = 0.80;

// The marker's picture as drawn: the generator's image, with the inner marker re-sampled to the
// configured inner size about its own centre (the outer marker is drawn at the image's own
// scale, so its cells stay exactly the generator's), padded to a power of two and mip-mapped.
//
// Sampling is a box filter over the pixel's footprint, which is what a sensor pixel integrates:
// on the finest mip level coarser than the footprint, over the 2 to 3 texels the box overlaps,
// with exact overlap weights. Up close that is the image's own texels, so an edge stays one
// pixel sharp at any magnification instead of being smeared by bilinear filtering.
class MarkerTexture {
  public:
    // From an 8-bit grey image (row-major, `width` x `height`). Null with `error` set when it is
    // not a generator e-ArUco image or `spec` cannot be drawn on it.
    static std::shared_ptr<const MarkerTexture> build(const uint8_t *grey, int width, int height,
                                                      const DockMarkerSpec &spec, std::string &error);
    // The same from PNG bytes.
    static std::shared_ptr<const MarkerTexture> from_png(const uint8_t *png, size_t size,
                                                         const DockMarkerSpec &spec, std::string &error);

    const DockLayout &layout() const { return layout_; }
    const DockMarkerSpec &spec() const { return spec_; }
    int size() const { return sizes_.empty() ? 0 : sizes_[0]; } // level-0 side, texels
    size_t levels() const { return levels_.size(); }
    double texels_per_m() const { return texels_per_m_; }

    // Grey level, 0 (black) .. 1 (white), at marker-plane point (u, v) metres, box-filtered for a
    // pixel `footprint_m` across. Off the image it is white, like the plate's border.
    double sample(double u_m, double v_m, double footprint_m) const;

    // Level-0 texel at (x, y), for the tests.
    uint8_t texel(int x, int y) const;

  private:
    MarkerTexture() = default;
    double box(size_t level, double x, double y, double width) const;

    DockMarkerSpec spec_;
    DockLayout layout_;
    double texels_per_m_{1.0};
    double centre_x_{0.0}; // the outer marker's centre, level-0 texels
    double centre_y_{0.0};
    std::vector<int> sizes_;
    std::vector<std::vector<uint8_t>> levels_;
};

// The marker this build carries (assets/markers/), for `spec`: decoded and built on first use —
// by the first frame that has a marker in shot or the first ground-truth request, never at
// startup — and shared after that. Null when the image is unusable (logged once).
std::shared_ptr<const MarkerTexture> dock_marker_texture(const DockMarkerSpec &spec);

// The embedded image's bytes (the PNG committed under assets/markers/).
const uint8_t *dock_marker_png();
size_t dock_marker_png_size();

// One marker on the ground.
struct GroundMarker {
    uint32_t vehicle_id{0};
    DockPose pose;
    DockMarkerSpec spec;
    // Drawn from this texture; null draws the marker this build carries (dock_marker_texture).
    std::shared_ptr<const MarkerTexture> texture;
};
using GroundMarkers = std::vector<GroundMarker>;

// The markers one frame can see, each with the screen box it can appear in: worked out once a
// frame, so a ground pixel tests a box per marker in shot rather than doing any geometry.
class MarkerView {
  public:
    MarkerView() = default;
    MarkerView(const GroundMarkers *markers, const Camera &camera);

    bool empty() const { return visible_.empty(); }
    size_t size() const { return visible_.size(); }

    // How much of pixel (x, y), whose ground point is (north, east) and footprint `footprint_m`,
    // a marker's plate covers (0..1), and the plate's albedo there. 0 when no plate is under it.
    double albedo(int x, int y, double north_m, double east_m, double footprint_m, double &albedo) const;

  private:
    struct Visible {
        const MarkerTexture *texture;
        DockPose pose;
        int x0, x1, y0, y1; // inclusive screen box
    };
    std::vector<Visible> visible_;
    std::vector<std::shared_ptr<const MarkerTexture>> textures_; // kept alive for the frame
};

// Every vehicle's dock, for the renderers and the control plane. A vehicle gets one at spawn, at
// its home (where it spawned, heading north); the control plane can move it and put it back;
// it goes with the vehicle. Not world state — no tick reads it — so any thread may use it: the
// tick thread adds and removes, HTTP threads move and read, render threads take seen_by().
class DockBoard {
  public:
    struct Entry {
        DockPose pose;
        DockPose home;
        bool at_home{true};
    };

    explicit DockBoard(const DockMarkerSpec &spec, DockVisibility visibility = DockVisibility::kOwn)
        : spec_(spec), visibility_(visibility) {}

    const DockMarkerSpec &spec() const { return spec_; }
    DockVisibility visibility() const { return visibility_; }

    // A vehicle spawned at (north, east): its dock is there, heading north.
    void add(uint32_t vehicle_id, double home_north_m, double home_east_m);
    void remove(uint32_t vehicle_id);

    // A vehicle's dock; nothing for a vehicle that has none (not spawned, or gone).
    std::optional<Entry> get(uint32_t vehicle_id) const;
    std::optional<Entry> set(uint32_t vehicle_id, const DockPose &pose);
    // The given fields of `patch` replace the dock's, in one step under the lock.
    std::optional<Entry> update(uint32_t vehicle_id, const DockPatch &patch);
    std::optional<Entry> reset(uint32_t vehicle_id); // back home

    // Every dock, as one immutable list: what a view that is no vehicle's own camera draws. The
    // same pointer until a dock changes.
    std::shared_ptr<const GroundMarkers> markers() const;

    // What vehicle `vehicle_id`'s own cameras draw (the pilot camera, its stills): its own dock
    // alone under kOwn — empty for a vehicle that has none — and every dock (markers()) under
    // kAll. Immutable, and the same pointer until that list changes, so a renderer can tell a
    // parked vehicle's picture is still current by comparing it: under kOwn another vehicle's
    // dock arriving, moving or going does not change it.
    std::shared_ptr<const GroundMarkers> seen_by(uint32_t vehicle_id) const;

  private:
    // After vehicle `vehicle_id`'s dock changed, arrived or went: the lists that hold it.
    void publish_locked(uint32_t vehicle_id);

    DockMarkerSpec spec_;
    DockVisibility visibility_;
    mutable std::mutex mutex_;
    std::map<uint32_t, Entry> entries_;
    std::shared_ptr<const GroundMarkers> markers_{std::make_shared<const GroundMarkers>()};
    // kOwn: each vehicle's own dock, as a list of one.
    std::map<uint32_t, std::shared_ptr<const GroundMarkers>> own_;
    std::shared_ptr<const GroundMarkers> none_{std::make_shared<const GroundMarkers>()};
};

} // namespace skysim::render
