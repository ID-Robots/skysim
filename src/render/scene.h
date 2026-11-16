#pragma once

#include "core/world.h"
#include "render/texture.h"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace skysim::render {

// What a pixel shows, for callers that need to know rather than guess from its colour: a
// scene triangle index (>= 0), or one of these.
inline constexpr int32_t kSurfaceGround = -1;
inline constexpr int32_t kSurfaceSky = -2;
inline constexpr int32_t kSurfaceUnknown = -3; // geometry the scene has no triangle for

// The static world as both renderers want it: triangles with their normals, what each one
// belongs to (which building, which face of it, which of its edges are the face's outline),
// how it is textured, and a grid for casting shadow rays through it. Built once, read-only
// afterwards, so any number of render threads can share one.
//
// The ground is not in here as triangles. The physics ground is a 4 km slab, and two
// triangles 4 km across are the worst thing to feed a rasteriser — one corner behind the
// camera and the whole ground used to vanish. Renderers intersect the plane NED z = 0
// analytically instead, which also means there is ground to draw where no tile is loaded.
class RasterScene {
  public:
    // Takes ownership: the triangle list is ~24 MB on a city-sized world and copying it in
    // was most of the memory this scene added. Ground-slab triangles are dropped (above).
    void build(std::vector<core::World::Triangle> triangles, uint32_t texture_seed = 1);

    size_t triangle_count() const { return tris_.size(); }

    const std::vector<core::World::Triangle> &triangles() const { return tris_; }
    const std::vector<core::Vec3> &normals() const { return normals_; }

    // Centroids, split by axis as floats.
    //
    // The range reject reads these for every triangle every frame — the one part of the frame
    // that scales with the whole world rather than with what is in shot. Keeping them
    // contiguous and narrow turns that scan from streaming 72-byte structs into tight arrays.
    const std::vector<float> &centroid_x() const { return cx_; }
    const std::vector<float> &centroid_y() const { return cy_; }
    const std::vector<float> &centroid_z() const { return cz_; }

    // Distance from each centroid to its furthest vertex, read alongside them by the same
    // reject: a triangle is only out of range once all of it is, or a building on the
    // draw-distance boundary sheds the triangles whose middles fall outside.
    const std::vector<float> &radius() const { return radius_; }

    const WorldTexture &texture() const { return texture_; }

    // Buildings are connected pieces of geometry; each has a key derived from its lowest
    // corner in world coordinates, so it keeps its colours whatever order tiles load in.
    size_t building_count() const { return building_count_; }
    uint32_t building_of(size_t tri) const { return tri_building_[tri]; }
    const SurfaceFrame &frame_of(size_t tri) const { return frames_[tri_frame_[tri]]; }

    // Albedo at a world point on triangle `tri`, or on the ground plane when tri < 0,
    // filtered for a pixel `footprint_m` across.
    void albedo(int tri, const core::Vec3 &p, double footprint_m, double out[3]) const;

    // Whether the sun is blocked at `p`: a ray from it towards the sun hits geometry.
    // `tri` is the triangle p lies on (-1 for the ground), which the ray must not hit.
    bool in_shadow(const core::Vec3 &p, int tri) const;

    // What a camera sees at p before haze: albedo under sky light, plus sun where it reaches.
    // A function of the world point alone — the same from every camera.
    void lit_colour(int tri, const core::Vec3 &p, double footprint_m, double out[3]) const;

    // The light at p applied to an albedo already in `colour` (lit_colour without the albedo):
    // for something lying on a surface, like a dock marker on the ground.
    void light(int tri, const core::Vec3 &p, double colour[3]) const;

    // The triangle a point on a building lies on (within tolerance_m of its plane), or -1.
    int find_triangle(const core::Vec3 &p, double tolerance_m = 0.02) const;

    // Distance from p to the nearest outline edge of its face. Roofs measure against every
    // outline edge of the whole face (a parapet must follow the roof's edge, not the edges of
    // whichever triangle of it p happens to be in); other faces through p's own triangle.
    double edge_distance(size_t tri, const core::Vec3 &p) const;

    // Highest point of any building, metres above the ground plane.
    double max_height() const { return grid_.max_height; }

  private:
    void build_identity();
    void build_grid();
    bool ray_hits(size_t tri, const core::Vec3 &o, const core::Vec3 &d, double t_min) const;

    std::vector<core::World::Triangle> tris_;
    std::vector<core::Vec3> normals_;
    std::vector<float> cx_, cy_, cz_, radius_;

    std::vector<uint32_t> tri_building_;
    std::vector<uint32_t> tri_frame_;
    std::vector<uint8_t> tri_outline_;               // bit k: edge opposite vertex k is outline
    std::vector<std::array<float, 3>> tri_altitude_; // vertex k to the opposite edge, metres
    std::vector<SurfaceFrame> frames_;
    // Outline edges of each roof face, as segments: frame f owns
    // roof_outline_[roof_outline_start_[f] .. roof_outline_start_[f + 1]).
    std::vector<uint32_t> roof_outline_start_;
    std::vector<std::array<core::Vec3, 2>> roof_outline_;
    size_t building_count_{0};
    WorldTexture texture_{1};

    // Uniform 2D grid over the buildings' footprint, holding every triangle that can cast a
    // shadow, plus the tallest point in each cell so a shadow ray skips cells it passes over.
    struct Grid {
        double origin_n{0.0};
        double origin_e{0.0};
        double cell{4.0};
        int64_t rows{0}; // along north
        int64_t cols{0}; // along east
        std::vector<uint32_t> start;
        std::vector<uint32_t> items;
        std::vector<float> top;
        double max_height{0.0};
    } grid_;
};

// Load every tile in `tiles_dir` (none when it is empty) and build a scene from it.
// `tiles_loaded` reports how many tiles that was.
std::shared_ptr<const RasterScene> load_scene(const std::string &tiles_dir, uint32_t texture_seed,
                                              size_t *tiles_loaded = nullptr);

} // namespace skysim::render
