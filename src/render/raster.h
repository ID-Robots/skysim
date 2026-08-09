#pragma once

#include "core/thread_pool.h"
#include "core/world.h"
#include "render/camera.h"
#include "render/image.h"

#include <cstdint>
#include <vector>

namespace skysim::render {

// A triangle already projected into screen space, ready to fill.
struct ScreenTriangle {
    // Pixel coordinates, and 1/depth for perspective-correct interpolation.
    float x[3];
    float y[3];
    float inv_w[3];
    float shade;    // flat Lambert term, computed once per triangle
    bool is_ground;
    int min_y;
    int max_y;
};

// Scene geometry in a form the rasteriser wants: triangles plus their
// precomputed normals, so the world is walked once rather than once per frame.
class RasterScene {
  public:
    // Takes ownership: the triangle list is ~24 MB on a city-sized world and
    // copying it in was most of the memory this scene added.
    void build(std::vector<core::World::Triangle> triangles);

    size_t triangle_count() const { return tris_.size(); }

    const std::vector<core::World::Triangle> &triangles() const { return tris_; }
    const std::vector<core::Vec3> &normals() const { return normals_; }

    // Centroids, split by axis as floats.
    //
    // The range reject reads these for every triangle every frame — the one
    // part of the frame that scales with the whole world rather than with what
    // is in shot. Keeping them contiguous and narrow turns that scan from
    // streaming 72-byte structs into three tight float arrays.
    const std::vector<float> &centroid_x() const { return cx_; }
    const std::vector<float> &centroid_y() const { return cy_; }
    const std::vector<float> &centroid_z() const { return cz_; }

    // Distance from each centroid to its furthest vertex, read alongside them by
    // the same reject: a triangle is only out of range once all of it is, or a
    // building on the draw-distance boundary sheds the triangles whose middles
    // fall outside and stands there with holes in it.
    const std::vector<float> &radius() const { return radius_; }

  private:
    std::vector<core::World::Triangle> tris_;
    std::vector<core::Vec3> normals_;
    std::vector<float> cx_, cy_, cz_, radius_;
};

// Draws the world by filling triangles instead of chasing a ray per pixel.
//
// The ray caster this replaces cost about 4.6 us per pixel, because every pixel
// walked the physics broadphase. The whole city is only ~330k triangles and a
// frame sees a few thousand of them, so filling those is a different order of
// cost entirely — the work stops scaling with resolution and starts scaling with
// what is actually in shot.
//
// Threading: the screen is cut into horizontal bands and each worker owns its
// band outright. A band's pixels and depth values are touched by exactly one
// thread, so there are no atomics and no locks in the inner loop — the parts
// that would otherwise serialise a rasteriser.
//
// Triangles are projected once, serially, then every band scans that list and
// skips what does not overlap its rows. Binning them per band up front would
// save the scan; at a few thousand visible triangles it has not been worth it.
class Rasterizer {
  public:
    explicit Rasterizer(const CameraConfig &cfg) : cfg_(cfg) {}

    Image render(const RasterScene &scene, const Camera &camera, core::ThreadPool *pool) const;

  private:
    // Project and cull, producing screen triangles for the bands that need them.
    void transform(const RasterScene &scene, const Camera &camera, std::vector<ScreenTriangle> &out) const;

    // Fill one horizontal band. Owns every pixel it writes.
    void fill_band(const std::vector<ScreenTriangle> &tris, Image &image, std::vector<float> &depth, int y0,
                   int y1) const;

    void draw_background(const Camera &camera, Image &image, int y0, int y1) const;

    CameraConfig cfg_;
};

} // namespace skysim::render
