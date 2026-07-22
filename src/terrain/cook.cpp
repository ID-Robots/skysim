// See cook.h. Minimal OBJ reader: 'v x y z' and 'f a b c ...' (1-based, optional /vt/vn
// suffixes, polygons fan-triangulated). That is the full contract with tools/cooker/pretile.py
// and the M5 decimation pre-step.
#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/RegisterTypes.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <mutex>
#include <sstream>
#include <vector>

#include "terrain/cook.h"

namespace skysim::terrain {

namespace {

void init_jolt_once() {
    static std::once_flag flag;
    std::call_once(flag, [] {
        JPH::RegisterDefaultAllocator();
        if (JPH::Factory::sInstance == nullptr) {
            JPH::Factory::sInstance = new JPH::Factory();
            JPH::RegisterTypes();
        }
    });
}

// ENU (x east, y north, z up) -> Jolt (x north, y up, z east): cyclic (y, z, x).
JPH::Float3 enu_to_jolt(double e, double n, double u) {
    return {static_cast<float>(n), static_cast<float>(u), static_cast<float>(e)};
}

// Jolt (n, u, e) -> NED (n, e, -u), for the AABB bookkeeping.
std::array<double, 3> jolt_to_ned(const JPH::Vec3 &j) { return {j.GetX(), j.GetZ(), -j.GetY()}; }

} // namespace

bool cook_obj_tile(const std::filesystem::path &obj_in, const std::filesystem::path &jshape_out,
                   CookResult *result, std::string *error) {
    init_jolt_once();

    std::ifstream in(obj_in);
    if (!in) {
        *error = "cannot open " + obj_in.string();
        return false;
    }

    std::vector<JPH::Float3> verts;
    JPH::TriangleList tris;
    std::string line;
    size_t line_no = 0;
    while (std::getline(in, line)) {
        ++line_no;
        if (line.size() < 2) {
            continue;
        }
        if (line[0] == 'v' && line[1] == ' ') {
            double x, y, z;
            if (std::sscanf(line.c_str() + 2, "%lf %lf %lf", &x, &y, &z) != 3) {
                *error = obj_in.string() + ":" + std::to_string(line_no) + ": bad vertex";
                return false;
            }
            verts.push_back(enu_to_jolt(x, y, z));
        } else if (line[0] == 'f' && line[1] == ' ') {
            std::istringstream ss(line.substr(2));
            std::vector<uint32_t> idx;
            std::string tok;
            while (ss >> tok) {
                const long v = std::strtol(tok.c_str(), nullptr, 10); // "12/3/4" -> 12
                if (v <= 0 || static_cast<size_t>(v) > verts.size()) {
                    *error = obj_in.string() + ":" + std::to_string(line_no) + ": bad face index";
                    return false;
                }
                idx.push_back(static_cast<uint32_t>(v - 1));
            }
            if (idx.size() < 3) {
                *error = obj_in.string() + ":" + std::to_string(line_no) + ": face with <3 verts";
                return false;
            }
            for (size_t k = 1; k + 1 < idx.size(); ++k) { // fan triangulation, winding kept
                tris.push_back(JPH::Triangle(verts[idx[0]], verts[idx[k]], verts[idx[k + 1]]));
            }
        }
    }
    if (tris.empty()) {
        *error = obj_in.string() + ": no faces";
        return false;
    }

    JPH::MeshShapeSettings settings(tris);
    JPH::Shape::ShapeResult shape_result = settings.Create();
    if (shape_result.HasError()) {
        *error = "MeshShape: " + std::string(shape_result.GetError().c_str());
        return false;
    }

    std::ofstream out(jshape_out, std::ios::binary);
    if (!out) {
        *error = "cannot write " + jshape_out.string();
        return false;
    }
    JPH::StreamOutWrapper stream(out);
    shape_result.Get()->SaveBinaryState(stream);

    if (result != nullptr) {
        result->triangles = tris.size();
        const JPH::AABox box = shape_result.Get()->GetLocalBounds();
        // Jolt AABB corners -> NED (note: NED z flips, so min/max swap on that axis).
        const auto a = jolt_to_ned(box.mMin);
        const auto b = jolt_to_ned(box.mMax);
        for (int k = 0; k < 3; ++k) {
            result->aabb_min_ned[k] = std::min(a[k], b[k]);
            result->aabb_max_ned[k] = std::max(a[k], b[k]);
        }
    }
    return true;
}

} // namespace skysim::terrain
