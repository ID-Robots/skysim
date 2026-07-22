// Jolt world implementation. Everything below the World interface is Jolt Y-up; every value
// crossing the interface is NED/FRD via core/frames.h. CLAUDE.md "Jolt-specific rules" apply.
#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <random>
#include <thread>
#include <unordered_map>
#include <vector>

#include "core/world.h"

namespace skysim::core {

namespace {

namespace object_layers {
constexpr JPH::ObjectLayer kStatic = 0;
constexpr JPH::ObjectLayer kMoving = 1;
constexpr JPH::ObjectLayer kCount = 2;
} // namespace object_layers

namespace bp_layers {
const JPH::BroadPhaseLayer kStatic(0);
const JPH::BroadPhaseLayer kMoving(1);
constexpr uint32_t kCount = 2;
} // namespace bp_layers

class BpLayerInterface final : public JPH::BroadPhaseLayerInterface {
  public:
    uint32_t GetNumBroadPhaseLayers() const override { return bp_layers::kCount; }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override {
        return layer == object_layers::kStatic ? bp_layers::kStatic : bp_layers::kMoving;
    }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char *GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override {
        return layer == bp_layers::kStatic ? "STATIC" : "MOVING";
    }
#endif
};

class ObjectVsBpFilter final : public JPH::ObjectVsBroadPhaseLayerFilter {
  public:
    bool ShouldCollide(JPH::ObjectLayer layer, JPH::BroadPhaseLayer bp) const override {
        // Vehicles collide with both layers; static never moves so it only pairs with MOVING.
        return layer == object_layers::kMoving || bp == bp_layers::kMoving;
    }
};

class ObjectPairFilter final : public JPH::ObjectLayerPairFilter {
  public:
    bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override {
        return a == object_layers::kMoving || b == object_layers::kMoving;
    }
};

// Factory/type registration is process-global; init once and keep it. The nullptr guard
// coordinates with terrain/cook.cpp, which may have initialized Jolt first in-process.
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

JPH::Vec3 to_jolt(const Vec3 &v) {
    const Vec3 j = ned_to_jolt(v);
    return {static_cast<float>(j[0]), static_cast<float>(j[1]), static_cast<float>(j[2])};
}

Vec3 from_jolt(const JPH::Vec3 &v) { return jolt_to_ned({v.GetX(), v.GetY(), v.GetZ()}); }

[[maybe_unused]] JPH::Quat to_jolt(const Quat &q_ned) {
    const Quat j = quat_ned_to_jolt(q_ned);
    // JPH::Quat is (x, y, z, w); ours is (w, x, y, z).
    return {static_cast<float>(j[1]), static_cast<float>(j[2]), static_cast<float>(j[3]),
            static_cast<float>(j[0])};
}

Quat from_jolt(const JPH::Quat &q) {
    return quat_jolt_to_ned(quat_normalize({q.GetW(), q.GetX(), q.GetY(), q.GetZ()}));
}

constexpr double kGravity = 9.81;
constexpr double kAccelClamp = 16.0 * kGravity; // per DESIGN.md ground-contact notes

struct VehicleEntry {
    JPH::BodyID body;
    double mass_kg = 0.0;
    Vec3 prev_vel_ned{}; // for specific-force finite difference
};

} // namespace

struct World::Impl {
    BpLayerInterface bp_interface;
    ObjectVsBpFilter obj_vs_bp;
    ObjectPairFilter obj_pair;
    std::unique_ptr<JPH::TempAllocatorImpl> temp_allocator;
    std::unique_ptr<JPH::JobSystemThreadPool> job_system;
    std::unique_ptr<JPH::PhysicsSystem> physics;

    double dt_s = 1.0 / 400.0;
    uint64_t tick = 0;
    uint32_t next_id = 1;
    std::unordered_map<uint32_t, VehicleEntry> vehicles;
    std::vector<JPH::BodyID> static_bodies;

    // Wind: steady + OU gusts, all randomness from this world-owned PRNG (invariant 4).
    Vec3 wind_steady{};
    Vec3 gust{};
    double gust_sigma = 0.0;
    double gust_tau = 2.0;
    std::mt19937_64 rng;
    std::normal_distribution<double> gauss{0.0, 1.0};

    struct PendingWrench {
        uint32_t id;
        Vec3 force_frd;
        Vec3 torque_frd;
    };
    std::vector<PendingWrench> pending;

    JPH::BodyInterface &bodies() { return physics->GetBodyInterface(); }
    const JPH::BodyInterface &bodies() const { return physics->GetBodyInterface(); }
};

World::World(const WorldConfig &cfg) : impl_(std::make_unique<Impl>()), dt_s_(cfg.dt_s) {
    init_jolt_once();
    impl_->dt_s = cfg.dt_s;

    impl_->temp_allocator = std::make_unique<JPH::TempAllocatorImpl>(32 * 1024 * 1024);
    const int hw = static_cast<int>(std::thread::hardware_concurrency());
    const int threads = cfg.worker_threads > 0 ? cfg.worker_threads : (hw > 1 ? hw - 1 : 1);
    impl_->job_system =
        std::make_unique<JPH::JobSystemThreadPool>(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, threads);

    impl_->physics = std::make_unique<JPH::PhysicsSystem>();
    impl_->physics->Init(cfg.max_bodies, 0, cfg.max_bodies * 2, cfg.max_bodies * 2, impl_->bp_interface,
                         impl_->obj_vs_bp, impl_->obj_pair);
    // NED gravity (0,0,+g) is Jolt (0,-g,0) — Jolt's default, but set it explicitly.
    impl_->physics->SetGravity(JPH::Vec3(0.0f, -static_cast<float>(kGravity), 0.0f));

    impl_->wind_steady = cfg.wind_steady_ned;
    impl_->gust_sigma = cfg.gust_sigma_mps;
    impl_->gust_tau = cfg.gust_tau_s > 0.0 ? cfg.gust_tau_s : 2.0;
    impl_->rng.seed(cfg.rng_seed);
}

World::~World() = default;

void World::add_ground_plane() {
    // 4 km x 4 km slab whose top face is NED z = 0.
    JPH::BoxShapeSettings shape_settings(JPH::Vec3(2000.0f, 1.0f, 2000.0f));
    auto shape = shape_settings.Create();
    JPH::BodyCreationSettings s(shape.Get(), JPH::RVec3(0.0f, -1.0f, 0.0f), JPH::Quat::sIdentity(),
                                JPH::EMotionType::Static, object_layers::kStatic);
    s.mFriction = 0.8f;
    s.mRestitution = 0.0f;
    const JPH::BodyID id = impl_->bodies().CreateAndAddBody(s, JPH::EActivation::DontActivate);
    impl_->static_bodies.push_back(id);
}

size_t World::load_tiles(const std::filesystem::path &dir) {
    size_t loaded = 0;
    std::error_code ec;
    if (!std::filesystem::is_directory(dir, ec)) {
        std::fprintf(stderr, "world: --tiles path is not a directory: %s\n", dir.c_str());
        return 0;
    }
    for (const auto &entry : std::filesystem::directory_iterator(dir, ec)) {
        if (entry.path().extension() != ".jshape") {
            continue;
        }
        std::ifstream in(entry.path(), std::ios::binary);
        if (!in) {
            std::fprintf(stderr, "world: cannot open tile %s\n", entry.path().c_str());
            continue;
        }
        JPH::StreamInWrapper stream(in);
        JPH::Shape::ShapeResult result = JPH::Shape::sRestoreFromBinaryState(stream);
        if (result.HasError()) {
            std::fprintf(stderr, "world: tile %s: %s\n", entry.path().c_str(), result.GetError().c_str());
            continue;
        }
        // Tiles are cooked in Jolt frame with world-absolute vertices; place at identity.
        JPH::BodyCreationSettings s(result.Get(), JPH::RVec3::sZero(), JPH::Quat::sIdentity(),
                                    JPH::EMotionType::Static, object_layers::kStatic);
        s.mFriction = 0.8f;
        s.mRestitution = 0.0f;
        impl_->static_bodies.push_back(impl_->bodies().CreateAndAddBody(s, JPH::EActivation::DontActivate));
        ++loaded;
    }
    return loaded;
}

double World::raycast(const Vec3 &origin_ned, const Vec3 &dir_ned, double max_dist_m,
                      uint32_t ignore_vehicle_id) const {
    const JPH::RVec3 origin(to_jolt(origin_ned));
    const JPH::Vec3 dir = to_jolt(dir_ned) * static_cast<float>(max_dist_m);
    JPH::RRayCast ray{origin, dir};
    JPH::RayCastResult hit;
    JPH::BodyID ignore;
    if (ignore_vehicle_id != 0) {
        auto it = impl_->vehicles.find(ignore_vehicle_id);
        if (it != impl_->vehicles.end()) {
            ignore = it->second.body;
        }
    }
    const JPH::IgnoreSingleBodyFilter body_filter(ignore);
    if (impl_->physics->GetNarrowPhaseQuery().CastRay(ray, hit, JPH::BroadPhaseLayerFilter{},
                                                      JPH::ObjectLayerFilter{}, body_filter)) {
        return hit.mFraction * max_dist_m;
    }
    return -1.0;
}

uint32_t World::add_vehicle(const VehicleBodyParams &p) {
    // FRD half-extents/inertia to Jolt body local (x fwd, y up, z right): swap y/z, negate none
    // (extents are magnitudes).
    const JPH::Vec3 half(static_cast<float>(p.half_extents_frd[0]), static_cast<float>(p.half_extents_frd[2]),
                         static_cast<float>(p.half_extents_frd[1]));
    JPH::BoxShapeSettings shape_settings(half, JPH::cDefaultConvexRadius);
    auto shape = shape_settings.Create();

    JPH::BodyCreationSettings s(shape.Get(), JPH::RVec3(to_jolt(p.start_pos_ned)), JPH::Quat::sIdentity(),
                                JPH::EMotionType::Dynamic, object_layers::kMoving);
    s.mOverrideMassProperties = JPH::EOverrideMassProperties::MassAndInertiaProvided;
    s.mMassPropertiesOverride.mMass = static_cast<float>(p.mass_kg);
    s.mMassPropertiesOverride.mInertia = JPH::Mat44::sScale(
        JPH::Vec3(static_cast<float>(p.inertia_diag_frd[0]), static_cast<float>(p.inertia_diag_frd[2]),
                  static_cast<float>(p.inertia_diag_frd[1])));
    s.mAllowSleeping = false;                           // a hovering quad must never be put to sleep
    s.mMotionQuality = JPH::EMotionQuality::LinearCast; // no tunneling into walls at 25 m/s
    s.mLinearDamping = 0.0f;                            // drag is modeled in vehicle/quad.cpp
    s.mAngularDamping = 0.0f;
    s.mFriction = 0.6f;
    s.mRestitution = 0.0f;

    const uint32_t id = impl_->next_id++;
    VehicleEntry e;
    e.body = impl_->bodies().CreateAndAddBody(s, JPH::EActivation::Activate);
    e.mass_kg = p.mass_kg;
    impl_->vehicles.emplace(id, e);
    return id;
}

void World::set_frozen(uint32_t id, bool frozen) {
    auto it = impl_->vehicles.find(id);
    if (it == impl_->vehicles.end()) {
        return;
    }
    auto &bi = impl_->bodies();
    const JPH::BodyID body = it->second.body;
    if (frozen) {
        bi.SetLinearAndAngularVelocity(body, JPH::Vec3::sZero(), JPH::Vec3::sZero());
        bi.SetMotionType(body, JPH::EMotionType::Kinematic, JPH::EActivation::Activate);
    } else {
        bi.SetMotionType(body, JPH::EMotionType::Dynamic, JPH::EActivation::Activate);
    }
    it->second.prev_vel_ned = {0.0, 0.0, 0.0};
}

void World::remove_vehicle(uint32_t id) {
    auto it = impl_->vehicles.find(id);
    if (it == impl_->vehicles.end()) {
        return;
    }
    impl_->bodies().RemoveBody(it->second.body);
    impl_->bodies().DestroyBody(it->second.body);
    impl_->vehicles.erase(it);
}

void World::apply_body_wrench(uint32_t id, const Vec3 &force_frd, const Vec3 &torque_frd) {
    impl_->pending.push_back({id, force_frd, torque_frd});
}

void World::step() {
    auto &bi = impl_->bodies();
    for (const auto &w : impl_->pending) {
        auto it = impl_->vehicles.find(w.id);
        if (it == impl_->vehicles.end()) {
            continue;
        }
        const JPH::BodyID body = it->second.body;
        const JPH::Quat rot = bi.GetRotation(body);
        const JPH::Vec3 f_local(static_cast<float>(w.force_frd[0]), -static_cast<float>(w.force_frd[2]),
                                static_cast<float>(w.force_frd[1]));
        const JPH::Vec3 t_local(static_cast<float>(w.torque_frd[0]), -static_cast<float>(w.torque_frd[2]),
                                static_cast<float>(w.torque_frd[1]));
        bi.AddForce(body, rot * f_local);
        bi.AddTorque(body, rot * t_local);
    }
    impl_->pending.clear();

    for (auto &[id, v] : impl_->vehicles) {
        v.prev_vel_ned = from_jolt(bi.GetLinearVelocity(v.body));
    }

    // Advance gusts (exact OU discretization: stationary sigma, correlation time tau).
    // Draw all three gaussians even when sigma is 0 so enabling wind never shifts the
    // stream consumed by future randomness sources (determinism across configs).
    if (impl_->gust_sigma > 0.0 || impl_->gust_tau > 0.0) {
        const double decay = std::exp(-impl_->dt_s / impl_->gust_tau);
        const double diffuse = impl_->gust_sigma * std::sqrt(1.0 - decay * decay);
        for (int k = 0; k < 3; ++k) {
            impl_->gust[k] = impl_->gust[k] * decay + diffuse * impl_->gauss(impl_->rng);
        }
    }

    impl_->physics->Update(static_cast<float>(impl_->dt_s), 1, impl_->temp_allocator.get(),
                           impl_->job_system.get());
    ++impl_->tick;
}

Vec3 World::wind_ned() const {
    return {impl_->wind_steady[0] + impl_->gust[0], impl_->wind_steady[1] + impl_->gust[1],
            impl_->wind_steady[2] + impl_->gust[2]};
}

BodyState World::get_state(uint32_t id) const {
    BodyState out;
    auto it = impl_->vehicles.find(id);
    if (it == impl_->vehicles.end()) {
        return out;
    }
    const auto &bi = impl_->bodies();
    const VehicleEntry &v = it->second;

    out.pos_ned = from_jolt(bi.GetCenterOfMassPosition(v.body));
    out.vel_ned = from_jolt(bi.GetLinearVelocity(v.body));
    out.quat_ned_frd = from_jolt(bi.GetRotation(v.body));

    const Vec3 omega_ned = from_jolt(bi.GetAngularVelocity(v.body));
    out.gyro_frd = quat_rotate_inverse(out.quat_ned_frd, omega_ned);

    // Specific force from the velocity delta over the last step: includes motor, drag,
    // gravity AND contact impulses, which is exactly what an accelerometer measures.
    Vec3 f_spec_ned;
    for (int k = 0; k < 3; ++k) {
        const double accel = (out.vel_ned[k] - v.prev_vel_ned[k]) / impl_->dt_s;
        f_spec_ned[k] = accel - (k == 2 ? kGravity : 0.0);
    }
    out.accel_body_frd = quat_rotate_inverse(out.quat_ned_frd, f_spec_ned);
    for (double &a : out.accel_body_frd) {
        a = std::clamp(a, -kAccelClamp, kAccelClamp);
    }
    return out;
}

double World::now() const { return static_cast<double>(impl_->tick) * impl_->dt_s; }
uint64_t World::tick_index() const { return impl_->tick; }

} // namespace skysim::core
