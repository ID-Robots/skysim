// Jolt world implementation. Everything below the World interface is Jolt Y-up; every value
// crossing the interface is NED/FRD via core/frames.h. CLAUDE.md "Jolt-specific rules" apply.
#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/StreamWrapper.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <random>
#include <thread>
#include <unordered_map>
#include <unordered_set>
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
    Vec3 prev_vel_ned{};            // for specific-force finite difference
    uint64_t midair_collisions = 0; // vehicle-vehicle contact manifolds
    uint64_t static_contacts = 0;   // ground/tile contact manifolds
    uint64_t building_contacts = 0; // streamed-tile contact manifolds only
};

// CLAUDE.md "Jolt-specific rules": contact callbacks run on Jolt's job threads during
// Update and must only enqueue, lock-free. Fixed slots + an atomic cursor; overflow events
// are dropped (counted) rather than blocking the solver.
class EnqueueOnlyContactListener final : public JPH::ContactListener {
  public:
    void OnContactAdded(const JPH::Body &body1, const JPH::Body &body2, const JPH::ContactManifold &,
                        JPH::ContactSettings &) override {
        const size_t slot = count_.fetch_add(1, std::memory_order_relaxed);
        if (slot < events_.size()) {
            events_[slot] = {body1.GetID(), body2.GetID()};
        }
    }

    // Drain on the tick thread after PhysicsSystem::Update returns (no callbacks running).
    template <typename Fn> void drain(Fn &&fn) {
        const size_t n = std::min(count_.exchange(0, std::memory_order_relaxed), events_.size());
        for (size_t i = 0; i < n; ++i) {
            fn(events_[i].first, events_[i].second);
        }
    }

  private:
    std::array<std::pair<JPH::BodyID, JPH::BodyID>, 256> events_{};
    std::atomic<size_t> count_{0};
};

} // namespace

struct World::Impl {
    BpLayerInterface bp_interface;
    ObjectVsBpFilter obj_vs_bp;
    ObjectPairFilter obj_pair;
    EnqueueOnlyContactListener contacts;
    std::unique_ptr<JPH::TempAllocatorImpl> temp_allocator;
    std::unique_ptr<JPH::JobSystem> job_system;
    std::unique_ptr<JPH::PhysicsSystem> physics;

    double dt_s = 1.0 / 400.0;
    uint64_t tick = 0;
    uint32_t next_id = 1;
    std::unordered_map<uint32_t, VehicleEntry> vehicles;
    std::vector<JPH::BodyID> static_bodies;
    std::unordered_map<uint32_t, JPH::BodyID> static_tiles; // streamed (M5)
    // Body indices of resident tiles, so contact attribution can distinguish a
    // building strike from an ordinary ground touch without a linear scan.
    std::unordered_set<uint32_t> tile_body_indices;
    // BodyID (index+sequence) -> vehicle id, so contact-event attribution is O(1) per contact
    // instead of an O(vehicles) linear scan (matters when many contacts fire in one step).
    std::unordered_map<uint32_t, uint32_t> body_to_vehicle;

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
    // Physics threading. Benchmarking (tools/bench/world_bench) shows that for the fleet sizes
    // one skysim world holds (tens to a few hundred quads), a multi-thread job system is a net
    // LOSS: dispatch + barrier + thread-wakeup jitter dwarfs the tiny per-body work and inflates
    // tick p99 several-fold. The crossover where worker threads win is ~1000 bodies. skysim's
    // scaling model is many single-world processes (DESIGN.md sharding), so the right default is
    // a single-threaded job system (calling thread does all jobs, zero sync). worker_threads:
    //   0 or -1 => single-threaded (default), N>0 => JobSystemThreadPool with N worker threads.
    if (cfg.worker_threads <= 0) {
        impl_->job_system = std::make_unique<JPH::JobSystemSingleThreaded>(JPH::cMaxPhysicsJobs);
    } else {
        impl_->job_system = std::make_unique<JPH::JobSystemThreadPool>(
            JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, cfg.worker_threads);
    }

    impl_->physics = std::make_unique<JPH::PhysicsSystem>();
    impl_->physics->Init(cfg.max_bodies, 0, cfg.max_bodies * 2, cfg.max_bodies * 2, impl_->bp_interface,
                         impl_->obj_vs_bp, impl_->obj_pair);
    // NED gravity (0,0,+g) is Jolt (0,-g,0) — Jolt's default, but set it explicitly.
    impl_->physics->SetGravity(JPH::Vec3(0.0f, -static_cast<float>(kGravity), 0.0f));
    impl_->physics->SetContactListener(&impl_->contacts);

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

namespace {
// Tiles are cooked in Jolt frame with world-absolute vertices; place at identity.
JPH::BodyID create_tile_body(JPH::BodyInterface &bodies, const std::filesystem::path &jshape) {
    std::ifstream in(jshape, std::ios::binary);
    if (!in) {
        std::fprintf(stderr, "world: cannot open tile %s\n", jshape.c_str());
        return {};
    }
    JPH::StreamInWrapper stream(in);
    JPH::Shape::ShapeResult result = JPH::Shape::sRestoreFromBinaryState(stream);
    if (result.HasError()) {
        std::fprintf(stderr, "world: tile %s: %s\n", jshape.c_str(), result.GetError().c_str());
        return {};
    }
    JPH::BodyCreationSettings s(result.Get(), JPH::RVec3::sZero(), JPH::Quat::sIdentity(),
                                JPH::EMotionType::Static, object_layers::kStatic);
    s.mFriction = 0.8f;
    s.mRestitution = 0.0f;
    return bodies.CreateAndAddBody(s, JPH::EActivation::DontActivate);
}
} // namespace

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
        const JPH::BodyID body = create_tile_body(impl_->bodies(), entry.path());
        if (!body.IsInvalid()) {
            impl_->static_bodies.push_back(body);
            // Register here as well as in add_static_tile(): bulk-loaded tiles are just
            // as much "buildings" as streamed ones, and contact attribution must treat
            // them the same or a strike on a preloaded tile looks like a ground touch.
            impl_->tile_body_indices.insert(body.GetIndexAndSequenceNumber());
            ++loaded;
        }
    }
    return loaded;
}

uint32_t World::add_static_tile(const std::filesystem::path &jshape) {
    const JPH::BodyID body = create_tile_body(impl_->bodies(), jshape);
    if (body.IsInvalid()) {
        return 0;
    }
    const uint32_t id = impl_->next_id++;
    impl_->static_tiles.emplace(id, body);
    impl_->tile_body_indices.insert(body.GetIndexAndSequenceNumber());
    return id;
}

void World::remove_static_tile(uint32_t id) {
    auto it = impl_->static_tiles.find(id);
    if (it == impl_->static_tiles.end()) {
        return;
    }
    impl_->tile_body_indices.erase(it->second.GetIndexAndSequenceNumber());
    impl_->bodies().RemoveBody(it->second);
    impl_->bodies().DestroyBody(it->second);
    impl_->static_tiles.erase(it);
}

void World::optimize_broadphase() { impl_->physics->OptimizeBroadPhase(); }

double World::raycast(const Vec3 &origin_ned, const Vec3 &dir_ned, double max_dist_m, uint32_t ignore_vehicle_id,
                      bool static_only) const {
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
    const JPH::SpecifiedBroadPhaseLayerFilter static_bp(bp_layers::kStatic);
    const JPH::SpecifiedObjectLayerFilter static_objects(object_layers::kStatic);
    const JPH::BroadPhaseLayerFilter any_bp;
    const JPH::ObjectLayerFilter any_object;
    const JPH::BroadPhaseLayerFilter &bp_filter = static_only ? static_bp : any_bp;
    const JPH::ObjectLayerFilter &object_filter = static_only ? static_objects : any_object;
    if (impl_->physics->GetNarrowPhaseQuery().CastRay(ray, hit, bp_filter, object_filter, body_filter)) {
        return hit.mFraction * max_dist_m;
    }
    return -1.0;
}

std::vector<World::PathHit> World::sweep_path(const std::vector<Vec3> &waypoints_ned, double clearance_m) const {
    std::vector<PathHit> hits;
    if (waypoints_ned.size() < 2) {
        return hits;
    }

    // Offsets perpendicular to each leg, so the sweep approximates the airframe's width
    // rather than an infinitely thin line through a doorway.
    const double r = std::max(0.0, clearance_m);

    for (size_t leg = 0; leg + 1 < waypoints_ned.size(); ++leg) {
        const Vec3 &a = waypoints_ned[leg];
        const Vec3 &b = waypoints_ned[leg + 1];

        const Vec3 delta{b[0] - a[0], b[1] - a[1], b[2] - a[2]};
        const double length = std::sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
        if (length < 1e-6) {
            continue; // duplicate waypoint
        }
        const Vec3 dir{delta[0] / length, delta[1] / length, delta[2] / length};

        // Any vector not parallel to dir works as a seed for the perpendicular basis.
        const Vec3 seed = (std::fabs(dir[2]) < 0.9) ? Vec3{0.0, 0.0, 1.0} : Vec3{1.0, 0.0, 0.0};
        Vec3 u{dir[1] * seed[2] - dir[2] * seed[1], dir[2] * seed[0] - dir[0] * seed[2],
               dir[0] * seed[1] - dir[1] * seed[0]};
        const double u_len = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
        if (u_len > 1e-9) {
            u = Vec3{u[0] / u_len, u[1] / u_len, u[2] / u_len};
        }
        const Vec3 v{dir[1] * u[2] - dir[2] * u[1], dir[2] * u[0] - dir[0] * u[2], dir[0] * u[1] - dir[1] * u[0]};

        const Vec3 offsets[5] = {
            {0.0, 0.0, 0.0}, {u[0] * r, u[1] * r, u[2] * r},    {-u[0] * r, -u[1] * r, -u[2] * r},
            {v[0] * r, v[1] * r, v[2] * r}, {-v[0] * r, -v[1] * r, -v[2] * r},
        };

        double nearest = -1.0;
        for (const Vec3 &offset : offsets) {
            const Vec3 origin{a[0] + offset[0], a[1] + offset[1], a[2] + offset[2]};
            const double distance = raycast(origin, dir, length, 0, /*static_only=*/true);
            if (distance >= 0.0 && (nearest < 0.0 || distance < nearest)) {
                nearest = distance;
            }
        }

        if (nearest >= 0.0) {
            hits.push_back(PathHit{leg,
                                   Vec3{a[0] + dir[0] * nearest, a[1] + dir[1] * nearest, a[2] + dir[2] * nearest},
                                   nearest});
        }
    }

    return hits;
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
    impl_->body_to_vehicle.emplace(e.body.GetIndexAndSequenceNumber(), id);
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
    impl_->body_to_vehicle.erase(it->second.body.GetIndexAndSequenceNumber());
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

    // Attribute this step's new contact manifolds to vehicles (tick thread, callbacks done).
    // O(1) per contact via the body->vehicle map.
    impl_->contacts.drain([this](JPH::BodyID a, JPH::BodyID b) {
        auto ia = impl_->body_to_vehicle.find(a.GetIndexAndSequenceNumber());
        auto ib = impl_->body_to_vehicle.find(b.GetIndexAndSequenceNumber());
        const auto end = impl_->body_to_vehicle.end();
        VehicleEntry *va = ia != end ? &impl_->vehicles.at(ia->second) : nullptr;
        VehicleEntry *vb = ib != end ? &impl_->vehicles.at(ib->second) : nullptr;
        if (va != nullptr && vb != nullptr) {
            ++va->midair_collisions;
            ++vb->midair_collisions;
        } else if (va != nullptr) {
            ++va->static_contacts;
            if (impl_->tile_body_indices.count(b.GetIndexAndSequenceNumber()) != 0) {
                ++va->building_contacts;
            }
        } else if (vb != nullptr) {
            ++vb->static_contacts;
            if (impl_->tile_body_indices.count(a.GetIndexAndSequenceNumber()) != 0) {
                ++vb->building_contacts;
            }
        }
    });
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
    out.midair_collisions = v.midair_collisions;
    out.static_contacts = v.static_contacts;
    out.building_contacts = v.building_contacts;
    return out;
}

double World::now() const { return static_cast<double>(impl_->tick) * impl_->dt_s; }
uint64_t World::tick_index() const { return impl_->tick; }

} // namespace skysim::core
