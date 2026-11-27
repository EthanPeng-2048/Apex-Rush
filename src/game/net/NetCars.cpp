#include "game/net/NetCars.h"

#include "net/protocol/wire_quantize.h"

#include <algorithm>
#include <cmath>

namespace apex {
namespace {

float Mix(float a, float b, float t) { return a + (b - a) * t; }

float MixAngle(float a, float b, float period, float t) {
    float d = std::fmod(b - a, period);
    if (d > period * 0.5f) d -= period;
    if (d < -period * 0.5f) d += period;
    return a + d * t;
}

Transform TransformFromWire(const net::CarNetState& c) {
    Transform t;
    t.position = Vec3{c.pos[0], c.pos[1], c.pos[2]};
    float qx = 0.f, qy = 0.f, qz = 0.f, qw = 1.f;
    net::q::YprToQuat(c.yaw, c.pitch, c.roll, qx, qy, qz, qw);
    t.rotation = Quat{qx, qy, qz, qw};
    return t;
}

}  // namespace

void ToCarNetState(std::uint8_t slot, const Transform& chassis, const VehicleState& state,
                   std::uint8_t lap, std::uint8_t position, bool finished, bool retired,
                   net::CarNetState& out) {
    out = net::CarNetState{};
    out.slot = slot;
    out.pos[0] = chassis.position.x;
    out.pos[1] = chassis.position.y;
    out.pos[2] = chassis.position.z;

    float yaw = 0.f, pitch = 0.f, roll = 0.f;
    net::q::QuatToYpr(chassis.rotation.x, chassis.rotation.y, chassis.rotation.z, chassis.rotation.w,
                      yaw, pitch, roll);
    out.yaw = net::q::WrapYaw(yaw);
    out.pitch = pitch;
    out.roll = roll;

    out.vel[0] = state.velocity.x;
    out.vel[1] = state.velocity.y;
    out.vel[2] = state.velocity.z;
    out.yawRate = state.angularVelocity.y;
    out.pitchRate = state.angularVelocity.x;
    out.rollRate = state.angularVelocity.z;
    out.steerDeg = state.steerAngleDeg;
    out.speedKmh = state.speedKmh;
    out.rpm = state.rpm;
    out.gear = static_cast<std::uint8_t>(std::clamp(state.gear, 0, 15));
    out.wingMode = static_cast<std::uint8_t>(state.wingMode);
    out.drsActive = state.drsActive;
    out.ersDeploying = state.ersDeploying;
    out.drsAvailable = state.drsAvailable;
    out.ersCharge = state.ersCharge;
    out.finished = finished;
    out.retired = retired;
    out.lap = lap;
    out.position = position;
    for (int i = 0; i < 4; ++i) {
        const auto& w = state.wheels[static_cast<std::size_t>(i)];
        out.wheelTravel[static_cast<std::size_t>(i)] =
            w.suspensionLength > 0.f ? std::clamp(1.f - w.compression, 0.f, 1.f) : 0.f;
        if (w.grounded) out.groundedMask |= static_cast<std::uint8_t>(1u << i);
        const bool onTrack = w.grounded != 0;
        out.onTrack = out.onTrack && onTrack;
    }
}

void ToSelfState(const VehicleSnapshotState& snap, net::SelfState& out) {
    for (int i = 0; i < 4; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        const auto& w = snap.state.wheels[idx];
        out.slipRatio[idx] = w.slipRatio;
        out.slipAngleDeg[idx] = w.slipAngleDeg;
        out.wheelSpinRate[idx] = snap.wheelOmega[idx];
        out.wear[idx] = w.wear;
        out.tempC[idx] = w.temperature;
        out.partHealth[idx] = 1.f;
    }
    out.damageTotal = 0.f;
    out.shiftCooldownTicks = 0;
    out.ersDeployTicks = 0;
    out.rngState = 0;
}

void FromCarNetState(const net::CarNetState& in, Transform& chassis, VehicleState& state) {
    chassis = TransformFromWire(in);
    state.transform = chassis;
    state.velocity = Vec3{in.vel[0], in.vel[1], in.vel[2]};
    state.angularVelocity = Vec3{in.pitchRate, in.yawRate, in.rollRate};
    state.speedKmh = in.speedKmh;
    state.rpm = in.rpm;
    state.gear = in.gear;
    state.steerAngleDeg = in.steerDeg;
    state.ersCharge = in.ersCharge;
    state.ersDeploying = in.ersDeploying;
    state.drsActive = in.drsActive;
    state.drsAvailable = in.drsAvailable;
    state.wingMode = static_cast<WingMode>(in.wingMode);
    const Vec3 fwd = TransformDirection(chassis, Vec3{0.f, 0.f, 1.f});
    state.forwardSpeed = state.velocity.x * fwd.x + state.velocity.y * fwd.y + state.velocity.z * fwd.z;
    for (int i = 0; i < 4; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        state.wheels[idx].grounded = (in.groundedMask & (1u << i)) != 0;
        state.wheels[idx].compression = 1.f - in.wheelTravel[idx];
    }
}

void ApplyWireToSnapshot(const net::CarNetState& car, const net::SelfState* self,
                         VehicleSnapshotState& snapshot) {
    Transform chassis;
    VehicleState state = snapshot.state;   // keep the fields the wire does not carry
    FromCarNetState(car, chassis, state);
    state.wheels = snapshot.state.wheels;   // wheels come from the self block, below
    if (self) {
        for (int i = 0; i < 4; ++i) {
            const auto idx = static_cast<std::size_t>(i);
            auto& w = state.wheels[idx];
            w.wear = self->wear[idx];
            w.temperature = self->tempC[idx];
            w.slipRatio = self->slipRatio[idx];
            w.slipAngleDeg = self->slipAngleDeg[idx];
            w.grounded = (car.groundedMask & (1u << i)) != 0;
            w.compression = 1.f - car.wheelTravel[idx];
            snapshot.wheelOmega[idx] = self->wheelSpinRate[idx];
            snapshot.wheelAlpha[idx] = self->slipAngleDeg[idx] * net::q::kDegToRad;
        }
    }
    snapshot.state = state;
    snapshot.transform = chassis;
    snapshot.linearVelocity = state.velocity;
    snapshot.angularVelocity = state.angularVelocity;
}

// ------------------------------------------------------------- interpolator ---

void RemoteInterpolator::Clear() { buffer_.clear(); }

void RemoteInterpolator::Push(const net::SnapshotPacket& packet, std::uint64_t arrivalMs) {
    Stored stored;
    stored.arrivalMs = arrivalMs;
    stored.hostTick = packet.hostTick;
    stored.carCount = packet.carCount;
    for (int i = 0; i < net::kMaxCars; ++i) stored.cars[static_cast<std::size_t>(i)] = packet.cars[i];
    buffer_.push_back(stored);
    if (buffer_.size() > 40) buffer_.erase(buffer_.begin(), buffer_.begin() + 8);
}

bool RemoteInterpolator::HasData(std::uint8_t slot) const {
    for (const Stored& s : buffer_) {
        for (int i = 0; i < s.carCount; ++i) {
            if (s.cars[static_cast<std::size_t>(i)].slot == slot) return true;
        }
    }
    return false;
}

std::uint16_t RemoteInterpolator::LastAppliedInputSeq() const {
    return buffer_.empty() ? 0 : 0;   // per-recipient value lives in the packet header
}

bool RemoteInterpolator::Sample(std::uint8_t slot, double renderTimeMs, Transform& chassis,
                                VehicleState& state) const {
    if (buffer_.size() < 2) return false;

    const double target = renderTimeMs;

    // Find the pair bracketing `target` on the arrival timeline.
    const Stored* older = nullptr;
    const Stored* newer = nullptr;
    for (std::size_t i = 0; i + 1 < buffer_.size(); ++i) {
        if (static_cast<double>(buffer_[i].arrivalMs) <= target &&
            static_cast<double>(buffer_[i + 1].arrivalMs) >= target) {
            older = &buffer_[i];
            newer = &buffer_[i + 1];
            break;
        }
    }
    if (!older) {
        older = &buffer_[buffer_.size() - 2];
        newer = &buffer_.back();
    }

    const auto findCar = [&](const Stored& s) -> const net::CarNetState* {
        for (int i = 0; i < s.carCount; ++i) {
            if (s.cars[static_cast<std::size_t>(i)].slot == slot) {
                return &s.cars[static_cast<std::size_t>(i)];
            }
        }
        return nullptr;
    };
    const net::CarNetState* a = findCar(*older);
    const net::CarNetState* b = findCar(*newer);
    if (!a) return false;
    if (!b) b = a;

    const double span = static_cast<double>(newer->arrivalMs) - static_cast<double>(older->arrivalMs);
    float t = span > 1.0 ? static_cast<float>((target - static_cast<double>(older->arrivalMs)) / span)
                         : 1.f;
    t = std::clamp(t, 0.f, 1.f);

    Transform ta = TransformFromWire(*a);
    Transform tb = TransformFromWire(*b);
    chassis.position = Vec3{Mix(ta.position.x, tb.position.x, t),
                            Mix(ta.position.y, tb.position.y, t),
                            Mix(ta.position.z, tb.position.z, t)};
    float qa[4] = {ta.rotation.x, ta.rotation.y, ta.rotation.z, ta.rotation.w};
    float qb[4] = {tb.rotation.x, tb.rotation.y, tb.rotation.z, tb.rotation.w};
    // Shortest-arc slerp via lerp + renormalise is plenty for interpolation
    // between snapshots 16-50 ms apart.
    if (qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3] < 0.f) {
        for (float& v : qb) v = -v;
    }
    float q[4];
    for (int i = 0; i < 4; ++i) q[i] = Mix(qa[i], qb[i], t);
    const float len = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    chassis.rotation = len > 1e-6f ? Quat{q[0] / len, q[1] / len, q[2] / len, q[3] / len}
                                   : ta.rotation;

    FromCarNetState(*b, chassis, state);   // discrete fields from the newer sample
    state.transform = chassis;
    state.velocity = Vec3{Mix(a->vel[0], b->vel[0], t), Mix(a->vel[1], b->vel[1], t),
                          Mix(a->vel[2], b->vel[2], t)};
    state.angularVelocity = Vec3{Mix(a->pitchRate, b->pitchRate, t),
                                 Mix(a->yawRate, b->yawRate, t),
                                 Mix(a->rollRate, b->rollRate, t)};
    state.speedKmh = Mix(a->speedKmh, b->speedKmh, t);
    state.rpm = Mix(a->rpm, b->rpm, t);
    state.steerAngleDeg = Mix(a->steerDeg, b->steerDeg, t);
    state.ersCharge = Mix(a->ersCharge, b->ersCharge, t);
    for (int i = 0; i < 4; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        state.wheels[idx].compression =
            Mix(1.f - a->wheelTravel[idx], 1.f - b->wheelTravel[idx], t);
        state.wheels[idx].grounded = (b->groundedMask & (1u << i)) != 0;
    }
    return true;
}

// ---------------------------------------------------------- local predictor ---

void LocalPredictor::Reset(std::uint32_t tick) {
    headTick_ = tick;
    for (auto& h : history_) h = HistoryEntry{};
    std::lock_guard lock(mutex_);
    hasAuthority_ = false;
    lastReconciledSeq_ = 0;
}

void LocalPredictor::PushAuthority(const net::CarNetState& car, const net::SelfState* self,
                                   std::uint16_t appliedInputSeq, std::uint32_t hostTick) {
    std::lock_guard lock(mutex_);
    authorityCar_ = car;
    authorityHasSelf_ = self != nullptr;
    if (self) authoritySelf_ = *self;
    authoritySeq_ = appliedInputSeq;
    authorityTick_ = hostTick;
    hasAuthority_ = true;
}

void LocalPredictor::Record(std::uint32_t tick, std::uint16_t seq, const CarInput& input) {
    HistoryEntry& e = history_[tick % kHistory];
    e.tick = tick;
    e.seq = seq;
    e.hasInput = true;
    e.input = input;
    headTick_ = tick;
}

bool LocalPredictor::Reconcile(Vehicle& vehicle, IPhysicsWorld& world, float dt,
                               std::uint32_t currentTick) {
    net::CarNetState car;
    net::SelfState self;
    bool hasSelf = false;
    std::uint16_t seq = 0;
    {
        std::lock_guard lock(mutex_);
        if (!hasAuthority_) return false;
        hasAuthority_ = false;
        if (!net::SeqNewer(authoritySeq_, lastReconciledSeq_) && lastReconciledSeq_ != 0) {
            return false;
        }
        car = authorityCar_;
        self = authoritySelf_;
        hasSelf = authorityHasSelf_;
        seq = authoritySeq_;
        lastReconciledSeq_ = seq;
    }

    // Where did we apply that input? Search newest -> oldest.
    std::uint32_t replayFrom = currentTick;
    bool found = false;
    for (std::uint32_t back = 0; back < kHistory && back <= headTick_; ++back) {
        if (headTick_ < back) break;
        const std::uint32_t tick = headTick_ - back;
        const HistoryEntry& e = history_[tick % kHistory];
        if (!e.hasInput || e.tick != tick) continue;
        if (e.seq == seq) {
            replayFrom = tick;
            found = true;
            break;
        }
    }

    VehicleSnapshotState snapshot = vehicle.SaveState();
    ApplyWireToSnapshot(car, hasSelf ? &self : nullptr, snapshot);
    vehicle.RestoreState(snapshot);
    if (!found) return true;

    if (currentTick - replayFrom > kMaxReplay) return true;   // too far: keep the snap

    for (std::uint32_t tick = replayFrom + 1; tick <= currentTick; ++tick) {
        const HistoryEntry& e = history_[tick % kHistory];
        if (!e.hasInput || e.tick != tick) break;
        vehicle.Update(e.input, dt);
        world.Step(dt);
    }
    return true;
}

// ------------------------------------------------------------------ proxies ---

void RemoteProxies::Build(IPhysicsWorld& world, int carCount, std::uint8_t localSlot,
                          const VehicleParams& params) {
    (void)carCount;
    (void)localSlot;
    for (int slot = 0; slot < net::kMaxCars; ++slot) {
        Proxy& p = proxies_[static_cast<std::size_t>(slot)];
        if (p.active) continue;
        BodyDesc desc;
        desc.halfExtents = Vec3{params.chassisHalfX, params.chassisHalfY, params.chassisHalfZ};
        desc.mass = 0.f;   // static: moved by SetTransform, never pushed
        desc.friction = 0.4f;
        desc.restitution = 0.05f;
        desc.allowSleep = false;
        const auto body = world.CreateBody(desc);
        if (!body) continue;
        p.body = *body;
        p.active = true;
        p.placed = false;
    }
}

void RemoteProxies::Dispose(IPhysicsWorld& world) {
    for (Proxy& p : proxies_) {
        if (p.active) (void)world.DestroyBody(p.body);
        p = Proxy{};
    }
}

void RemoteProxies::Update(IPhysicsWorld& world, const RemoteInterpolator& interp,
                           double renderTimeMs, std::uint8_t carCount) {
    for (int slot = 0; slot < net::kMaxCars; ++slot) {
        Proxy& p = proxies_[static_cast<std::size_t>(slot)];
        if (!p.active) continue;
        Transform chassis;
        VehicleState state;
        if (!enabled_ || slot >= carCount || !interp.Sample(static_cast<std::uint8_t>(slot),
                                                           renderTimeMs, chassis, state)) {
            if (p.placed) {
                // Park it far below the map so it cannot block anyone.
                Transform away;
                away.position = Vec3{0.f, -5000.f, 0.f};
                (void)world.SetTransform(p.body, away);
                p.placed = false;
            }
            continue;
        }
        (void)world.SetTransform(p.body, chassis);
        p.placed = true;
    }
}

}  // namespace apex
