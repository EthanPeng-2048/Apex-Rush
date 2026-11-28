#pragma once

// Game-side networking glue: wire <-> engine conversions, the local car's
// rollback prediction, and remote car interpolation with collision proxies.
// See docs/online/architecture.md §4-§5.

#include <algorithm>
#include <array>
#include <cstdint>
#include <mutex>
#include <vector>

#include "game/Track.h"
import rush.engine.core;
import rush.engine.physics;
import rush.engine.render;

#include "net/protocol/wire.h"
#include "net/session.h"

namespace apex {

// --- wire <-> engine ---------------------------------------------------------

void ToCarNetState(std::uint8_t slot, const Transform& chassis, const VehicleState& state,
                   std::uint8_t lap, std::uint8_t position, bool finished, bool retired,
                   net::CarNetState& out);

void ToSelfState(const VehicleSnapshotState& snap, net::SelfState& out);

// Rebuilds transform + VehicleState from the compact wire form.
void FromCarNetState(const net::CarNetState& in, Transform& chassis, VehicleState& state);

// Writes the authority view into `snapshot` (which should start from
// Vehicle::SaveState()): pose, momentum, wheels, gearbox/engine readout and the
// tire integrators the network carries.
void ApplyWireToSnapshot(const net::CarNetState& car, const net::SelfState* self,
                         VehicleSnapshotState& snapshot);

// --- remote interpolation ----------------------------------------------------

// Snapshot playout buffer for other players' cars (N-05): render slightly in
// the past and interpolate between the two snapshots that bracket that time.
class RemoteInterpolator {
public:
    void Clear();
    void Push(const net::SnapshotPacket& packet, std::uint64_t arrivalMs);
    void SetDelayMs(double delayMs) { delayMs_ = delayMs; }
    double DelayMs() const { return delayMs_; }

    // Fills `state`/`chassis` for `slot` at `renderTimeMs` (local ms). Returns
    // false when there is no data for that slot yet.
    bool Sample(std::uint8_t slot, double renderTimeMs, Transform& chassis,
                VehicleState& state) const;

    bool HasData(std::uint8_t slot) const;

private:
    struct Stored {
        std::uint64_t arrivalMs = 0;
        std::uint32_t hostTick = 0;
        std::uint8_t carCount = 0;
        std::array<net::CarNetState, net::kMaxCars> cars{};
    };
    std::vector<Stored> buffer_;
    double delayMs_ = 40.0;
};

// --- correction smoothing ----------------------------------------------------

// Pose difference a rollback produced: the simulated pose moved by this much
// when the authority took over. `position` is (before - after), `rotation` is
// (before * inverse(after)) — apply them on top of the corrected pose to keep
// the drawn car where the player already saw it. `replayed` says whether the
// unconfirmed inputs could be re-simulated (false = input gap, hard snap).
struct ReconcileCorrection {
    bool applied = false;
    bool replayed = false;
    Vec3 position{};
    Quat rotation = Quat::Identity();
};

// Absorbs corrections into a decaying render offset instead of snapping the
// drawn car (docs/online/architecture.md §4.4): tau = time constant, and
// anything past the snap thresholds is accepted instantly — a 2 m jump is a
// teleport, not something to animate.
class CorrectionSmoother {
public:
    void Reset();
    void SetTauMs(float ms) { tauSec_ = std::clamp(ms, 0.f, 1000.f) * 0.001f; }
    float TauSec() const { return tauSec_; }

    // Sim thread, right after a rollback landed. Accumulates so back-to-back
    // corrections do not cancel the smoothing of the earlier one.
    void Push(const ReconcileCorrection& correction);

    // Sim thread, once per tick: exponential decay toward zero.
    void Decay(float dt);

    // Adds the remaining offset to a drawn/published pose.
    void Apply(Transform& pose) const;
    bool Active() const;

private:
    Vec3 position_{};
    // Rotation offset kept as axis + angle: decaying an angle is a scalar
    // multiply, while decay through quaternions would have to rebuild it every
    // tick and could never reach zero (acos near w = 1 is float-quantised).
    Vec3 rotAxis_{0.f, 1.f, 0.f};
    float rotAngleRad_ = 0.f;
    float tauSec_ = 0.08f;
};

// --- local prediction --------------------------------------------------------

// Own-car prediction: apply input immediately, then roll back to the
// authoritative state and replay the unconfirmed inputs when a snapshot lands.
class LocalPredictor {
public:
    struct HistoryEntry {
        std::uint32_t tick = 0;
        std::uint16_t seq = 0;
        bool hasInput = false;
        CarInput input{};
    };

    void Reset(std::uint32_t tick);
    void PushAuthority(const net::CarNetState& car, const net::SelfState* self,
                       std::uint16_t appliedInputSeq, std::uint32_t hostTick);

    // Sim thread: consume a pending authority view (if any) by restoring and
    // replaying. `lastSimulatedTick` is the newest tick that has already run —
    // the replay covers everything after the authoritative input up to and
    // including it, and the tick being started now is simulated normally after.
    // Returns true when a correction was applied; `correction` then describes the
    // pose delta for the render layer.
    bool Reconcile(Vehicle& vehicle, IPhysicsWorld& world, float dt,
                   std::uint32_t lastSimulatedTick, ReconcileCorrection* correction);

    void Record(std::uint32_t tick, std::uint16_t seq, const CarInput& input);

private:
    static constexpr std::uint32_t kHistory = 32;
    static constexpr std::uint32_t kMaxReplay = 16;

    std::array<HistoryEntry, kHistory> history_{};
    std::uint32_t headTick_ = 0;

    std::mutex mutex_;
    bool hasAuthority_ = false;
    bool anyReconciled_ = false;
    net::CarNetState authorityCar_{};
    net::SelfState authoritySelf_{};
    bool authorityHasSelf_ = false;
    std::uint16_t authoritySeq_ = 0;
    std::uint32_t authorityTick_ = 0;
    std::uint16_t lastReconciledSeq_ = 0;
};

// --- remote collision proxies ------------------------------------------------

// Other players are not simulated locally, but they must be solid: a mass-0 box
// tracked to the interpolated pose makes them block/deflect the local car
// without ever becoming a second author of their own state. Seats are dense
// (see net/roster.h), so `carCount` + `localSlot` describe the set: every seat
// gets a proxy except our own, which is a real simulated car.
class RemoteProxies {
public:
    void Build(IPhysicsWorld& world, int carCount, std::uint8_t localSlot,
               const VehicleParams& params);
    void Dispose(IPhysicsWorld& world);
    void Update(IPhysicsWorld& world, const RemoteInterpolator& interp, double renderTimeMs,
                std::uint8_t carCount);
    void SetEnabled(bool on) { enabled_ = on; }

private:
    struct Proxy {
        bool active = false;
        BodyHandle body = kInvalidBody;
        bool placed = false;
    };
    std::array<Proxy, net::kMaxCars> proxies_{};
    std::uint8_t localSlot_ = 0;
    bool enabled_ = true;
};

}  // namespace apex
