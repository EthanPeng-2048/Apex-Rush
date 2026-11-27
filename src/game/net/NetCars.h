#pragma once

// Game-side networking glue: wire <-> engine conversions, the local car's
// rollback prediction, and remote car interpolation with collision proxies.
// See docs/online/architecture.md §4-§5.

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
    std::uint16_t LastAppliedInputSeq() const;

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
    // replaying. Returns true when a correction was applied.
    bool Reconcile(Vehicle& vehicle, IPhysicsWorld& world, float dt,
                   std::uint32_t currentTick);

    void Record(std::uint32_t tick, std::uint16_t seq, const CarInput& input);

private:
    static constexpr std::uint32_t kHistory = 32;
    static constexpr std::uint32_t kMaxReplay = 16;

    std::array<HistoryEntry, kHistory> history_{};
    std::uint32_t headTick_ = 0;

    std::mutex mutex_;
    bool hasAuthority_ = false;
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
