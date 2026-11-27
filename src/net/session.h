#pragma once

// Session layer: the host and client state machines. Everything here speaks the
// wire structs from wire.h; nothing here knows about Vehicle or Game.

#include <array>
#include <cstdint>
#include <expected>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "net/discovery.h"
#include "net/protocol/wire.h"
#include "net/transport.h"

namespace apex::net {

enum class SessionState : std::uint8_t {
    Idle = 0,
    Advertising,   // host: room open, waiting for players
    Searching,     // client: not connected yet
    Joining,       // client: connect + hello handshake
    Lobby,
    Racing,
    Results,
    Closed,
};

enum class SessionEventKind : std::uint8_t {
    None = 0,
    PeerJoined,
    PeerLeft,
    JoinRejected,
    RulesReceived,
    LobbyChanged,
    RaceStarted,
    LapComplete,
    PlayerFinished,
    RaceEnded,
    CarRecovered,
    PartDamaged,
    Restart,
    HostGone,
    SessionEnded,
};

// Flat event payload: events are rare, so a fat struct beats a variant here.
struct SessionEvent {
    SessionEventKind kind = SessionEventKind::None;
    PeerId peer{};
    std::uint8_t slot = 0;
    std::uint8_t value = 0;      // position / reason / part / lap
    std::uint32_t ms = 0;        // lap time / race time
    std::uint32_t bestMs = 0;
    std::uint32_t hostTick = 0;
    std::uint16_t countdownCs = 0;
    std::string text;                            // player name (PeerJoined)
    RejectReason reason = RejectReason::Internal;
    RulesPacket rules{};
    std::vector<LobbyPlayer> lobby;
    std::vector<RaceEndEntry> results;
};

// One sampled local input intent (no world state).
struct InputSample {
    float throttle = 0.f;
    float brake = 0.f;
    float steer = 0.f;
    bool handbrake = false;
    bool reverse = false;
    std::uint8_t drsPress = 0;       // monotonic press counters
    std::uint8_t wingPress = 0;
    std::uint8_t overtakePress = 0;
};

struct NetConfig {
    std::string playerName = "PLAYER";
    // Identity hashed from the name (net/identity.h). Sent in Hello; the host
    // refuses a second player claiming an id it already seated.
    PlayerId playerId = kInvalidPlayerId;
    std::uint16_t roomCode = 0;
    std::uint16_t gamePort = kGamePortDefault;
    std::uint16_t discoveryPort = kDiscoveryPortDefault;
    int maxPlayers = kMaxCars;
    int snapshotHz = 30;
    int inputHz = 60;
    std::uint32_t buildSig = 0;
    std::uint32_t mapHash = 0;
    std::string mapName;
    RulesPacket rules{};
};

class INetSession {
public:
    virtual ~INetSession() = default;

    // Main thread, once per frame.
    virtual void Tick(std::uint64_t nowMs) = 0;

    // Main thread, after Tick: drains the event queue.
    virtual std::span<const SessionEvent> DrainEvents() = 0;

    virtual SessionState State() const = 0;
    virtual bool IsHost() const = 0;
    // Seat of the local player. Before a race it is the arrival slot (dense,
    // may still change); FreezeRoster/serve renumbers it to the seat, and the
    // seat is what the grid, the snapshot `slot` field and input routing use
    // for the rest of the race.
    virtual std::uint8_t LocalSlot() const = 0;

    // --- host side ---------------------------------------------------------
    // Authoritative world published at the snapshot rate. `self` carries each
    // slot's full integrator state so the host can attach the right one to each
    // recipient; header fields (tick/seq/applied) are filled by the session.
    struct HostWorld {
        SnapshotPacket packet{};
        std::array<SelfState, kMaxCars> self{};
        std::array<std::uint8_t, kMaxCars> selfValid{};
    };
    virtual void PublishSnapshot(const HostWorld& world) { (void)world; }
    // Sim thread: record which input sequence has actually been simulated.
    virtual void SetAppliedInputSeq(std::uint8_t slot, std::uint16_t seq) {
        (void)slot;
        (void)seq;
    }
    // Host: update the rule set that gets advertised in Welcome/RaceStart.
    virtual void SetRules(const RulesPacket& rules) { (void)rules; }
    // Host: seat every joined player (rank of playerId, see net/roster.h) and
    // fold the frozen roster into the rules. Called once, before StartRace, so
    // the grid both ends build is the same grid. Renumbers peer slots to seats.
    virtual void FreezeRoster() {}
    // Rules currently in force (the host's own copy, or the last one received).
    virtual const RulesPacket& Rules() const = 0;
    virtual void StartRace(std::uint16_t countdownCs) { (void)countdownCs; }
    virtual void RestartRace(std::uint16_t countdownCs) { (void)countdownCs; }
    virtual void EndRace(std::vector<RaceEndEntry> results) {}
    virtual void BroadcastLobby() {}
    virtual void SendLapComplete(const LapCompletePacket&) {}
    virtual void SendPlayerFinished(const PlayerFinishedPacket&) {}
    virtual void SendCarRecovered(const CarRecoveredPacket&) {}
    virtual void SendPartDamaged(const PartDamagedPacket&) {}
    virtual void Kick(PeerId) {}
    // Slot -> latest received input, or a zeroed sample when nothing arrived.
    virtual InputSample LatestInput(std::uint8_t slot) const { return {}; }
    virtual bool HasFreshInput(std::uint8_t slot) const { return false; }

    // --- client side -------------------------------------------------------
    virtual void SubmitLocalInput(const InputSample&) {}
    // Snapshot history with arrival stamps, oldest first (interpolation source).
    struct TimedSnapshot {
        SnapshotPacket packet{};
        std::uint64_t arrivalMs = 0;
    };
    virtual const SnapshotPacket* LatestSnapshot() const { return nullptr; }
    virtual std::span<const TimedSnapshot> RecentSnapshots() const { return {}; }
    virtual std::uint16_t AppliedInputSeq() const { return 0; }
    // Newest host tick seen (drives the local tick clock estimate).
    virtual std::uint32_t LatestHostTick() const { return 0; }

    virtual PeerStats LocalStats() const { return {}; }
    virtual std::uint32_t JoinMs() const { return 0; }
};

// Host: `Listen` + advertise `config.roomCode`.
std::unique_ptr<INetSession> CreateHostSession(const NetConfig& config, ITransport& transport,
                                               IRoomDiscovery& discovery);
// Client: connect to `host`. Discover rooms via the same discovery object.
std::unique_ptr<INetSession> CreateClientSession(const NetConfig& config, ITransport& transport,
                                                 IRoomDiscovery& discovery, Endpoint host);

}  // namespace apex::net
