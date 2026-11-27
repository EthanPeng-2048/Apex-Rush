#pragma once

// LAN protocol wire format (see docs/online/protocol.md).
//
// Everything here is a plain value type + pure encode/decode. No sockets, no
// clocks, no globals: the whole file is unit-testable without a network.
// Sizes are pinned by static_asserts in the .cpp and by --net-selftest.

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>
#include <string>
#include <vector>

namespace apex::net {

enum class NetError : std::uint8_t {
    Ok = 0,
    NotReady,
    Closed,
    PeerGone,
    Timeout,
    PayloadTooLarge,
    Malformed,
    UnsupportedVersion,
    Rejected,
    RoomFull,
    RaceInProgress,
    DuplicatePeer,
};

constexpr std::uint8_t kProtocolVersion = 1;
constexpr std::size_t kMaxDatagram = 1200;   // < ENet MTU: never fragment
constexpr int kMaxCars = 8;
constexpr std::size_t kNameBytes = 16;
constexpr std::size_t kMapNameBytes = 24;

// Wire message ids.
enum class MsgType : std::uint8_t {
    Input = 0x10,
    Snapshot = 0x20,
    Hello = 0x40,
    Welcome = 0x41,
    Reject = 0x42,
    LobbyUpdate = 0x43,
    RaceStart = 0x50,
    LapComplete = 0x51,
    PlayerFinished = 0x52,
    RaceEnd = 0x53,
    CarRecovered = 0x54,
    PartDamaged = 0x55,
    PeerLeft = 0x56,
    SessionEnd = 0x57,
    Bye = 0x58,
    Restart = 0x59,
};

// Reject reasons (Welcome/Reject payload).
enum class RejectReason : std::uint8_t {
    ProtocolMismatch = 1,
    BuildMismatch = 2,
    MapMismatch = 3,
    RoomFull = 4,
    RaceInProgress = 5,
    DuplicatePeer = 6,
    Internal = 7,
};

// Session phase mirrored in snapshots (matches game RacePhase numbering below).
enum class NetPhase : std::uint8_t { Lobby = 0, Countdown = 1, Racing = 2, Finished = 3 };

// Input button bits.
enum InputButton : std::uint8_t {
    kBtnHandbrake = 1u << 0,
    kBtnReverse = 1u << 1,
    kBtnLookBack = 1u << 2,
};

// Game rules carried by Welcome / RaceStart.
struct RulesPacket {
    std::uint8_t laps = 3;
    std::uint8_t carCount = 4;
    std::uint8_t damage = 1;
    std::uint8_t seasonBits = 1;   // bit0 DRS, bit1 active aero, bit2 overtake
    std::uint8_t absLevel = 0;
    std::uint8_t tcLevel = 0;
    std::uint8_t escLevel = 0;
    std::uint8_t steerLevel = 0;
    std::uint8_t physicsEngine = 0;  // 0 qu3e, 1 nudge, 2 rpc
    std::uint8_t snapshotHz = 30;
};

// ---------------------------------------------------------------- input (C1)

struct InputFrame {
    std::uint8_t throttle = 0;   // 1/255
    std::uint8_t brake = 0;      // 1/255
    std::int8_t steer = 0;       // 1/127
    std::uint8_t buttons = 0;    // InputButton bits
};

struct InputPacket {
    std::uint16_t firstSeq = 0;                 // newest frame sequence
    std::uint8_t count = 0;                     // frames carried (oldest first)
    std::array<InputFrame, 3> frames{};
    std::uint8_t drsPress = 0;                  // monotonic press counters
    std::uint8_t wingPress = 0;
    std::uint8_t overtakePress = 0;
    std::uint16_t ackSnapshotSeq = 0;
};

// ------------------------------------------------------------- snapshot (C2)

struct CarNetState {
    std::uint8_t slot = 0;
    float pos[3]{};        // world position (decoded from anchor + int16)
    float yaw = 0.f;       // radians, [0, 2pi)
    float pitch = 0.f;     // radians
    float roll = 0.f;      // radians
    float vel[3]{};        // m/s
    float yawRate = 0.f;   // rad/s
    float pitchRate = 0.f;
    float rollRate = 0.f;
    float steerDeg = 0.f;
    float speedKmh = 0.f;
    float rpm = 0.f;
    std::uint8_t gear = 1;
    std::uint8_t wingMode = 0;
    bool drsActive = false;
    bool ersDeploying = false;
    bool finished = false;
    bool drsAvailable = false;
    bool retired = false;
    bool onTrack = true;
    std::uint8_t groundedMask = 0;
    float ersCharge = 0.f;
    std::uint8_t lap = 0;
    std::uint8_t position = 0;
    std::array<float, 4> wheelTravel{};   // 0..1 suspension travel
};

// Full integrator state of the receiving client's own car (rollback support).
struct SelfState {
    std::array<float, 4> slipRatio{};
    std::array<float, 4> slipAngleDeg{};
    std::array<float, 4> wheelSpinRate{};
    std::array<float, 4> wear{};
    std::array<float, 4> tempC{};
    std::array<float, 4> partHealth{};
    float damageTotal = 0.f;
    std::uint8_t shiftCooldownTicks = 0;
    std::uint8_t ersDeployTicks = 0;
    std::uint32_t rngState = 0;
};

struct SnapshotPacket {
    std::uint32_t hostTick = 0;
    std::uint16_t snapSeq = 0;
    std::uint8_t phase = 0;
    std::uint16_t phaseTimerCs = 0;
    std::uint32_t raceClockMs = 0;
    std::uint8_t carCount = 0;
    float anchor[3]{};
    std::uint16_t appliedInputSeq = 0;
    std::array<CarNetState, kMaxCars> cars{};
    bool hasSelf = false;
    SelfState self{};
};

// ------------------------------------------------------------- control (C0)

struct HelloPacket {
    std::uint32_t buildSig = 0;
    std::uint32_t mapHash = 0;
    std::string name;
    std::uint8_t colorPref = 0;
};

struct WelcomePacket {
    std::uint8_t peerId = 0;
    std::uint8_t slot = 0;
    std::uint32_t sessionId = 0;
    RulesPacket rules{};
    std::uint32_t mapHash = 0;
    std::string mapName;
    float gridPos[3]{};
    std::uint16_t gridYaw = 0;
};

struct RejectPacket {
    RejectReason reason = RejectReason::Internal;
    std::uint8_t detail = 0;
};

struct LobbyPlayer {
    std::uint8_t peerId = 0;
    std::uint8_t slot = 0;
    std::string name;
    std::uint8_t color = 0;
    std::uint8_t ready = 0;
};

struct LobbyUpdatePacket {
    std::vector<LobbyPlayer> players;
};

struct RaceStartPacket {
    std::uint32_t startHostTick = 0;
    std::uint16_t countdownCs = 350;
    RulesPacket rules{};
};

struct LapCompletePacket {
    std::uint8_t slot = 0;
    std::uint8_t lap = 0;
    std::uint32_t lapTimeMs = 0;
    std::uint32_t bestLapMs = 0;
};

struct PlayerFinishedPacket {
    std::uint8_t slot = 0;
    std::uint8_t position = 0;
    std::uint32_t raceTimeMs = 0;
    std::uint32_t bestLapMs = 0;
};

struct RaceEndEntry {
    std::uint8_t slot = 0;
    std::uint8_t position = 0;
    std::uint8_t finished = 0;
    std::uint32_t raceTimeMs = 0;
    std::uint32_t bestLapMs = 0;
};

struct RaceEndPacket {
    std::vector<RaceEndEntry> entries;
};

struct CarRecoveredPacket {
    std::uint8_t slot = 0;
    std::uint8_t reason = 0;
};

struct PartDamagedPacket {
    std::uint8_t slot = 0;
    std::uint8_t part = 0;
    std::uint8_t health = 0;   // 1/255
    std::uint8_t impactKmh = 0;
};

struct PeerLeftPacket {
    std::uint8_t peerId = 0;
    std::uint8_t reason = 0;
};

struct SessionEndPacket {
    std::uint8_t reason = 0;
};

struct RestartPacket {
    std::uint32_t startHostTick = 0;
    std::uint16_t countdownCs = 350;
};

// --------------------------------------------------------------- discovery

constexpr std::uint8_t kDiscoveryVersion = 1;
// NOTE: keep these clear of Windows' Hyper-V/WSL dynamically reserved ranges
// (45xxx-46xxx are commonly excluded there and bind() then fails silently).
constexpr std::uint16_t kDiscoveryPortDefault = 51231;
constexpr std::uint16_t kGamePortDefault = 51230;
constexpr std::uint8_t kDiscoAnnounce = 1;
constexpr std::uint8_t kDiscoQuery = 2;

struct AnnouncePacket {
    std::uint16_t roomCode = 0;
    std::uint16_t gamePort = 0;
    std::uint8_t protocolVersion = kProtocolVersion;
    std::uint32_t buildSig = 0;
    std::uint32_t mapHash = 0;
    std::string mapName;
    std::string hostName;
    std::uint8_t players = 1;
    std::uint8_t maxPlayers = 8;
    std::uint8_t phase = 0;
};

struct QueryPacket {
    std::uint16_t roomCode = 0;   // 0 = all rooms
};

// ------------------------------------------------------------- encode/decode
//
// All encoders return the byte count written or PayloadTooLarge. All decoders
// return Malformed on truncation / trailing bytes / bad version. They are pure
// functions: same input -> same bytes, no hidden state.

std::expected<std::size_t, NetError> EncodeInput(const InputPacket& p, std::span<std::byte> out);
std::expected<InputPacket, NetError> DecodeInput(std::span<const std::byte> in);

std::expected<std::size_t, NetError> EncodeSnapshot(const SnapshotPacket& p, std::span<std::byte> out);
std::expected<SnapshotPacket, NetError> DecodeSnapshot(std::span<const std::byte> in);

std::expected<std::size_t, NetError> EncodeHello(const HelloPacket& p, std::span<std::byte> out);
std::expected<HelloPacket, NetError> DecodeHello(std::span<const std::byte> in);

std::expected<std::size_t, NetError> EncodeWelcome(const WelcomePacket& p, std::span<std::byte> out);
std::expected<WelcomePacket, NetError> DecodeWelcome(std::span<const std::byte> in);

std::expected<std::size_t, NetError> EncodeReject(const RejectPacket& p, std::span<std::byte> out);
std::expected<RejectPacket, NetError> DecodeReject(std::span<const std::byte> in);

std::expected<std::size_t, NetError> EncodeLobbyUpdate(const LobbyUpdatePacket& p, std::span<std::byte> out);
std::expected<LobbyUpdatePacket, NetError> DecodeLobbyUpdate(std::span<const std::byte> in);

std::expected<std::size_t, NetError> EncodeRaceStart(const RaceStartPacket& p, std::span<std::byte> out);
std::expected<RaceStartPacket, NetError> DecodeRaceStart(std::span<const std::byte> in);

std::expected<std::size_t, NetError> EncodeLapComplete(const LapCompletePacket& p, std::span<std::byte> out);
std::expected<LapCompletePacket, NetError> DecodeLapComplete(std::span<const std::byte> in);

std::expected<std::size_t, NetError> EncodePlayerFinished(const PlayerFinishedPacket& p, std::span<std::byte> out);
std::expected<PlayerFinishedPacket, NetError> DecodePlayerFinished(std::span<const std::byte> in);

std::expected<std::size_t, NetError> EncodeRaceEnd(const RaceEndPacket& p, std::span<std::byte> out);
std::expected<RaceEndPacket, NetError> DecodeRaceEnd(std::span<const std::byte> in);

std::expected<std::size_t, NetError> EncodeCarRecovered(const CarRecoveredPacket& p, std::span<std::byte> out);
std::expected<CarRecoveredPacket, NetError> DecodeCarRecovered(std::span<const std::byte> in);

std::expected<std::size_t, NetError> EncodePartDamaged(const PartDamagedPacket& p, std::span<std::byte> out);
std::expected<PartDamagedPacket, NetError> DecodePartDamaged(std::span<const std::byte> in);

std::expected<std::size_t, NetError> EncodePeerLeft(const PeerLeftPacket& p, std::span<std::byte> out);
std::expected<PeerLeftPacket, NetError> DecodePeerLeft(std::span<const std::byte> in);

std::expected<std::size_t, NetError> EncodeSessionEnd(const SessionEndPacket& p, std::span<std::byte> out);
std::expected<SessionEndPacket, NetError> DecodeSessionEnd(std::span<const std::byte> in);

std::expected<std::size_t, NetError> EncodeRestart(const RestartPacket& p, std::span<std::byte> out);
std::expected<RestartPacket, NetError> DecodeRestart(std::span<const std::byte> in);

std::expected<std::size_t, NetError> EncodeBye(std::span<std::byte> out);

std::expected<std::size_t, NetError> EncodeAnnounce(const AnnouncePacket& p, std::span<std::byte> out);
std::expected<AnnouncePacket, NetError> DecodeAnnounce(std::span<const std::byte> in);

std::expected<std::size_t, NetError> EncodeQuery(const QueryPacket& p, std::span<std::byte> out);
std::expected<QueryPacket, NetError> DecodeQuery(std::span<const std::byte> in);

// Peek the message type without decoding the body (dispatch helper).
std::expected<MsgType, NetError> PeekMessageType(std::span<const std::byte> in);

// True when `a` is newer than `b` under u16 wrap-around.
inline bool SeqNewer(std::uint16_t a, std::uint16_t b) {
    return static_cast<std::int16_t>(static_cast<std::uint16_t>(a - b)) > 0;
}

}  // namespace apex::net
