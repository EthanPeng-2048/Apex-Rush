// LAN mode: room UI, session plumbing and the client/host race paths.
//
// Game.h declares these; keeping the definitions here stops Game.cpp from
// growing a second personality. See docs/online/architecture.md.

#include "game/Game.h"

#include "net/roster.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>

namespace apex {
namespace {

constexpr std::uint64_t kQueryIntervalMs = 1000;

std::uint64_t NowMs() {
    using clock = std::chrono::steady_clock;
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(clock::now().time_since_epoch())
            .count());
}

std::uint32_t Fnv1a(const void* data, std::size_t size, std::uint32_t seed = 2166136261u) {
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    std::uint32_t h = seed;
    for (std::size_t i = 0; i < size; ++i) {
        h ^= bytes[i];
        h *= 16777619u;
    }
    return h;
}

// Build signature: catches "same map, different physics constants" pairs.
// The digest is value-only (engine-side ParamsDigest); hashing the raw struct
// here also covered its uninitialised tail padding, which let two machines
// running the same exe disagree on the signature.
std::uint32_t BuildSignature() {
    return ParamsDigest(VehicleParams{}, Fnv1a("apex-rush-lan-v1", 16));
}

std::uint32_t HashMapFile(const std::string& mapName) {
    const std::string path = DataPath("assets/maps/" + mapName + ".rushmap");
    std::ifstream in(path, std::ios::binary);
    // No map on this machine = hash 0 (docs/online/protocol.md): the host
    // rejects it with MAP MISSING OR DIFFERENT instead of admitting a car onto
    // a track it cannot build.
    if (!in) return 0;
    std::uint32_t h = 2166136261u;
    char buf[4096];
    while (in.read(buf, sizeof(buf)) || in.gcount() > 0) {
        h = Fnv1a(buf, static_cast<std::size_t>(in.gcount()), h);
        if (!in) break;
    }
    return h;
}

const char* RejectText(net::RejectReason reason) {
    switch (reason) {
        case net::RejectReason::ProtocolMismatch:
            return "VERSION MISMATCH";
        case net::RejectReason::BuildMismatch:
            return "GAME BUILD MISMATCH";
        case net::RejectReason::MapMismatch:
            return "MAP MISSING OR DIFFERENT";
        case net::RejectReason::RoomFull:
            return "ROOM IS FULL";
        case net::RejectReason::RaceInProgress:
            return "RACE ALREADY RUNNING";
        case net::RejectReason::DuplicatePeer:
            return "ALREADY JOINED";
        default:
            return "JOIN FAILED";
    }
}

}  // namespace

// ---------------------------------------------------------------- helpers ---

std::unique_ptr<net::IRoomDiscovery> Game::MakeDiscovery() {
    net::DiscoveryOptions opts;
    opts.portSpan = static_cast<std::uint16_t>(
        std::clamp(config_.GetInt("net", "discovery_port_span", net::kDiscoveryPortSpan), 1, 64));
    opts.multicast = config_.GetBool("net", "discovery_multicast", true);
    const std::uint16_t port = static_cast<std::uint16_t>(
        std::clamp(config_.GetInt("net", "discovery_port", net::kDiscoveryPortDefault), 1, 65535));
    return net::MakeUdpDiscovery(port, opts);
}

void Game::EnterLan() {
    if (netRole_ != NetRole::Offline) return;
    // Identity comes first: the seat layout is derived from player ids, so a
    // player who has not said who they are cannot be put on a grid.
    if (ChosenPlayerName().empty()) {
        EnterLanNameEntry();
        return;
    }
    if (!netDiscovery_) netDiscovery_ = MakeDiscovery();
    (void)netDiscovery_->BeginSearch();
    lanUi_ = LanUi::Root;
    lanScreensStale_ = true;   // reopening LAN refocuses the room browser
    lanMessage_.clear();
    lanMessageTimer_ = 0.f;
    phase_ = RacePhase::Lan;
}

// Name resolution, highest priority first:
//   1. `--player <name>`: an explicit one-off override for automation and QA.
//   2. `[net] player_name`: the persisted choice (written by the login screen,
//      or edited in the config by hand).
//   3. the name typed on this session's login screen.
// Empty means nobody has chosen: the login screen asks, and unattended roles
// fall back to a default (see EffectivePlayerName).
std::string Game::ChosenPlayerName() const {
    if (!playerNameCli_.empty()) return TrimPlayerName(playerNameCli_);
    const std::string configured = config_.GetString("net", "player_name", "");
    if (!configured.empty()) return TrimPlayerName(configured);
    return TrimPlayerName(lanPlayerName_);
}

std::string Game::EffectivePlayerName(bool host) const {
    const std::string chosen = ChosenPlayerName();
    if (!chosen.empty()) return chosen;
    // --drive-test / --net-host / --net-join never open the login screen, so
    // they need a seatable name without asking. The role decides the default:
    // netRole_ is still Offline while the session is being created.
    if (host) return "HOST";
    return "PLAYER-" + std::to_string(std::max(1, lanJoinOrdinal_));
}

void Game::EnterLanNameEntry() {
    // The screen itself is built with the other LAN screens (BuildLanScreens);
    // this only routes to it. SyncLanScreen sees the sub-state change and
    // refocuses, exactly like every other LAN screen.
    lanUi_ = LanUi::NameEntry;
    lanMessage_.clear();
    lanMessageTimer_ = 0.f;
    phase_ = RacePhase::Lan;
}

void Game::UpdateLanNameEntry() {
    // Called from UpdateLan after the engine resolved the text field: the buffer
    // is already edited, this only decides what the action means.
    UiScreen& screen = screens_.lanName;
    const UiAction action = ui_.Update(screen, rawInput_);
    if (action.kind == UiActionKind::Back) {
        phase_ = RacePhase::Menu;
        return;
    }
    if (action.kind != UiActionKind::Activate) return;
    if (screen.widgets.empty()) return;
    const std::string typed = TrimPlayerName(screen.widgets.front().value);
    if (typed.empty()) {
        lanMessage_ = "NAME CANNOT BE EMPTY";
        lanMessageTimer_ = 2.f;
        return;
    }
    lanPlayerName_ = typed;
    // Persisted so the next launch starts from the same identity (the config is
    // only written on a clean exit, as everywhere else).
    config_.SetString("net", "player_name", typed);
    EnterLan();
}

bool Game::AutoHost(std::uint16_t roomCode) {
    if (netRole_ != NetRole::Offline) return false;
    LanStartHost(roomCode);
    return netRole_ == NetRole::Host;
}

bool Game::AutoJoin(std::uint16_t roomCode, const std::string& hostIp) {
    if (netRole_ != NetRole::Offline) return false;
    net::Endpoint ep;
    ep.port = static_cast<std::uint16_t>(
        std::clamp(config_.GetInt("net", "game_port", net::kGamePortDefault), 1, 65535));
    if (!hostIp.empty() && hostIp != "127.0.0.1") {
        unsigned a = 0, b = 0, c = 0, d = 0;
        if (std::sscanf(hostIp.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
            ep.ipv4 = (a << 24) | (b << 16) | (c << 8) | d;
        }
    } else {
        ep.ipv4 = 0x7F000001u;   // loopback
    }
    LanStartClient(roomCode, ep);
    return netRole_ == NetRole::Client;
}

void Game::LanStartHost(std::uint16_t roomCode) {
    const std::string name = EffectivePlayerName(true);
    localPlayerId_ = MakePlayerId(name);
    netConfig_ = net::NetConfig{};
    netConfig_.playerName = name;
    netConfig_.playerId = localPlayerId_;
    netConfig_.roomCode = roomCode;
    netConfig_.gamePort = static_cast<std::uint16_t>(
        std::clamp(config_.GetInt("net", "game_port", net::kGamePortDefault), 1, 65535));
    netConfig_.discoveryPort = static_cast<std::uint16_t>(
        std::clamp(config_.GetInt("net", "discovery_port", net::kDiscoveryPortDefault), 1, 65535));
    netConfig_.maxPlayers = std::clamp(config_.GetInt("net", "max_players", 8), 2, net::kMaxCars);
    netConfig_.snapshotHz = std::clamp(config_.GetInt("net", "snapshot_hz", 30), 10, 60);
    netConfig_.inputHz = std::clamp(config_.GetInt("net", "input_hz", 60), 10, 60);
    netConfig_.buildSig = BuildSignature();
    netConfig_.mapHash = HashMapFile(mapName_);
    netConfig_.mapName = mapName_;
    FillHostRules(netConfig_.rules);
    netConfig_.rules.carCount = 1;

    netTransport_ = net::MakeEnetTransport();
    if (!netTransport_) {
        LanStop("NETWORK INIT FAILED");
        return;
    }
    if (!netDiscovery_) netDiscovery_ = MakeDiscovery();
    netSession_ = net::CreateHostSession(netConfig_, *netTransport_, *netDiscovery_);
    netRole_ = NetRole::Host;
    const net::DiscoveryStatus disco =
        netDiscovery_ ? netDiscovery_->Status() : net::DiscoveryStatus{};
    std::printf("[net] hosting room %04u on udp %u (discovery %u%s) as %s\n",
                static_cast<unsigned>(netConfig_.roomCode),
                static_cast<unsigned>(netConfig_.gamePort),
                static_cast<unsigned>(disco.boundPort),
                disco.multicast ? "" : " broadcast-only", netConfig_.playerName.c_str());
    netCarCount_ = 1;
    netLobbyCount_ = 0;
    netRoster_ = net::Roster{};
    netLocalSeat_.store(0, std::memory_order_relaxed);
    lanUi_ = LanUi::Lobby;
    phase_ = RacePhase::Lan;
}

void Game::LanStartClient(std::uint16_t roomCode, const net::Endpoint& host) {
    if (netRole_ != NetRole::Offline) return;
    ++lanJoinOrdinal_;
    const std::string name = EffectivePlayerName(false);
    localPlayerId_ = MakePlayerId(name);
    netConfig_ = net::NetConfig{};
    netConfig_.playerName = name;
    netConfig_.playerId = localPlayerId_;
    netConfig_.roomCode = roomCode;
    netConfig_.gamePort = static_cast<std::uint16_t>(
        std::clamp(config_.GetInt("net", "game_port", net::kGamePortDefault), 1, 65535));
    netConfig_.discoveryPort = static_cast<std::uint16_t>(
        std::clamp(config_.GetInt("net", "discovery_port", net::kDiscoveryPortDefault), 1, 65535));
    netConfig_.buildSig = BuildSignature();
    netConfig_.mapHash = HashMapFile(mapName_);
    netConfig_.mapName = mapName_;

    netTransport_ = net::MakeEnetTransport();
    if (!netTransport_) {
        LanStop("NETWORK INIT FAILED");
        return;
    }
    if (!netDiscovery_) netDiscovery_ = MakeDiscovery();
    netSession_ = net::CreateClientSession(netConfig_, *netTransport_, *netDiscovery_, host);
    netRole_ = NetRole::Client;
    std::printf("[net] joining room %04u at %u.%u.%u.%u:%u as %s\n",
                static_cast<unsigned>(netConfig_.roomCode), (host.ipv4 >> 24) & 0xFF,
                (host.ipv4 >> 16) & 0xFF, (host.ipv4 >> 8) & 0xFF, host.ipv4 & 0xFF,
                static_cast<unsigned>(host.port), netConfig_.playerName.c_str());
    lanUi_ = LanUi::Lobby;
    lanJoinStartMs_ = NowMs();
    phase_ = RacePhase::Lan;
}

void Game::LanStop(const std::string& message) {
    if (netProxies_ && world_) netProxies_->Dispose(*world_);
    netProxies_.reset();
    netPredict_.reset();
    netInterp_.reset();
    netSession_.reset();
    if (netDiscovery_) netDiscovery_->StopAdvertising();
    netTransport_.reset();
    netRole_ = NetRole::Offline;
    netCarCount_ = 1;
    netRoster_ = net::Roster{};
    netRulesLocked_ = false;
    netLocalSeat_.store(0, std::memory_order_relaxed);
    netReadout_ = NetRaceReadout{};
    netAppliedSeq_.fill(0);
    clientCorrection_.Reset();
    {
        std::lock_guard lock(netInputMutex_);
        netRemoteFresh_.fill(false);
    }
    lanMessage_ = message;
    lanMessageTimer_ = 3.f;
    lanUi_ = message.empty() ? LanUi::Root : LanUi::Message;
    phase_ = RacePhase::Lan;
    // Back to the parked offline scene so the 3D view is not a dead race.
    StartRace();
    phase_ = RacePhase::Lan;
}

// ------------------------------------------------------------------- tick ---

void Game::LanTick() {
    if (!netSession_) return;
    const std::uint64_t now = NowMs();
    netSession_->Tick(now);
    LanHandleEvents();
    if (!netSession_) return;   // events may have torn the session down

    if (netRole_ == NetRole::Host) {
        {
            // Remote input mailbox: main thread fills, sim thread consumes. It
            // is keyed by seat (a peer's slot is its seat once the roster is
            // frozen), so a hole in the numbering simply leaves a slot cold
            // instead of shifting everyone's input onto the wrong car.
            std::lock_guard lock(netInputMutex_);
            for (int slot = 0; slot < net::kMaxCars; ++slot) {
                netRemoteInput_[static_cast<std::size_t>(slot)] = netSession_->LatestInput(
                    static_cast<std::uint8_t>(slot));
                // The seq travels with the sample it belongs to: the host reports
                // "this state was built from that input" per recipient.
                netRemoteInputSeq_[static_cast<std::size_t>(slot)] = netSession_->LatestInputSeq(
                    static_cast<std::uint8_t>(slot));
                netRemoteFresh_[static_cast<std::size_t>(slot)] = netSession_->HasFreshInput(
                    static_cast<std::uint8_t>(slot));
            }
        }
        std::lock_guard lock(hostWorldMutex_);
        if (hostWorldFresh_) {
            hostWorldFresh_ = false;
            netSession_->PublishSnapshot(hostWorld_);
        }
        if (now - netLogMs_ >= 1000) {
            netLogMs_ = now;
            const net::PeerStats st = netSession_->LocalStats();
            std::printf("[net] host t=%.0fs phase=%d cars=%d session=%d lobby=%u\n",
                        static_cast<double>(now) / 1000.0,
                        static_cast<int>(static_cast<RacePhase>(phase_)),
                        static_cast<int>(netCarCount_), static_cast<int>(netSession_->State()),
                        static_cast<unsigned>(netLobbyCount_));
            (void)st;
            for (const Racer& racer : racers_) {
                const Vec3& p = racer.vehicle.State().transform.position;
                std::printf("[net] host car seat=%d name=%s pos=(%.1f,%.1f,%.1f)\n", racer.seat,
                            racer.name.c_str(), static_cast<double>(p.x),
                            static_cast<double>(p.y), static_cast<double>(p.z));
            }
            std::fflush(stdout);
        }
        return;
    }

    // Client: publish the sample, feed the interpolator, mirror the authority.
    // The handshake events already synced the seat; re-reading it here covers
    // the case where the session moved on without an event.
    netLocalSeat_.store(netSession_->LocalSlot(), std::memory_order_relaxed);
    if (phase_ == RacePhase::Racing || phase_ == RacePhase::Countdown) {
        const net::InputSample sample = LanSampleLocalInput();
        {
            std::lock_guard lock(netInputMutex_);
            netLocalSample_ = sample;
        }
        netSession_->SubmitLocalInput(sample);
        // One sequence space: the session stamps the sample, the rollback history
        // records the same number, the host echoes it back in appliedInputSeq.
        localInputSeq_ = netSession_->LocalInputSeq();
    }
    if (const net::SnapshotPacket* snap = netSession_->LatestSnapshot()) {
        if (netInterp_ && snap->hostTick != lastPushedHostTick_) {
            netInterp_->Push(*snap, now);
            lastPushedHostTick_ = snap->hostTick;
        }
        if (now - netLogMs_ >= 1000) {
            netLogMs_ = now;
            const net::PeerStats st = netSession_->LocalStats();
            const VehicleState& own =
                racers_.empty() ? VehicleState{} : LocalRacer().vehicle.State();
            std::printf("[net] client t=%.0fs rtt=%ums cars=%u lap=%u pos=%u/%u seat=%d "
                        "pos3=(%.1f,%.1f,%.1f) speed=%.0fkm/h "
                        "rollback=%d snap=%d corr=%.0fmm\n",
                        static_cast<double>(now) / 1000.0, st.rttMs,
                        static_cast<unsigned>(snap->carCount), static_cast<unsigned>(netReadout_.lap),
                        static_cast<unsigned>(netReadout_.position),
                        static_cast<unsigned>(netReadout_.carCount), LocalSeat(),
                        static_cast<double>(own.transform.position.x),
                        static_cast<double>(own.transform.position.y),
                        static_cast<double>(own.transform.position.z),
                        static_cast<double>(own.speedKmh), netReconciles_, netSnaps_,
                        static_cast<double>(netCorrMaxM_ * 1000.f));
            netReconciles_ = 0;
            netSnaps_ = 0;
            netCorrMaxM_ = 0.f;
            std::fflush(stdout);
        }
        netReadout_.valid = true;
        netReadout_.phase = snap->phase;
        netReadout_.raceClockMs = snap->raceClockMs;
        netReadout_.carCount = snap->carCount;
        const std::uint8_t localSeat = netLocalSeat_.load(std::memory_order_relaxed);
        for (int i = 0; i < snap->carCount; ++i) {
            const net::CarNetState& car = snap->cars[static_cast<std::size_t>(i)];
            if (car.slot != localSeat) continue;
            netReadout_.lap = car.lap;
            netReadout_.position = car.position;
            break;
        }
    }
}

void Game::LanHandleEvents() {
    if (!netSession_) return;
    for (const net::SessionEvent& ev : netSession_->DrainEvents()) {
        switch (ev.kind) {
            case net::SessionEventKind::PeerJoined: {
                std::printf("[net] peer %u joined as slot %u\n", static_cast<unsigned>(ev.peer.value),
                            static_cast<unsigned>(ev.slot));
                if (lanUi_ == LanUi::Lobby) lanMessage_.clear();
                break;
            }
            case net::SessionEventKind::LobbyChanged: {
                netLobbyCount_ = static_cast<std::uint8_t>(
                    std::min<std::size_t>(ev.lobby.size(), net::kMaxCars));
                for (std::size_t i = 0; i < netLobbyCount_; ++i) netLobby_[i] = ev.lobby[i];
                break;
            }
            case net::SessionEventKind::RulesReceived: {
                // Sync the seat inside the event: the same frame's RaceStarted
                // must not read the previous (arrival-order) slot.
                netLocalSeat_.store(ev.slot, std::memory_order_relaxed);
                netConfig_.rules = ev.rules;
                ApplyHostRules(ev.rules);
                std::printf("[net] joined as slot %u (%s)\n", static_cast<unsigned>(ev.slot),
                            netConfig_.playerName.c_str());
                break;
            }
            case net::SessionEventKind::JoinRejected: {
                LanStop(std::string("JOIN REJECTED: ") + RejectText(ev.reason));
                return;
            }
            case net::SessionEventKind::RaceStarted: {
                std::printf("[net] race starting (laps=%u cars=%u seat=%u)\n",
                            static_cast<unsigned>(ev.rules.laps),
                            static_cast<unsigned>(ev.rules.carCount),
                            static_cast<unsigned>(ev.slot));
                if (netRole_ == NetRole::Client) {
                    // The roster travels with the rules: our seat is our rank in
                    // it, known before a single snapshot arrives.
                    netLocalSeat_.store(ev.slot, std::memory_order_relaxed);
                    netConfig_.rules = ev.rules;
                    ApplyHostRules(ev.rules);
                    LanBeginClientRace();
                }
                break;
            }
            case net::SessionEventKind::Restart: {
                if (netRole_ == NetRole::Client) {
                    if (racers_.empty()) {
                        LanBeginClientRace();
                    } else {
                        // Same grid formula as the spawn: our own box, not box 0.
                        ResetGridForStart();
                        localTick_ = 0;
                        if (netPredict_) netPredict_->Reset(0);
                        clientCorrection_.Reset();
                        phase_ = RacePhase::Countdown;
                        phaseTimer_ = static_cast<float>(ev.countdownCs) * 0.01f;
                        raceClock_ = 0.f;
                    }
                }
                break;
            }
            case net::SessionEventKind::LapComplete: {
                netReadout_.valid = true;
                break;
            }
            case net::SessionEventKind::PlayerFinished:
            case net::SessionEventKind::RaceEnded: {
                if (netRole_ == NetRole::Client) {
                    netReadout_.valid = true;
                    phase_ = RacePhase::Finished;
                }
                break;
            }
            case net::SessionEventKind::HostGone: {
                LanStop("HOST DISCONNECTED");
                return;
            }
            case net::SessionEventKind::SessionEnded: {
                LanStop("SESSION ENDED");
                return;
            }
            case net::SessionEventKind::PeerLeft: {
                break;
            }
            default:
                break;
        }
    }
}

UiScreen& Game::ActiveLanScreen() {
    switch (lanUi_) {
        case LanUi::RoomCode:
            return screens_.lanCode;
        case LanUi::RoomList:
            return screens_.lanRooms;
        case LanUi::Lobby:
            return screens_.lanLobby;
        case LanUi::Message:
            return screens_.lanMessage;
        case LanUi::NameEntry:
            return screens_.lanName;
        case LanUi::Root:
        default:
            return screens_.lanRoot;
    }
}

void Game::BuildLanScreens() {
    const auto skeleton = [](UiScreen& s, const char* hint, float rowStartY) {
        s.title = "LAN RACE";
        s.titleSize = 42;
        s.x = 96.f;
        s.y = 70.f;
        s.rowStartY = rowStartY;
        s.rowHeight = 44.f;
        s.dimBackground = false;   // RenderLan paints the opaque LAN backdrop
        s.hint = hint;
    };

    if (screens_.lanRoot.widgets.empty()) {
        UiScreen& s = screens_.lanRoot;
        skeleton(s, "W/S SELECT    ENTER CONFIRM    ESC BACK", 170.f);
        s.widgets = {
            {.kind = UiKind::Button, .id = "host", .label = "HOST RACE", .size = 26},
            {.kind = UiKind::Button, .id = "join", .label = "JOIN RACE", .size = 26},
            {.kind = UiKind::Button, .id = "back", .label = "BACK", .size = 26},
        };
    }
    if (screens_.lanCode.widgets.empty()) {
        // The room code is one shared parameter: HOST and JOIN both edit this
        // wheel, and the role only decides what a confirmed code does.
        UiScreen& s = screens_.lanCode;
        skeleton(s, "LEFT/RIGHT PICK DIGIT   UP/DOWN CHANGE   ENTER CONFIRM   ESC BACK", 150.f);
        s.widgets = {{.kind = UiKind::Digits, .id = "code", .label = "ROOM CODE",
                      .value = "0000", .size = 26, .digits = 4}};
    }
    if (screens_.lanRooms.widgets.empty()) {
        skeleton(screens_.lanRooms, "ENTER JOIN    ESC BACK", 170.f);
    }
    if (screens_.lanName.widgets.empty()) {
        // Login: one text field, nothing else. The field swallows the
        // navigation keys while focused, so a second row would be unreachable.
        UiScreen& s = screens_.lanName;
        skeleton(s, "A-Z 0-9   ENTER CONFIRM   ESC BACK", 260.f);
        s.y = 170.f;   // below the "LAN RACE" heading RenderLan already drew
        s.widgets = {{.kind = UiKind::TextField, .id = "player_name", .label = "NAME",
                      .value = "", .size = 26, .digits = 15}};
    }
    if (screens_.lanMessage.widgets.empty()) {
        UiScreen& s = screens_.lanMessage;
        skeleton(s, "", 180.f);
        s.rowHeight = 60.f;
        s.widgets = {
            {.kind = UiKind::Label, .label = "", .size = 26},
            {.kind = UiKind::Button, .id = "continue", .label = "ENTER TO CONTINUE", .size = 20},
        };
    }
    // Only the host can start the race, so the lobby skeleton is per role.
    UiScreen& lobby = screens_.lanLobby;
    if (lobby.widgets.empty() || lanLobbyRole_ != netRole_) {
        lanLobbyRole_ = netRole_;
        skeleton(lobby, "W/S SELECT    ENTER CONFIRM    ESC BACK", 150.f);
        lobby.widgets.clear();
        lobby.widgets.push_back({.kind = UiKind::Header, .label = "LOBBY", .size = 24});
        if (netRole_ == NetRole::Host) {
            lobby.widgets.push_back(
                {.kind = UiKind::Button, .id = "start", .label = "START RACE", .size = 26});
        } else {
            lobby.widgets.push_back(
                {.kind = UiKind::Label, .label = "WAITING FOR HOST...", .size = 26});
        }
        lobby.widgets.push_back({.kind = UiKind::Button, .id = "leave", .label = "LEAVE", .size = 26});
    }
}

void Game::SyncLanScreen() {
    BuildLanScreens();
    const bool reopened = lanScreensStale_ || lanUi_ != lanScreenState_;
    lanScreensStale_ = false;
    lanScreenState_ = lanUi_;

    // Dynamic text is refreshed in place: rebuilding the screen would drop the
    // focus index, the digit sub-cursor and the row focus flags.
    if (lanUi_ == LanUi::Root) {
        char hint[96];
        std::snprintf(hint, sizeof(hint),
                      "ROOM CODE %04u    W/S SELECT    ENTER CONFIRM    ESC BACK",
                      static_cast<unsigned>(LanRoomCode()));
        screens_.lanRoot.hint = hint;
    } else if (lanUi_ == LanUi::RoomList) {
        UiScreen& s = screens_.lanRooms;
        const int keep = s.cursor;
        s.widgets.clear();
        for (const net::RoomInfo& room : lanRooms_) {
            char entry[128];
            std::snprintf(entry, sizeof(entry), "%04u  %s  %s  %u/%u", room.announce.roomCode,
                          room.announce.hostName.c_str(), room.announce.mapName.c_str(),
                          static_cast<unsigned>(room.announce.players),
                          static_cast<unsigned>(room.announce.maxPlayers));
            s.widgets.push_back({.kind = UiKind::Button, .id = "room", .label = entry, .size = 26});
        }
        if (s.widgets.empty()) {
            // "No rooms" has three different causes; show which one it is, so
            // the player is not sent after a firewall that was never at fault.
            const net::DiscoveryStatus status =
                netDiscovery_ ? netDiscovery_->Status() : net::DiscoveryStatus{};
            char entry[128];
            if (status.boundPort == 0) {
                std::snprintf(entry, sizeof(entry), "NO ROOMS - DISCOVERY SOCKET FAILED");
            } else {
                std::snprintf(entry, sizeof(entry), "NO ROOMS FOUND  UDP %u %s  TX %u RX %u",
                              static_cast<unsigned>(status.boundPort),
                              status.multicast ? "MCAST" : "BCAST",
                              static_cast<unsigned>(status.sent),
                              static_cast<unsigned>(status.received));
            }
            s.widgets.push_back({.kind = UiKind::Label, .label = entry, .size = 22});
        }
        // The list is discovered, so its length moves under the focus: keep the
        // cursor inside it instead of trusting last frame's index.
        s.cursor = std::clamp(keep, 0, static_cast<int>(s.widgets.size()) - 1);
    } else if (lanUi_ == LanUi::Lobby) {
        UiScreen& s = screens_.lanLobby;
        // Player rows go after the buttons (the first rows never move), so a
        // player joining or leaving cannot shift a focusable index.
        s.widgets.resize(2);
        for (int i = 0; i < netLobbyCount_; ++i) {
            const net::LobbyPlayer& player = netLobby_[static_cast<std::size_t>(i)];
            char entry[96];
            std::snprintf(entry, sizeof(entry), "P%d  %s", player.slot + 1, player.name.c_str());
            s.widgets.push_back({.kind = UiKind::Label, .label = entry, .size = 22});
        }
    } else if (lanUi_ == LanUi::Message) {
        screens_.lanMessage.widgets[0].label = lanMessage_;
    }

    if (reopened) UiController::ResetFocus(ActiveLanScreen());
}

std::uint16_t Game::LanRoomCode() {
    BuildLanScreens();
    const std::string& digits = screens_.lanCode.widgets[0].value;
    std::uint16_t code = 0;
    for (int i = 0; i < 4; ++i) {
        const char d = i < static_cast<int>(digits.size()) ? digits[static_cast<std::size_t>(i)] : '0';
        code = static_cast<std::uint16_t>(code * 10 + ((d >= '0' && d <= '9') ? d - '0' : 0));
    }
    return code;
}

void Game::LanOpenRoomCode(LanRole role) {
    BuildLanScreens();
    lanPendingRole_ = role;
    lanUi_ = LanUi::RoomCode;
    lanScreenState_ = LanUi::RoomCode;
    UiController::ResetFocus(screens_.lanCode);
}

void Game::LanConfirmRoomCode() {
    const std::uint16_t code = LanRoomCode();
    if (lanPendingRole_ == LanRole::Host) {
        LanStartHost(code);
        return;
    }
    lanRooms_.clear();
    lanQueryNextMs_ = 0;   // UpdateLan sends the first query on the next frame
    lanUi_ = LanUi::RoomList;
}

void Game::LanLeaveToMenu() {
    if (netDiscovery_) netDiscovery_->StopAdvertising();
    LanStop("");
    lanUi_ = LanUi::Root;
    phase_ = RacePhase::Menu;
}

void Game::UpdateLan() {
    if (lanMessageTimer_ > 0.f) lanMessageTimer_ -= renderer_.FrameTime();

    // Discovery polling and room filtering are state work, not screen work.
    if (lanUi_ == LanUi::RoomList) {
        const std::uint16_t code = LanRoomCode();
        // One query is one point of failure: a dropped datagram - or a host
        // whose reply crosses a filtered link - leaves the browser empty
        // forever. While the player is looking, ask once a second.
        const std::uint64_t now = NowMs();
        if (netDiscovery_ && now >= lanQueryNextMs_) {
            lanQueryNextMs_ = now + kQueryIntervalMs;
            netDiscovery_->SendQuery(code);
        }
        if (netDiscovery_) netDiscovery_->Poll(now);
        const std::span<const net::RoomInfo> rooms =
            netDiscovery_ ? netDiscovery_->Rooms() : std::span<const net::RoomInfo>{};
        lanRooms_.clear();
        for (const net::RoomInfo& room : rooms) {
            if (room.announce.protocolVersion != net::kProtocolVersion) continue;
            if (code != 0 && room.announce.roomCode != code) continue;
            lanRooms_.push_back(room);
        }
    }
    // Automation: --net-auto starts as soon as a second player is in.
    if (lanUi_ == LanUi::Lobby && netRole_ == NetRole::Host &&
        config_.GetBool("net", "autostart_race", false) && netLobbyCount_ >= 2) {
        LanRequestRaceLoad();
        return;
    }

    SyncLanScreen();
    if (lanUi_ == LanUi::NameEntry) {
        UpdateLanNameEntry();
        return;
    }
    UiScreen& screen = ActiveLanScreen();
    const UiAction action = ui_.Update(screen, rawInput_);

    switch (lanUi_) {
        case LanUi::Root: {
            // ESC on the root screen has always left the LAN mode entirely.
            if (action.kind == UiActionKind::Back) {
                LanLeaveToMenu();
                return;
            }
            if (action.kind != UiActionKind::Activate) break;
            if (action.id == "host") {
                LanOpenRoomCode(LanRole::Host);
            } else if (action.id == "join") {
                LanOpenRoomCode(LanRole::Join);
            } else {
                LanLeaveToMenu();
            }
            break;
        }
        case LanUi::RoomCode: {
            if (action.kind == UiActionKind::Back) {
                lanUi_ = LanUi::Root;
                break;
            }
            // Enter confirms the code: HOST opens the room with it, JOIN
            // searches for it. The wheel itself was edited by the engine.
            if (action.kind == UiActionKind::Activate) LanConfirmRoomCode();
            break;
        }
        case LanUi::RoomList: {
            if (action.kind == UiActionKind::Back) {
                lanUi_ = LanUi::RoomCode;
                break;
            }
            if (action.kind != UiActionKind::Activate) break;
            const int row = screen.cursor;
            if (row < 0 || row >= static_cast<int>(lanRooms_.size())) break;  // "no rooms" row
            LanStartClient(LanRoomCode(), lanRooms_[static_cast<std::size_t>(row)].host);
            break;
        }
        case LanUi::Lobby: {
            if (action.kind == UiActionKind::Back) {
                LanStop("LEFT ROOM");
                return;
            }
            if (action.kind != UiActionKind::Activate) break;
            if (action.id == "start") {
                if (netLobbyCount_ < 2) {
                    lanMessage_ = "WAITING FOR PLAYERS";
                    lanMessageTimer_ = 2.f;
                } else {
                    LanRequestRaceLoad();
                }
            } else {
                LanStop("LEFT ROOM");
                return;
            }
            break;
        }
        case LanUi::Message: {
            if (action.kind == UiActionKind::Activate || action.kind == UiActionKind::Back) {
                if (netRole_ != NetRole::Offline) LanStop("");
                lanUi_ = LanUi::Root;
            }
            break;
        }
    }

    // A handler may have switched sub-state: rebuild now so this frame draws
    // the screen the player is actually on.
    if (lanUi_ != lanScreenState_) SyncLanScreen();
}

void Game::RenderLan() {
    const float w = static_cast<float>(renderer_.ScreenWidth());
    const float h = static_cast<float>(renderer_.ScreenHeight());
    renderer_.DrawRect2D(0.f, 0.f, w, h, Color{0.05f, 0.06f, 0.08f, 1.f});
    ui_.Draw(ActiveLanScreen(), renderer_);

    if (lanUi_ == LanUi::Lobby && netSession_) {
        const net::PeerStats stats = netSession_->LocalStats();
        char info[96];
        std::snprintf(info, sizeof(info), "RTT %ums", stats.rttMs);
        renderer_.DrawText2D(info, w - 220.f, h - 60.f, 18, Color::Gray());
    }
    if (lanMessageTimer_ > 0.f && !lanMessage_.empty() && lanUi_ != LanUi::Message) {
        renderer_.DrawText2D(lanMessage_, 96.f, h - 100.f, 22, Color{0.95f, 0.7f, 0.5f, 1.f});
    }
}

// ------------------------------------------------------------- race start ---

void Game::LanRequestRaceLoad() {
    // Host: freeze the seat layout first (rank of playerId, see net/roster.h),
    // then build the grid from it. The same roster goes out with the rules, so
    // every client puts itself on the box the host gave it.
    netEndSent_ = false;
    FillHostRules(netConfig_.rules);
    netSession_->SetRules(netConfig_.rules);
    netSession_->FreezeRoster();
    netConfig_.rules = netSession_->Rules();
    netRoster_ = netConfig_.rules.roster;
    netCarCount_ = static_cast<std::uint8_t>(std::clamp<int>(netRoster_.count, 1, net::kMaxCars));
    if (const auto seat = net::SeatOf(netRoster_, localPlayerId_)) {
        netLocalSeat_.store(*seat, std::memory_order_relaxed);
    }
    netSession_->BroadcastLobby();
    std::lock_guard lock(hostWorldMutex_);
    hostWorldFresh_ = false;
    StartRace();
    if (netRole_ == NetRole::Host) {
        netSession_->StartRace(static_cast<std::uint16_t>(3.5f * 100));
    }
    std::printf("[net] grid:");
    for (const Racer& racer : racers_) {
        std::printf(" seat=%d:%s", racer.seat, racer.name.c_str());
    }
    std::printf(" (local seat %d)\n", LocalSeat());
    std::fflush(stdout);
}

void Game::ApplyHostRules(const net::RulesPacket& rules) {
    // The host's rules are the simulation: season gates and assist levels decide
    // what the car has and how it handles, so a client on a different local
    // config would be driving a different car on the same grid.
    netRules_ = rules;
    netRulesLocked_ = true;
    seasonDrs_ = (rules.seasonBits & net::kSeasonDrs) != 0;
    seasonAero_ = (rules.seasonBits & net::kSeasonActiveAero) != 0;
    seasonOverride_ = (rules.seasonBits & net::kSeasonOvertake) != 0;
    seasonErs_ = (rules.seasonBits & net::kSeasonErs) != 0;
    netRoster_ = rules.roster;
    if (const auto seat = net::SeatOf(netRoster_, localPlayerId_)) {
        netLocalSeat_.store(*seat, std::memory_order_relaxed);
    }
}

void Game::LanBeginClientRace() {
    // Client: same track (signature checked at join), only its own car spawned.
    const std::string mapName = netConfig_.mapName.empty() ? mapName_ : netConfig_.mapName;
    mapName_ = mapName;
    totalLaps_ = std::clamp<int>(netConfig_.rules.laps, 1, 99);
    netCarCount_ = std::max<std::uint8_t>(1, netConfig_.rules.roster.count);
    netRulesLocked_ = true;
    netRemoteFresh_.fill(false);
    StartRace();
    netInterp_ = std::make_unique<RemoteInterpolator>();
    netInterp_->SetDelayMs(
        std::clamp(config_.GetInt("net", "interp_buffer_ms", 40), 10, 150));
    netPredict_ = std::make_unique<LocalPredictor>();
    netPredict_->Reset(0);
    localTick_ = 0;
    localInputSeq_ = 0;
    // Corrections are absorbed visually over this window (§4.4); 0 snaps instead.
    clientCorrection_.SetTauMs(
        static_cast<float>(config_.GetInt("net", "correction_tau_ms", 80)));
    clientCorrection_.Reset();
    if (world_ && !racers_.empty()) {
        netProxies_ = std::make_unique<RemoteProxies>();
        netProxies_->Build(*world_, netCarCount_, netLocalSeat_.load(std::memory_order_relaxed),
                           LocalRacer().vehicle.Params());
        netProxies_->SetEnabled(true);
    }
}

// ------------------------------------------------------------- host world ---

void Game::LanBuildHostWorld(net::INetSession::HostWorld& out) {
    out.packet = net::SnapshotPacket{};
    out.packet.phase = static_cast<std::uint8_t>(
        phase_ == RacePhase::Countdown
            ? net::NetPhase::Countdown
            : (phase_ == RacePhase::Racing
                   ? net::NetPhase::Racing
                   : (phase_ == RacePhase::Finished ? net::NetPhase::Finished
                                                    : net::NetPhase::Lobby)));
    out.packet.phaseTimerCs = static_cast<std::uint16_t>(
        std::clamp(phaseTimer_, 0.f, 600.f) * 100.f);
    out.packet.raceClockMs =
        static_cast<std::uint32_t>(std::clamp(raceClock_, 0.f, 4.0e6f) * 1000.f);
    out.packet.carCount = static_cast<std::uint8_t>(std::min<int>(racers_.size(), net::kMaxCars));
    out.packet.hostTick = hostTick_;
    // Anchor at our own car so int16 offsets stay small.
    if (!racers_.empty()) {
        const Transform t = LocalRacer().vehicle.State().transform;
        out.packet.anchor[0] = t.position.x;
        out.packet.anchor[1] = t.position.y;
        out.packet.anchor[2] = t.position.z;
    }
    for (int i = 0; i < out.packet.carCount; ++i) {
        Racer& racer = racers_[static_cast<std::size_t>(i)];
        const VehicleState& state = racer.vehicle.State();
        Transform chassis = state.transform;
        if (world_) {
            if (const auto t = world_->GetTransform(racer.vehicle.Chassis())) chassis = *t;
        }
        // The wire carries the seat, not the local array index: that is what a
        // client matches against its own seat and what the host's input mailbox
        // is keyed by.
        ToCarNetState(static_cast<std::uint8_t>(racer.seat), chassis, state,
                      static_cast<std::uint8_t>(racer.lap),
                      static_cast<std::uint8_t>(std::clamp(racer.position, 0, 255)),
                      racer.finished, false, out.packet.cars[static_cast<std::size_t>(i)]);
        // The self block is addressed to the recipient, whose session slot is
        // its seat.
        const VehicleSnapshotState snap = racer.vehicle.SaveState();
        const auto seat = static_cast<std::size_t>(racer.seat);
        if (seat < static_cast<std::size_t>(net::kMaxCars)) {
            ToSelfState(snap, out.self[seat]);
            out.selfValid[seat] = 1;
            // Which input this car's state was just built from: the recipient
            // replays everything after it.
            out.appliedSeq[seat] = netAppliedSeq_[seat];
        }
    }
}

// ------------------------------------------------------------ host inputs ---

void Game::LanApplyRemoteInputs() {
    std::array<net::InputSample, net::kMaxCars> inputs;
    std::array<std::uint16_t, net::kMaxCars> seqs;
    std::array<bool, net::kMaxCars> fresh;
    {
        std::lock_guard lock(netInputMutex_);
        inputs = netRemoteInput_;
        seqs = netRemoteInputSeq_;
        fresh = netRemoteFresh_;
    }
    const std::uint8_t localSeat = netLocalSeat_.load(std::memory_order_relaxed);
    // One pass over the roster: the car for a peer is the car on its seat, so a
    // roster hole leaves a seat cold instead of feeding a neighbour's input to
    // the wrong car.
    for (int s = 0; s < netRoster_.count; ++s) {
        const std::uint8_t seat = netRoster_.entries[static_cast<std::size_t>(s)].seat;
        if (seat == localSeat) continue;   // our own car is driven by the mailbox
        Racer* racerPtr = RacerBySlot(seat);
        if (!racerPtr) continue;
        Racer& racer = *racerPtr;
        CarInput& in = racer.input;
        if (phase_ != RacePhase::Racing) {
            in = CarInput{};
            continue;
        }
        const auto slot = static_cast<std::size_t>(seat);
        if (!fresh[slot]) {
            // No packets for this peer: coast instead of running away.
            in.throttle = 0.f;
            in.brake = 0.6f;
            in.steer = 0.f;
            in.handbrake = false;
            in.reverse = false;
            in.drsZone = false;
            in.drsRequest = false;
            in.wingToggle = false;
            in.overtake = false;
            continue;
        }
        const net::InputSample& sample = inputs[slot];
        const float d = track_.ProjectDistance(racer.vehicle.State().transform.position,
                                              racer.lapDistance);
        const bool inZone = track_.InDrsZone(d);
        bool& latch = netDrsLatch_[slot];
        if (!inZone) {
            latch = false;
        } else if (sample.drsPress != lastDrsPress_[slot] && seasonDrs_) {
            latch = !latch;
        }
        lastDrsPress_[slot] = sample.drsPress;
        in.throttle = sample.throttle;
        in.brake = sample.brake;
        in.steer = sample.steer;
        in.handbrake = sample.handbrake;
        in.reverse = sample.reverse;
        in.drsZone = inZone;
        in.drsRequest = latch;
        in.wingToggle = seasonAero_ && sample.wingPress != lastWingPress_[slot];
        lastWingPress_[slot] = sample.wingPress;
        in.overtake = seasonOverride_ && sample.overtakePress != 0;
        // This is the input the tick about to run will step, so it is the rollback
        // landmark the client must replay from. Stale peers keep the previous
        // value: their car was driven by the coast input, not by a new sample.
        netAppliedSeq_[slot] = seqs[slot];
    }
}

// ------------------------------------------------------------ client path ---

double Game::LanRenderTimeMs() const {
    const double now = static_cast<double>(NowMs());
    return now - (netInterp_ ? netInterp_->DelayMs() : 40.0);
}

net::InputSample Game::LanSampleLocalInput() {
    net::InputSample s;
    s.throttle = rawInput_.throttle;
    s.brake = rawInput_.brake;
    s.steer = rawInput_.steer;
    s.handbrake = rawInput_.handbrake;
    s.reverse = rawInput_.reverse;
    if (rawInput_.drsToggle) ++netPressDrs_;
    if (rawInput_.wingModeToggle) ++netPressWing_;
    if (rawInput_.overtake) ++netPressOvertake_;
    s.drsPress = netPressDrs_;
    s.wingPress = netPressWing_;
    s.overtakePress = netPressOvertake_;
    return s;
}

void Game::LanClientSimTick(float frameDt) {
    if (racers_.empty() || !netSession_) return;
    Racer& player = LocalRacer();
    const std::uint8_t localSeat = netLocalSeat_.load(std::memory_order_relaxed);

    // Snapshot -> predictor mailbox (converted here so the sim thread only sees
    // engine types). The authority for our car is the car on our seat; taking
    // car 0 instead would snap us onto the host's car every snapshot.
    if (netPredict_ && netSession_) {
        if (const net::SnapshotPacket* snap = netSession_->LatestSnapshot()) {
            for (int i = 0; i < snap->carCount; ++i) {
                const net::CarNetState& car = snap->cars[static_cast<std::size_t>(i)];
                if (car.slot != localSeat) continue;
                netPredict_->PushAuthority(car, snap->hasSelf ? &snap->self : nullptr,
                                           snap->appliedInputSeq, snap->hostTick);
                break;
            }
        }
    }

    accumulator_ += frameDt;
    int guard = 0;
    while (accumulator_ >= fixedDt_ && guard++ < 8) {
        accumulator_ -= fixedDt_;
        ++localTick_;

        const bool racing = phase_ == RacePhase::Racing;
        if (netPredict_) {
            // Rollback to the last authority, replay our unconfirmed inputs up to
            // the tick that has already run, then simulate the current one below.
            ReconcileCorrection correction;
            if (netPredict_->Reconcile(player.vehicle, *world_, fixedDt_, localTick_ - 1,
                                       &correction)) {
                clientCorrection_.Push(correction);
                netCorrMaxM_ = std::max(netCorrMaxM_, Length(correction.position));
                ++netReconciles_;
                if (!correction.replayed) ++netSnaps_;
            }
        }

        CarInput in{};
        if (racing) {
            net::InputSample s;{std::lock_guard lock(netInputMutex_); s = netLocalSample_;}
            const float d = track_.ProjectDistance(player.vehicle.State().transform.position,
                                                   player.lapDistance);
            const bool inZone = track_.InDrsZone(d);
            if (!inZone) {
                clientDrsLatch_ = false;
            } else if (s.drsPress != netPressDrsSeen_ && seasonDrs_) {
                clientDrsLatch_ = !clientDrsLatch_;
            }
            netPressDrsSeen_ = s.drsPress;
            in.throttle = s.throttle;
            in.brake = s.brake;
            in.steer = s.steer;
            in.handbrake = s.handbrake;
            in.reverse = s.reverse;
            in.drsZone = inZone;
            in.drsRequest = clientDrsLatch_;
            in.wingToggle = seasonAero_ && s.wingPress != netPressWingSeen_;
            netPressWingSeen_ = s.wingPress;
            in.overtake = seasonOverride_ && s.overtakePress != 0;
        }
        player.input = in;
        player.vehicle.Update(in, fixedDt_);
        (*world_).Step(fixedDt_);
        if (netPredict_) netPredict_->Record(localTick_, localInputSeq_, in);

        // Remote cars block but are never pushed.
        if (netProxies_ && netInterp_) {
            netProxies_->Update(*world_, *netInterp_, LanRenderTimeMs(), netCarCount_);
        }

        // Local race bookkeeping (HUD only - results stay authoritative).
        if (racing) {
            const float d = track_.ProjectDistance(player.vehicle.State().transform.position,
                                                   player.lapDistance);
            const float prev = player.lapDistance;
            player.lapDistance = d;
            const float total = track_.TotalLength();
            if (total > 1.f && prev > total * 0.75f && d < total * 0.25f) {
                ++player.lap;
            }
        }
    }
    // The visual offset left by a rollback ages on wall-clock time, not on the
    // authority timeline: it exists only to hide the seam from the player.
    clientCorrection_.Decay(frameDt);
    if (phase_ == RacePhase::Countdown) {
        phaseTimer_ -= frameDt;
        if (phaseTimer_ <= 0.f) {
            phase_ = RacePhase::Racing;
            raceClock_ = 0.f;
        }
    } else if (phase_ == RacePhase::Racing) {
        raceClock_ += frameDt;
    }
    PublishSnapshot();
}

}  // namespace apex
