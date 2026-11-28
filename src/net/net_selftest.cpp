// --net-selftest: protocol + session checks that run without a network stack
// and without a window. Exit code 0 = every check passed.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

#include "net/discovery.h"
#include "net/discovery_plan.h"
#include "net/identity.h"
#include "net/protocol/wire.h"
#include "net/roster.h"
#include "net/session.h"
#include "net/transport.h"

namespace {

int g_failures = 0;

void Check(bool ok, const char* what) {
    if (!ok) {
        std::printf("  FAIL %s\n", what);
        ++g_failures;
    }
}

template <typename T>
void CheckRoundTrip(const char* name, const std::vector<std::byte>& bytes, std::size_t expectedSize) {
    if (bytes.size() != expectedSize) {
        std::printf("  FAIL %s size %zu (expected %zu)\n", name, bytes.size(), expectedSize);
        ++g_failures;
    }
}

std::vector<std::byte> ToBytes(std::span<const std::byte> s) { return {s.begin(), s.end()}; }

void TestCodec() {
    using namespace apex::net;
    std::printf("[codec]\n");
    std::array<std::byte, kMaxDatagram> buf{};

    // Input: 22 bytes, round-trips quantized values.
    InputPacket in;
    in.firstSeq = 4242;
    in.count = 3;
    for (int i = 0; i < 3; ++i) {
        in.frames[static_cast<std::size_t>(i)].throttle = static_cast<std::uint8_t>(100 + i);
        in.frames[static_cast<std::size_t>(i)].brake = 20;
        in.frames[static_cast<std::size_t>(i)].steer = static_cast<std::int8_t>(-30 + i);
        in.frames[static_cast<std::size_t>(i)].buttons = kBtnHandbrake;
    }
    in.drsPress = 7;
    in.wingPress = 3;
    in.overtakePress = 1;
    in.ackSnapshotSeq = 99;
    const auto inSize = EncodeInput(in, buf);
    Check(inSize.has_value() && *inSize == 22, "input encodes to 22 B");
    if (inSize) {
        const auto back = DecodeInput(std::span<const std::byte>(buf.data(), *inSize));
        Check(back.has_value(), "input decodes");
        if (back) {
            Check(back->firstSeq == in.firstSeq && back->count == 3, "input seq/count");
            bool framesOk = true;
            for (int i = 0; i < 3; ++i) {
                const auto& a = in.frames[static_cast<std::size_t>(i)];
                const auto& b = back->frames[static_cast<std::size_t>(i)];
                framesOk = framesOk && a.throttle == b.throttle && a.brake == b.brake &&
                           a.steer == b.steer && a.buttons == b.buttons;
            }
            Check(framesOk, "input frames round-trip");
            Check(back->drsPress == 7 && back->ackSnapshotSeq == 99, "input counters");
        }
        // Truncation and trailing garbage must be rejected.
        Check(!DecodeInput(std::span<const std::byte>(buf.data(), 10)).has_value(),
              "input truncation rejected");
        buf[*inSize] = std::byte{0};
        Check(!DecodeInput(std::span<const std::byte>(buf.data(), *inSize + 1)).has_value(),
              "input trailing byte rejected");
        buf[*inSize] = std::byte{0};
    }

    // Snapshot: 345 B with 8 cars + self block.
    SnapshotPacket snap;
    snap.hostTick = 123456;
    snap.snapSeq = 77;
    snap.phase = static_cast<std::uint8_t>(NetPhase::Racing);
    snap.phaseTimerCs = 350;
    snap.raceClockMs = 65000;
    snap.carCount = 8;
    snap.anchor[0] = 100.f;
    snap.anchor[1] = 2.f;
    snap.anchor[2] = -50.f;
    snap.appliedInputSeq = 321;
    for (int i = 0; i < 8; ++i) {
        auto& c = snap.cars[static_cast<std::size_t>(i)];
        c.slot = static_cast<std::uint8_t>(i);
        c.pos[0] = 100.f + 3.f * i;
        c.pos[1] = 2.5f;
        c.pos[2] = -50.f - 2.f * i;
        c.yaw = 0.5f + 0.1f * i;
        c.pitch = -0.02f;
        c.roll = 0.03f;
        c.vel[0] = 20.f;
        c.vel[1] = 0.5f;
        c.vel[2] = 30.f;
        c.yawRate = 0.4f;
        c.steerDeg = 12.f;
        c.speedKmh = 210.f;
        c.rpm = 11000.f;
        c.gear = 7;
        c.ersCharge = 0.42f;
        c.lap = static_cast<std::uint8_t>(i + 1);
        c.position = static_cast<std::uint8_t>(i + 1);
        c.wheelTravel[0] = 0.25f;
        c.wheelTravel[1] = 0.5f;
        c.wheelTravel[2] = 0.75f;
        c.wheelTravel[3] = 1.f;
        c.groundedMask = 0x0F;
    }
    snap.hasSelf = true;
    snap.self.wear[0] = 0.3f;
    snap.self.tempC[1] = 95.f;
    snap.self.slipRatio[2] = 0.12f;
    snap.self.wheelSpinRate[3] = 88.f;
    snap.self.rngState = 0xDEADBEEF;
    const auto snapSize = EncodeSnapshot(snap, buf);
    Check(snapSize.has_value() && *snapSize == 345, "snapshot encodes to 345 B");
    if (snapSize) {
        const auto back = DecodeSnapshot(std::span<const std::byte>(buf.data(), *snapSize));
        if (!back) std::printf("    snap decode err=%d\n", static_cast<int>(back.error()));
        Check(back.has_value(), "snapshot decodes");
        if (back) {
            Check(back->hostTick == snap.hostTick && back->snapSeq == snap.snapSeq,
                  "snapshot header");
            Check(back->carCount == 8 && back->hasSelf, "snapshot car count / self block");
            const auto& a = snap.cars[3];
            const auto& b = back->cars[3];
            Check(std::abs(a.pos[0] - b.pos[0]) < 0.04f && std::abs(a.pos[2] - b.pos[2]) < 0.04f,
                  "snapshot position quantization < 4 cm");
            Check(std::abs(a.yaw - b.yaw) < 0.001f, "snapshot yaw");
            Check(std::abs(a.speedKmh - b.speedKmh) < 0.06f, "snapshot speed");
            Check(back->cars[3].gear == 7 && std::abs(back->cars[3].ersCharge - 0.42f) < 0.01f,
                  "snapshot gear / ers");
            Check(std::abs(back->self.wheelSpinRate[3] - 88.f) < 0.1f, "self wheel spin");
            Check(back->self.rngState == 0xDEADBEEF, "self rng state");
        }
        Check(!DecodeSnapshot(std::span<const std::byte>(buf.data(), *snapSize - 1)).has_value(),
              "snapshot truncation rejected");
    }
    Check(!EncodeSnapshot(snap, std::span<std::byte>(buf.data(), 64)).has_value(),
          "snapshot overflow rejected");

    // Control messages.
    HelloPacket hello;
    hello.buildSig = 0xABCD1234u;
    hello.mapHash = 0x11112222u;
    hello.name = "PLAYER1";
    hello.playerId = apex::MakePlayerId("PLAYER1");
    if (const auto n = EncodeHello(hello, buf)) {
        Check(*n == 35, "hello encodes to 35 B");
        const auto back = DecodeHello(std::span<const std::byte>(buf.data(), *n));
        if (!back) std::printf("    hello decode err=%d\n", static_cast<int>(back.error()));
        Check(back.has_value() && back->name == "PLAYER1" && back->buildSig == hello.buildSig,
              "hello round-trip");
        Check(back.has_value() && back->playerId == hello.playerId, "hello carries playerId");
    }

    WelcomePacket welcome;
    welcome.peerId = 3;
    welcome.slot = 3;
    welcome.sessionId = 9;
    welcome.rules.laps = 5;
    welcome.mapName = "test_circuit";
    welcome.playerId = 0x1122334455667788ull;
    welcome.rules.roster.count = 2;
    welcome.rules.roster.entries[0].playerId = 0x10;
    welcome.rules.roster.entries[0].seat = 0;
    welcome.rules.roster.entries[0].name = "HOST";
    welcome.rules.roster.entries[1].playerId = 0x20;
    welcome.rules.roster.entries[1].seat = 1;
    welcome.rules.roster.entries[1].name = "CLIENT";
    if (const auto n = EncodeWelcome(welcome, buf)) {
        Check(*n == 269, "welcome encodes to 269 B (rules carry the roster)");
        const auto back = DecodeWelcome(std::span<const std::byte>(buf.data(), *n));
        Check(back.has_value() && back->slot == 3 && back->rules.laps == 5 &&
                  back->mapName == "test_circuit",
              "welcome round-trip");
        Check(back.has_value() && back->playerId == welcome.playerId, "welcome carries playerId");
        Check(back.has_value() && back->rules.roster.count == 2 &&
                  back->rules.roster.entries[1].seat == 1 &&
                  back->rules.roster.entries[1].name == "CLIENT" &&
                  back->rules.roster.entries[1].playerId == 0x20,
              "welcome carries the roster");
        Check(!DecodeWelcome(std::span<const std::byte>(buf.data(), *n - 1)).has_value(),
              "welcome truncation rejected");
    }

    // Rules + roster in the RaceStart the client seats itself from.
    RaceStartPacket start;
    start.startHostTick = 42;
    start.countdownCs = 350;
    start.rules = welcome.rules;
    if (const auto n = EncodeRaceStart(start, buf)) {
        Check(*n == 219, "race start encodes to 219 B");
        const auto back = DecodeRaceStart(std::span<const std::byte>(buf.data(), *n));
        Check(back.has_value() && back->startHostTick == 42 &&
                  back->rules.roster.entries[0].name == "HOST",
              "race start round-trip with roster");
    }

    AnnouncePacket ad;
    ad.roomCode = 1234;
    ad.gamePort = 45998;
    ad.mapName = "test_circuit";
    ad.hostName = "HOST";
    ad.playerId = 0xDEADBEEFCAFEull;
    if (const auto n = EncodeAnnounce(ad, buf)) {
        Check(*n == 70, "announce encodes to 70 B");
        const auto back = DecodeAnnounce(std::span<const std::byte>(buf.data(), *n));
        Check(back.has_value() && back->roomCode == 1234 && back->hostName == "HOST",
              "announce round-trip");
        Check(back.has_value() && back->playerId == ad.playerId, "announce carries playerId");
    }

    QueryPacket query;
    query.roomCode = 4321;
    if (const auto n = EncodeQuery(query, buf)) {
        Check(*n == 8, "query encodes to 8 B");
        const auto back = DecodeQuery(std::span<const std::byte>(buf.data(), *n));
        Check(back.has_value() && back->roomCode == 4321, "query round-trip");
    }

    // Garbage in, error out (never a crash).
    std::vector<std::byte> junk(64, std::byte{0x7F});
    Check(!DecodeSnapshot(junk).has_value(), "garbage snapshot rejected");
    Check(!PeekMessageType(std::span<const std::byte>(junk.data(), 1)).has_value(),
          "short peek rejected");
}

// Identity + seat model: the id must not depend on spelling or spacing, seats
// must be the rank of the id (so the host is not "car 0" by construction), and a
// roster hole must not shift anybody onto someone else's grid box.
void TestIdentityAndSeats() {
    using namespace apex;
    using namespace apex::net;
    std::printf("[identity]\n");

    const PlayerId pilot = MakePlayerId("Pilot");
    Check(pilot != kInvalidPlayerId, "name hashes to an id");
    Check(pilot == MakePlayerId("  PILOT "), "id ignores case and surrounding spaces");
    Check(MakePlayerId("") == kInvalidPlayerId, "empty name has no id");
    Check(MakePlayerId("pilot", 1) != pilot, "variant salt rehashes to a different id");
    Check(MakePlayerId("pilot", 1) == MakePlayerId("PILOT", 1), "variant rehash stays stable");
    Check(TrimPlayerName("  Pi lot  ") == "Pi lot", "display name keeps its spelling");

    // Seats from ids, host included. Arrival slots {1,3} with a hole at 2 must
    // still produce seats 0 and 1 — dense, and in id order, not slot order.
    const PlayerId hostId = MakePlayerId("host");
    const PlayerId aId = MakePlayerId("alpha");
    const PlayerId bId = MakePlayerId("beta");
    const RosterCandidate candidates[] = {
        {.slot = 0, .playerId = hostId, .name = "host"},
        {.slot = 3, .playerId = bId, .name = "beta"},
        {.slot = 1, .playerId = aId, .name = "alpha"},
    };
    const Roster roster = BuildRoster(candidates);
    Check(roster.count == 3, "roster seats every candidate");
    // Sorted by playerId ascending, wherever each one sat in the session.
    PlayerId sorted[3] = {hostId, aId, bId};
    std::sort(sorted, sorted + 3);
    bool orderOk = true;
    for (int i = 0; i < 3; ++i) {
        orderOk = orderOk && roster.entries[static_cast<std::size_t>(i)].playerId == sorted[i] &&
                  roster.entries[static_cast<std::size_t>(i)].seat == i;
    }
    Check(orderOk, "seats are dense ranks of playerId (roster hole tolerated)");

    const auto hostSeat = SeatOf(roster, hostId);
    Check(hostSeat.has_value() && (*hostSeat == 0 || *hostSeat == 1 || *hostSeat == 2),
          "host is seated like anybody else");
    Check(SeatOf(roster, MakePlayerId("stranger")) == std::nullopt,
          "unseated player has no seat");

    // A candidate without an id (hashed off an empty name) is skipped, not
    // seated on the first box.
    const RosterCandidate noId[] = {{.slot = 0, .playerId = kInvalidPlayerId, .name = ""}};
    Check(BuildRoster(noId).count == 0, "nameless candidate is not seated");
}

void TestSessions() {
    using namespace apex::net;
    std::printf("[session]\n");

    LoopbackNet net;
    auto hostTransport = MakeLoopbackTransport(net, true);
    auto clientTransport = MakeLoopbackTransport(net, false);
    auto hostDiscovery = MakeNullDiscovery();
    auto clientDiscovery = MakeNullDiscovery();

    NetConfig hostCfg;
    hostCfg.playerName = "HOST";
    hostCfg.playerId = apex::MakePlayerId("HOST");
    hostCfg.roomCode = 1234;
    hostCfg.buildSig = 0xAABBCCDD;
    hostCfg.mapHash = 0x11223344;
    hostCfg.mapName = "test_circuit";
    hostCfg.rules.laps = 2;

    NetConfig clientCfg = hostCfg;
    clientCfg.playerName = "CLIENT";
    clientCfg.playerId = apex::MakePlayerId("CLIENT");

    auto host = CreateHostSession(hostCfg, *hostTransport, *hostDiscovery);
    Endpoint ep;
    auto client = CreateClientSession(clientCfg, *clientTransport, *clientDiscovery, ep);

    std::uint64_t now = 1000;
    const auto step = [&](int frames) {
        for (int i = 0; i < frames; ++i) {
            now += 16;
            host->Tick(now);
            client->Tick(now);
            (void)host->DrainEvents();
            (void)client->DrainEvents();
        }
    };

    std::vector<SessionEvent> clientEvents;
    const auto stepCollect = [&](int frames) {
        for (int i = 0; i < frames; ++i) {
            now += 16;
            host->Tick(now);
            client->Tick(now);
            (void)host->DrainEvents();
            for (const SessionEvent& ev : client->DrainEvents()) clientEvents.push_back(ev);
        }
    };
    stepCollect(20);
    Check(client->State() == SessionState::Lobby, "client reaches Lobby after Welcome");
    Check(client->LocalSlot() >= 1, "client assigned a non-zero slot");
    Check(host->LocalSlot() == 0, "host is slot 0");

    stepCollect(20);
    bool sawLobby = false;
    for (const SessionEvent& ev : clientEvents) {
        if (ev.kind == SessionEventKind::LobbyChanged && ev.lobby.size() == 2) sawLobby = true;
    }
    Check(sawLobby, "client sees a 2-player lobby");

    // Client input reaches the host.
    InputSample sample;
    sample.throttle = 0.75f;
    sample.steer = -0.5f;
    client->SubmitLocalInput(sample);
    // The client only sends on its input interval; give it a few frames.
    step(10);
    const InputSample received = host->LatestInput(1);
    Check(std::abs(received.throttle - 0.75f) < 0.01f, "host receives client throttle");
    Check(std::abs(received.steer + 0.5f) < 0.02f, "host receives client steering");
    // The rollback landmark needs both numbers: the session that stamps the
    // sample and the host report that echoes it back with the state.
    Check(client->LocalInputSeq() == 1, "client stamps its own input sequence");
    Check(host->LatestInputSeq(1) == 1, "host reports the sequence it received");

    // Host publishes a snapshot; the client must receive it with its own self
    // block and an applied-input sequence.
    INetSession::HostWorld world;
    world.packet.phase = static_cast<std::uint8_t>(NetPhase::Racing);
    world.packet.carCount = 2;
    world.packet.hostTick = 500;
    world.packet.cars[0].slot = 0;
    world.packet.cars[1].slot = 1;
    world.packet.cars[1].lap = 1;
    world.packet.cars[1].position = 2;
    world.selfValid[1] = 1;
    world.self[1].slipRatio[0] = 0.05f;
    world.appliedSeq[1] = 3;
    for (int i = 0; i < 8; ++i) {
        host->PublishSnapshot(world);
        step(2);
    }
    const SnapshotPacket* snap = client->LatestSnapshot();
    Check(snap != nullptr, "client received a snapshot");
    if (snap) {
        Check(snap->carCount == 2, "snapshot car count");
        Check(snap->hasSelf, "snapshot carries the client's self block");
        Check(snap->appliedInputSeq == 3, "applied input seq reaches the client");
        Check(snap->cars[1].lap == 1 && snap->cars[1].position == 2, "authority race fields");
    }

    // --- roster freeze: seats, not arrival slots ---------------------------
    host->SetRules(hostCfg.rules);
    host->FreezeRoster();
    const RulesPacket& frozen = host->Rules();
    Check(frozen.roster.count == 2, "host seats every joined player");
    const auto clientSeat = apex::net::SeatOf(frozen.roster, clientCfg.playerId);
    Check(clientSeat.has_value(), "client appears in the frozen roster");
    host->StartRace(350);
    stepCollect(4);
    Check(clientSeat.has_value() && client->LocalSlot() == *clientSeat,
          "client takes its seat from the roster, not from its arrival slot");

    // Duplicate identity: a second player claiming an id the host already
    // seated is refused instead of sharing its car.
    {
        LoopbackNet net3;
        auto hostTransport3 = MakeLoopbackTransport(net3, true);
        auto clientTransport3 = MakeLoopbackTransport(net3, false);
        auto hostDiscovery3 = MakeNullDiscovery();
        auto clientDiscovery3 = MakeNullDiscovery();
        NetConfig dupHost = hostCfg;
        NetConfig dupClient = hostCfg;   // same name -> same playerId
        auto host3 = CreateHostSession(dupHost, *hostTransport3, *hostDiscovery3);
        auto client3 = CreateClientSession(dupClient, *clientTransport3, *clientDiscovery3, ep);
        bool duplicate = false;
        for (int i = 0; i < 40; ++i) {
            now += 16;
            host3->Tick(now);
            client3->Tick(now);
            (void)host3->DrainEvents();
            for (const SessionEvent& ev : client3->DrainEvents()) {
                if (ev.kind == SessionEventKind::JoinRejected &&
                    ev.reason == RejectReason::DuplicatePeer) {
                    duplicate = true;
                }
            }
        }
        Check(duplicate, "duplicate playerId rejected");
    }

    // Reject path: wrong build signature.
    LoopbackNet net2;
    auto hostTransport2 = MakeLoopbackTransport(net2, true);
    auto clientTransport2 = MakeLoopbackTransport(net2, false);
    auto hostDiscovery2 = MakeNullDiscovery();
    auto clientDiscovery2 = MakeNullDiscovery();
    NetConfig badClient = clientCfg;
    badClient.buildSig = 0xDEADBEEF;
    auto host2 = CreateHostSession(hostCfg, *hostTransport2, *hostDiscovery2);
    auto client2 = CreateClientSession(badClient, *clientTransport2, *clientDiscovery2, ep);
    bool rejected = false;
    for (int i = 0; i < 40; ++i) {
        now += 16;
        host2->Tick(now);
        client2->Tick(now);
        (void)host2->DrainEvents();
        for (const SessionEvent& ev : client2->DrainEvents()) {
            if (ev.kind == SessionEventKind::JoinRejected &&
                ev.reason == RejectReason::BuildMismatch) {
                rejected = true;
            }
        }
    }
    Check(rejected, "build mismatch rejected");
    Check(client2->State() == SessionState::Closed, "rejected client closes");
}

void TestRealSockets() {
    using namespace apex::net;
    std::printf("[real enet sockets]\n");
    auto hostTransport = MakeEnetTransport();
    auto clientTransport = MakeEnetTransport();
    if (!hostTransport || !clientTransport) {
        std::printf("  SKIP no enet\n");
        return;
    }
    auto hostDiscovery = MakeNullDiscovery();
    auto clientDiscovery = MakeNullDiscovery();

    NetConfig hostCfg;
    hostCfg.playerName = "HOST";
    hostCfg.roomCode = 4321;
    hostCfg.gamePort = 51232;
    hostCfg.buildSig = 0x1234ABCD;
    hostCfg.mapHash = 0x55667788;
    hostCfg.mapName = "test_circuit";
    hostCfg.rules.laps = 3;
    NetConfig clientCfg = hostCfg;
    clientCfg.playerName = "CLIENT";

    auto host = CreateHostSession(hostCfg, *hostTransport, *hostDiscovery);
    Endpoint ep;
    ep.ipv4 = 0x7F000001u;
    ep.port = hostCfg.gamePort;
    auto client = CreateClientSession(clientCfg, *clientTransport, *clientDiscovery, ep);

    std::uint64_t now = 1000;
    const auto pump = [&](int frames) {
        for (int i = 0; i < frames; ++i) {
            now += 16;
            host->Tick(now);
            client->Tick(now);
            (void)host->DrainEvents();
            (void)client->DrainEvents();
        }
    };

    bool joined = false;
    for (int i = 0; i < 300 && !joined; ++i) {
        pump(1);
        joined = client->State() == SessionState::Lobby;
        if (i % 60 == 0) {
            std::printf("    f=%d hostState=%d clientState=%d\n", i,
                        static_cast<int>(host->State()), static_cast<int>(client->State()));
        }
    }
    Check(joined, "client joined over real UDP");
    Check(client->LocalSlot() >= 1, "slot assigned over real UDP");

    // Inputs travel client -> host.
    InputSample sample;
    sample.throttle = 1.f;
    sample.steer = 0.25f;
    client->SubmitLocalInput(sample);
    bool inputSeen = false;
    for (int i = 0; i < 120 && !inputSeen; ++i) {
        pump(1);
        inputSeen = host->HasFreshInput(client->LocalSlot()) &&
                    host->LatestInput(client->LocalSlot()).throttle > 0.9f;
    }
    Check(inputSeen, "host receives input over real UDP");

    // Snapshots travel host -> client.
    INetSession::HostWorld world;
    world.packet.phase = static_cast<std::uint8_t>(NetPhase::Racing);
    world.packet.carCount = 2;
    world.packet.cars[0].slot = 0;
    world.packet.cars[1].slot = 1;
    world.packet.cars[1].lap = 2;
    world.selfValid[1] = 1;
    world.appliedSeq[client->LocalSlot()] = 5;
    bool snapSeen = false;
    for (int i = 0; i < 200 && !snapSeen; ++i) {
        host->PublishSnapshot(world);
        pump(1);
        const SnapshotPacket* snap = client->LatestSnapshot();
        snapSeen = snap && snap->carCount == 2 && snap->hasSelf && snap->appliedInputSeq == 5;
    }
    Check(snapSeen, "snapshot + self block over real UDP");
}

void TestDiscoveryPlan() {
    using namespace apex::net;
    std::printf("[discovery plan]\n");

    // Port preference: configured port first, then the rest of the range.
    const std::vector<std::uint16_t> ports = PortCandidates(51231, 4);
    Check(ports.size() == 4, "port range size");
    Check(ports.front() == 51231 && ports.back() == 51234, "port range bounds");
    Check(std::is_sorted(ports.begin(), ports.end()), "port range ascending");
    Check(PortCandidates(51231, 0).empty(), "empty span yields no ports");
    // A range that would wrap past 65535 stops there instead of wrapping onto
    // low ports nobody is listening on.
    Check(PortCandidates(65534, 4).size() == 2, "port range clamps at 65535");

    // Targets: zero dropped, duplicates collapsed, one entry per datagram.
    const std::uint32_t raw[] = {0xFFFFFFFFu, 0xC0A801FFu, 0xC0A801FFu, 0, 0x7F000001u,
                                 0xC0A80166u};
    const std::vector<std::uint32_t> merged = MergeTargets(raw);
    Check(merged.size() == 4, "targets de-duplicated");
    Check(std::is_sorted(merged.begin(), merged.end()), "targets sorted");
    Check(std::find(merged.begin(), merged.end(), 0u) == merged.end(), "0.0.0.0 dropped");
    Check(std::find(merged.begin(), merged.end(), 0xFFFFFFFFu) != merged.end(),
          "limited broadcast kept");
    Check(std::find(merged.begin(), merged.end(), 0x7F000001u) != merged.end(),
          "loopback kept");
}

// The discovery layer end to end over real sockets, on one machine: the second
// instance must not be blocked by the first (that is the whole point of the
// port range) and must see the room within a couple of announce intervals.
void TestRealDiscovery() {
    using namespace apex::net;
    std::printf("[real discovery]\n");

    auto host = MakeUdpDiscovery(kDiscoveryPortDefault);
    auto searcher = MakeUdpDiscovery(kDiscoveryPortDefault);
    Check(host->BeginSearch().has_value(), "host discovery socket");
    Check(searcher->BeginSearch().has_value(), "searcher discovery socket");
    Check(host->Status().boundPort != 0, "host bound a port");
    Check(searcher->Status().boundPort != 0, "searcher bound a port");
    const auto inRange = [](const DiscoveryStatus& s) {
        return s.fallbackPort || (s.boundPort >= kDiscoveryPortDefault &&
                                  s.boundPort < kDiscoveryPortDefault + kDiscoveryPortSpan);
    };
    Check(inRange(host->Status()) && inRange(searcher->Status()), "ports come from the range");

    AnnouncePacket ad;
    ad.roomCode = 4321;
    ad.gamePort = kGamePortDefault;
    ad.mapName = "test_circuit";
    ad.hostName = "SELFTEST-HOST";
    ad.playerId = apex::MakePlayerId("SELFTEST-HOST");
    Check(host->Advertise(ad).has_value(), "host advertising");
    searcher->SendQuery(ad.roomCode);

    using clock = std::chrono::steady_clock;
    const auto start = clock::now();
    bool found = false;
    while (!found && std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - start)
                            .count() < 4000) {
        const auto now = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(clock::now().time_since_epoch())
                .count());
        host->Poll(now);
        searcher->Poll(now);
        for (const RoomInfo& room : searcher->Rooms()) {
            if (room.announce.roomCode != ad.roomCode) continue;
            if (room.announce.playerId != ad.playerId) continue;
            found = room.host.ipv4 != 0 && room.host.port == ad.gamePort;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    Check(found, "announce reaches a second instance on this machine");
    Check(searcher->Status().received > 0, "searcher counted an announcement");
    // The room must be one row, not one per interface it arrived on.
    const auto rooms = searcher->Rooms();
    const auto dupes = std::count_if(rooms.begin(), rooms.end(), [&](const RoomInfo& r) {
        return r.announce.roomCode == ad.roomCode && r.announce.playerId == ad.playerId;
    });
    Check(dupes == 1, "one row per room despite the multi-target fan-out");
    host->StopAdvertising();
}

}  // namespace

namespace apex {

int RunNetSelfTest() {
    std::printf("apex-rush net self-test\n");
    TestCodec();
    TestDiscoveryPlan();
    TestRealDiscovery();
    TestIdentityAndSeats();
    TestSessions();
    TestRealSockets();
    if (g_failures == 0) {
        std::printf("net self-test: OK\n");
        return 0;
    }
    std::printf("net self-test: %d FAILURE(S)\n", g_failures);
    return 1;
}

int RunDiscoveryProbe(int seconds, int roomCode) {
    using namespace apex::net;
    using clock = std::chrono::steady_clock;

    auto disco = MakeUdpDiscovery(kDiscoveryPortDefault);
    if (!disco->BeginSearch().has_value()) {
        std::printf("probe: cannot open a discovery socket\n");
        return 1;
    }
    const DiscoveryStatus opened = disco->Status();
    std::printf("probe: bound udp %u%s, multicast %s, range %u-%u\n",
                static_cast<unsigned>(opened.boundPort),
                opened.fallbackPort ? " (range busy, ephemeral)" : "",
                opened.multicast ? "joined" : "off",
                static_cast<unsigned>(kDiscoveryPortDefault),
                static_cast<unsigned>(kDiscoveryPortDefault + kDiscoveryPortSpan - 1));

    if (roomCode > 0) {
        AnnouncePacket ad;
        ad.roomCode = static_cast<std::uint16_t>(roomCode);
        ad.gamePort = kGamePortDefault;
        ad.mapName = "probe";
        ad.hostName = "PROBE-HOST";
        ad.playerId = apex::MakePlayerId("PROBE-HOST");
        (void)disco->Advertise(ad);
        std::printf("probe: advertising room %04d for %d s\n", roomCode, seconds);
    } else {
        std::printf("probe: searching for %d s\n", seconds);
    }

    std::vector<PlayerId> seen;
    std::uint64_t nextQueryMs = 0;
    bool found = false;
    const auto start = clock::now();
    for (;;) {
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            clock::now() - start);
        if (elapsed.count() >= static_cast<long long>(seconds) * 1000) break;
        const auto now = static_cast<std::uint64_t>(elapsed.count());
        if (now >= nextQueryMs) {
            nextQueryMs = now + 1000;
            disco->SendQuery(static_cast<std::uint16_t>(std::max(roomCode, 0)));
        }
        disco->Poll(now);
        for (const RoomInfo& room : disco->Rooms()) {
            if (std::find(seen.begin(), seen.end(), room.announce.playerId) != seen.end()) continue;
            seen.push_back(room.announce.playerId);
            found = true;
            std::printf("probe: room %04u '%s' map=%s %u/%u at %u.%u.%u.%u:%u\n",
                        static_cast<unsigned>(room.announce.roomCode),
                        room.announce.hostName.c_str(), room.announce.mapName.c_str(),
                        static_cast<unsigned>(room.announce.players),
                        static_cast<unsigned>(room.announce.maxPlayers),
                        (room.host.ipv4 >> 24) & 0xFF, (room.host.ipv4 >> 16) & 0xFF,
                        (room.host.ipv4 >> 8) & 0xFF, room.host.ipv4 & 0xFF,
                        static_cast<unsigned>(room.host.port));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    const DiscoveryStatus end = disco->Status();
    std::printf("probe: sent %u, received %u, failures %u -> %s\n", end.sent, end.received,
                end.sendFailures, found ? "rooms found" : "NO ROOM SEEN");
    disco->StopAdvertising();
    return found ? 0 : 2;
}

}  // namespace apex
