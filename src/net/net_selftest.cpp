// --net-selftest: protocol + session checks that run without a network stack
// and without a window. Exit code 0 = every check passed.

#include <chrono>
#include <cstdio>
#include <cstring>
#include <vector>

#include "net/discovery.h"
#include "net/protocol/wire.h"
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
    if (const auto n = EncodeHello(hello, buf)) {
        Check(*n == 27, "hello encodes to 27 B");
        const auto back = DecodeHello(std::span<const std::byte>(buf.data(), *n));
        if (!back) std::printf("    hello decode err=%d\n", static_cast<int>(back.error()));
        Check(back.has_value() && back->name == "PLAYER1" && back->buildSig == hello.buildSig,
              "hello round-trip");
    }

    WelcomePacket welcome;
    welcome.peerId = 3;
    welcome.slot = 3;
    welcome.sessionId = 9;
    welcome.rules.laps = 5;
    welcome.mapName = "test_circuit";
    if (const auto n = EncodeWelcome(welcome, buf)) {
        Check(*n == 60, "welcome encodes to 60 B");
        const auto back = DecodeWelcome(std::span<const std::byte>(buf.data(), *n));
        Check(back.has_value() && back->slot == 3 && back->rules.laps == 5 &&
                  back->mapName == "test_circuit",
              "welcome round-trip");
    }

    AnnouncePacket ad;
    ad.roomCode = 1234;
    ad.gamePort = 45998;
    ad.mapName = "test_circuit";
    ad.hostName = "HOST";
    if (const auto n = EncodeAnnounce(ad, buf)) {
        Check(*n == 62, "announce encodes to 62 B");
        const auto back = DecodeAnnounce(std::span<const std::byte>(buf.data(), *n));
        Check(back.has_value() && back->roomCode == 1234 && back->hostName == "HOST",
              "announce round-trip");
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
    hostCfg.roomCode = 1234;
    hostCfg.buildSig = 0xAABBCCDD;
    hostCfg.mapHash = 0x11223344;
    hostCfg.mapName = "test_circuit";
    hostCfg.rules.laps = 2;

    NetConfig clientCfg = hostCfg;
    clientCfg.playerName = "CLIENT";

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
    host->SetAppliedInputSeq(1, 3);
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
    host->SetAppliedInputSeq(client->LocalSlot(), 5);
    bool snapSeen = false;
    for (int i = 0; i < 200 && !snapSeen; ++i) {
        host->PublishSnapshot(world);
        pump(1);
        const SnapshotPacket* snap = client->LatestSnapshot();
        snapSeen = snap && snap->carCount == 2 && snap->hasSelf && snap->appliedInputSeq == 5;
    }
    Check(snapSeen, "snapshot + self block over real UDP");
}

}  // namespace

namespace apex {

int RunNetSelfTest() {
    std::printf("apex-rush net self-test\n");
    TestCodec();
    TestSessions();
    TestRealSockets();
    if (g_failures == 0) {
        std::printf("net self-test: OK\n");
        return 0;
    }
    std::printf("net self-test: %d FAILURE(S)\n", g_failures);
    return 1;
}

}  // namespace apex
