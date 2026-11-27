#pragma once

// Transport abstraction: the only thing the session layer knows about the
// network. Implementations (ENet / loopback / null) live in transport.cpp, and
// only that file may include <enet/enet.h> or <winsock2.h>.

#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <span>
#include <vector>

#include "net/protocol/wire.h"

namespace apex::net {

// ENet channel 0 carries high-frequency unreliable traffic, channel 1 the
// reliable control/event stream.
enum class Channel : std::uint8_t { State = 0, Control = 1 };
enum class Delivery : std::uint8_t { Unreliable, UnreliableSequenced, Reliable };

struct PeerId {
    static constexpr std::uint8_t kInvalidPeerValue = 0xFF;
    std::uint8_t value = kInvalidPeerValue;
    bool valid() const { return value != kInvalidPeerValue; }
    friend bool operator==(PeerId a, PeerId b) { return a.value == b.value; }
    friend bool operator!=(PeerId a, PeerId b) { return a.value != b.value; }
};

struct Endpoint {
    std::uint32_t ipv4 = 0;   // host byte order
    std::uint16_t port = 0;
};

struct PeerStats {
    std::uint32_t rttMs = 0;
    std::uint32_t rttVarianceMs = 0;
    std::uint8_t packetLossPct = 0;
    std::uint64_t bytesIn = 0;
    std::uint64_t bytesOut = 0;
};

enum class NetEventKind : std::uint8_t { Connected, Disconnected, TimedOut, Received };

struct NetEvent {
    NetEventKind kind = NetEventKind::Received;
    PeerId peer{};
    Channel channel = Channel::State;
    std::uint16_t sequence = 0;
    std::span<const std::byte> payload;   // valid until the next Poll()
};

class ITransport {
public:
    virtual ~ITransport() = default;

    // Host role: accept up to `maxPeers` connections on `bind`.
    virtual std::expected<void, NetError> Listen(const Endpoint& bind, int maxPeers) = 0;
    // Client role: connect to the host. Returns immediately; the connection
    // result arrives as a Connected/Disconnected event.
    virtual std::expected<void, NetError> Connect(const Endpoint& host) = 0;

    virtual void Close() = 0;

    virtual std::expected<void, NetError> Send(PeerId to, Channel ch, Delivery mode,
                                               std::span<const std::byte> packet) = 0;

    // Drains pending events. Main thread only.
    virtual std::span<const NetEvent> Poll(std::uint64_t nowMs) = 0;

    virtual PeerStats Stats(PeerId peer) const = 0;

    // Host: peers currently connected. Client: single entry (the host).
    virtual std::span<const PeerId> Peers() const = 0;

    virtual bool Listening() const = 0;
    virtual bool Connected() const = 0;
};

// --- factories ---------------------------------------------------------------

// Real UDP transport over ENet. `bindPort` 0 lets the OS choose.
std::unique_ptr<ITransport> MakeEnetTransport();

// In-process transport pair for tests: the two ends share one queue, delivered
// on the next Poll (zero latency, optional packet loss).
std::unique_ptr<ITransport> MakeLoopbackTransport(class LoopbackNet& net, bool isHost);

// Compiles everywhere, drops everything (WASM / single player).
std::unique_ptr<ITransport> MakeNullTransport();

// Shared plumbing for the loopback pair.
class LoopbackNet {
public:
    struct Item {
        bool toHost = false;
        Channel channel = Channel::State;
        std::uint16_t sequence = 0;
        std::vector<std::byte> bytes;
    };
    // Simulated loss percentage (0 = perfect link).
    int lossPct = 0;
    // Deterministic PRNG so runs are reproducible.
    std::uint32_t rng = 0x1234567u;
    std::vector<Item> toHost;
    std::vector<Item> toClient;
    std::uint32_t NextRandom();
};

}  // namespace apex::net
