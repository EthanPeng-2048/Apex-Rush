#pragma once

// LAN room discovery (N-01/N-02/N-03): the host advertises, clients listen and
// may also actively query. Kept behind an interface so tests and WASM can use
// the null/static variants instead of real sockets.

#include <cstdint>
#include <expected>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "net/protocol/wire.h"
#include "net/transport.h"

namespace apex::net {

struct RoomInfo {
    AnnouncePacket announce;
    Endpoint host{};
    std::uint64_t lastSeenMs = 0;
};

// What the socket layer actually achieved. Discovery used to fail silently -
// a bind() that the OS refused looked exactly like "nobody is hosting", which
// sends the player hunting for a firewall that was never the problem. The UI
// shows these numbers so "no rooms" can be told apart from "no socket".
struct DiscoveryStatus {
    std::uint16_t boundPort = 0;    // 0 = no socket at all
    bool multicast = false;         // joined the rendezvous group
    bool fallbackPort = false;      // the whole range was taken: ephemeral port
    std::uint32_t sent = 0;         // datagrams handed to sendto()
    std::uint32_t received = 0;     // announcements accepted
    std::uint32_t sendFailures = 0;
};

// How much of the network the discovery layer is allowed to use. Both knobs
// exist for the one case the defaults cannot cover: a machine where group
// traffic is filtered outright.
struct DiscoveryOptions {
    std::uint16_t portSpan = kDiscoveryPortSpan;
    bool multicast = true;
};

class IRoomDiscovery {
public:
    virtual ~IRoomDiscovery() = default;

    // Host side: start/refresh the 1 Hz broadcast (sent from Poll()).
    virtual std::expected<void, NetError> Advertise(const AnnouncePacket& ad) = 0;
    virtual void StopAdvertising() = 0;

    // Client side: listen for announcements and answer to queries.
    virtual std::expected<void, NetError> BeginSearch() = 0;
    virtual void SendQuery(std::uint16_t roomCode) = 0;

    // Pump sockets / timers. Main thread, once per frame.
    virtual void Poll(std::uint64_t nowMs) = 0;

    // Live rooms (announcements seen within the TTL). Filtering by room code is
    // the caller's job (it also drives the UI).
    virtual std::span<const RoomInfo> Rooms() const = 0;

    // What the socket layer actually achieved, for the room browser's
    // diagnostics line and stderr.
    virtual DiscoveryStatus Status() const = 0;
};

std::unique_ptr<IRoomDiscovery> MakeUdpDiscovery(std::uint16_t discoveryPort,
                                                DiscoveryOptions opts = {});
std::unique_ptr<IRoomDiscovery> MakeNullDiscovery();

}  // namespace apex::net
