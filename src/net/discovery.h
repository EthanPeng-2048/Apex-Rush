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
};

std::unique_ptr<IRoomDiscovery> MakeUdpDiscovery(std::uint16_t discoveryPort);
std::unique_ptr<IRoomDiscovery> MakeNullDiscovery();

}  // namespace apex::net
