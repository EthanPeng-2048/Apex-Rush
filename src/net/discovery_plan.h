#pragma once

// Discovery planning: which ports an instance may bind and which addresses one
// datagram has to be pushed at. Both are pure functions of their input so they
// can be checked by --net-selftest without opening a socket.

#include <algorithm>
#include <cstdint>
#include <span>
#include <vector>

namespace apex::net {

// Ports to try, in preference order: the configured port first (so a lone
// instance keeps the familiar one and old builds still find it), then the rest
// of the range. Empty when the span is zero.
inline std::vector<std::uint16_t> PortCandidates(std::uint16_t base, std::uint16_t span) {
    std::vector<std::uint16_t> ports;
    for (std::uint16_t i = 0; i < span; ++i) {
        const auto port = static_cast<std::uint16_t>(base + i);
        if (port < base) break;   // wrapped past 65535
        ports.push_back(port);
    }
    return ports;
}

// One entry per datagram to send. 255.255.255.255 alone is not enough: the
// kernel only emits it on the default-route interface, so a box with Wi-Fi and
// Ethernet subnets never reaches the other one. Every interface therefore
// contributes its broadcast and its own address (the unicast entry is what
// makes two instances on one machine see each other when broadcasts are
// filtered), and loopback closes the circle. Sorted + de-duplicated so a
// multi-homed interface does not double every packet; 0.0.0.0 is dropped.
inline std::vector<std::uint32_t> MergeTargets(std::span<const std::uint32_t> raw) {
    std::vector<std::uint32_t> out;
    out.reserve(raw.size());
    for (const std::uint32_t addr : raw) {
        if (addr != 0) out.push_back(addr);
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

}  // namespace apex::net
