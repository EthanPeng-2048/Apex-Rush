// UDP room discovery. Raw sockets (Winsock / BSD) are confined to this file.
//
// Two channels carry the same announce, because neither is reliable on its own:
//   * multicast to the rendezvous group on the base port - the only UDP mode
//     where every listener on a host gets its own copy, so several instances
//     can run side by side;
//   * broadcast fan-out across every port of the range on every interface -
//     covers networks that filter group traffic, and covers an instance that
//     lost the race for the base port and had to settle for another one.

#include "net/discovery.h"

#include "net/discovery_plan.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <vector>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#include <iphlpapi.h>
#elif !defined(__EMSCRIPTEN__)
#include <arpa/inet.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace apex::net {

namespace {

constexpr std::uint64_t kAnnounceIntervalMs = 1000;
constexpr std::uint64_t kRoomTtlMs = 3000;
constexpr std::uint64_t kTargetRefreshMs = 5000;
constexpr std::uint32_t kLoopback = 0x7F000001u;

}  // namespace

class NullDiscovery final : public IRoomDiscovery {
public:
    std::expected<void, NetError> Advertise(const AnnouncePacket&) override { return {}; }
    void StopAdvertising() override {}
    std::expected<void, NetError> BeginSearch() override { return {}; }
    void SendQuery(std::uint16_t) override {}
    void Poll(std::uint64_t) override {}
    std::span<const RoomInfo> Rooms() const override { return {}; }
    DiscoveryStatus Status() const override { return {}; }
};

#if !defined(__EMSCRIPTEN__)

namespace {

// Winsock and BSD sockets differ in the handle type, the close call, the
// non-blocking switch, process-wide init, interface enumeration and group
// join. These shims are the only platform-specific knowledge below.
#if defined(_WIN32)
using NativeSocket = SOCKET;
constexpr NativeSocket kNoSocket = INVALID_SOCKET;

bool SocketLayerReady() {
    static const bool ready = [] {
        WSADATA data;
        return WSAStartup(MAKEWORD(2, 2), &data) == 0;
    }();
    return ready;
}
void CloseSocket(NativeSocket s) { closesocket(s); }
bool SetNonBlocking(NativeSocket s) {
    u_long on = 1;
    return ioctlsocket(s, FIONBIO, &on) == 0;
}
// GetAdaptersAddresses, because the netmask is what turns an address into a
// subnet-directed broadcast (x.y.z.255) - the address that actually crosses a
// switch when the limited broadcast 255.255.255.255 is dropped or leaves via
// the wrong NIC. Only adapters that are up contribute.
void CollectLocalAddresses(NativeSocket, std::vector<std::uint32_t>& unicast,
                           std::vector<std::uint32_t>& broadcast) {
    constexpr ULONG kFlags = GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST |
                             GAA_FLAG_SKIP_DNS_SERVER;
    ULONG size = 16 * 1024;
    std::vector<char> buffer(size);
    auto* addrs = reinterpret_cast<IP_ADAPTER_ADDRESSES*>(buffer.data());
    ULONG rc = GetAdaptersAddresses(AF_INET, kFlags, nullptr, addrs, &size);
    if (rc == ERROR_BUFFER_OVERFLOW) {
        buffer.resize(size);
        addrs = reinterpret_cast<IP_ADAPTER_ADDRESSES*>(buffer.data());
        rc = GetAdaptersAddresses(AF_INET, kFlags, nullptr, addrs, &size);
    }
    if (rc != NO_ERROR) return;
    for (const IP_ADAPTER_ADDRESSES* adapter = addrs; adapter != nullptr;
         adapter = adapter->Next) {
        if (adapter->OperStatus != IfOperStatusUp) continue;
        for (const IP_ADAPTER_UNICAST_ADDRESS* entry = adapter->FirstUnicastAddress;
             entry != nullptr; entry = entry->Next) {
            const sockaddr* sa = entry->Address.lpSockaddr;
            if (sa == nullptr || sa->sa_family != AF_INET) continue;
            const auto* sin = reinterpret_cast<const sockaddr_in*>(sa);
            const std::uint32_t addr = ntohl(sin->sin_addr.s_addr);
            const unsigned bits = entry->OnLinkPrefixLength;
            const std::uint32_t mask =
                bits >= 32 ? 0xFFFFFFFFu : (bits == 0 ? 0u : 0xFFFFFFFFu << (32 - bits));
            unicast.push_back(addr);
            broadcast.push_back(addr | ~mask);
        }
    }
}
#else
using NativeSocket = int;
constexpr NativeSocket kNoSocket = -1;

bool SocketLayerReady() { return true; }  // nothing to initialise on POSIX
void CloseSocket(NativeSocket s) { ::close(s); }
bool SetNonBlocking(NativeSocket s) {
    const int flags = ::fcntl(s, F_GETFL, 0);
    return flags != -1 && ::fcntl(s, F_SETFL, flags | O_NONBLOCK) != -1;
}
void CollectLocalAddresses(NativeSocket, std::vector<std::uint32_t>& unicast,
                           std::vector<std::uint32_t>& broadcast) {
    ifaddrs* list = nullptr;
    if (::getifaddrs(&list) != 0) return;
    for (ifaddrs* it = list; it != nullptr; it = it->ifa_next) {
        if (it->ifa_addr == nullptr || it->ifa_addr->sa_family != AF_INET) continue;
        if ((it->ifa_flags & IFF_UP) == 0) continue;
        const auto* addr = reinterpret_cast<const sockaddr_in*>(it->ifa_addr);
        unicast.push_back(ntohl(addr->sin_addr.s_addr));
        if ((it->ifa_flags & IFF_BROADCAST) != 0 && it->ifa_broadaddr != nullptr) {
            const auto* brd = reinterpret_cast<const sockaddr_in*>(it->ifa_broadaddr);
            broadcast.push_back(ntohl(brd->sin_addr.s_addr));
        }
    }
    ::freeifaddrs(list);
}
#endif

// Host-order IPv4 from dotted text, 0 when unparsable. inet_addr/inet_pton
// differ in name and deprecation status across the two platforms; four octets
// do not.
std::uint32_t ParseIpv4(const char* text) {
    unsigned a = 0, b = 0, c = 0, d = 0;
    if (std::sscanf(text, "%u.%u.%u.%u", &a, &b, &c, &d) != 4) return 0;
    if (a > 255 || b > 255 || c > 255 || d > 255) return 0;
    return (a << 24) | (b << 16) | (c << 8) | d;
}

bool JoinMulticastGroup(NativeSocket s, std::uint32_t group) {
    if (group == 0) return false;
    ip_mreq request{};
    request.imr_multiaddr.s_addr = htonl(group);
    request.imr_interface.s_addr = htonl(INADDR_ANY);
    return ::setsockopt(s, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                        reinterpret_cast<const char*>(&request), sizeof(request)) == 0;
}

void SetMulticastHops(NativeSocket s, int hops) {
    ::setsockopt(s, IPPROTO_IP, IP_MULTICAST_TTL, reinterpret_cast<const char*>(&hops),
                 sizeof(hops));
}

std::string DescribeAddress(std::uint32_t addr) {
    char text[16];
    std::snprintf(text, sizeof(text), "%u.%u.%u.%u", (addr >> 24) & 0xFF, (addr >> 16) & 0xFF,
                  (addr >> 8) & 0xFF, addr & 0xFF);
    return text;
}

class UdpDiscovery final : public IRoomDiscovery {
public:
    UdpDiscovery(std::uint16_t port, DiscoveryOptions opts)
        : ports_(PortCandidates(port, opts.portSpan)),
          rendezvousPort_(port),
          group_(ParseIpv4(kDiscoveryMulticastGroup)),
          multicast_(opts.multicast && opts.portSpan > 0) {}

    ~UdpDiscovery() override { Close(); }

    std::expected<void, NetError> Advertise(const AnnouncePacket& ad) override {
        if (!EnsureSocket()) return std::unexpected(NetError::NotReady);
        advertise_ = ad;
        advertising_ = true;
        lastBroadcastMs_ = 0;   // send one immediately on the next Poll
        return {};
    }

    void StopAdvertising() override { advertising_ = false; }

    std::expected<void, NetError> BeginSearch() override {
        if (!EnsureSocket()) return std::unexpected(NetError::NotReady);
        return {};
    }

    void SendQuery(std::uint16_t roomCode) override {
        if (!EnsureSocket()) return;
        QueryPacket q;
        q.roomCode = roomCode;
        std::array<std::byte, 16> buf{};
        const auto n = EncodeQuery(q, buf);
        if (!n) return;
        SendEverywhere(buf.data(), *n);
    }

    void Poll(std::uint64_t nowMs) override {
        if (socket_ == kNoSocket) return;

        RefreshTargets(nowMs);

        if (advertising_ && nowMs - lastBroadcastMs_ >= kAnnounceIntervalMs) {
            lastBroadcastMs_ = nowMs;
            std::array<std::byte, 128> buf{};
            if (const auto n = EncodeAnnounce(advertise_, buf)) {
                SendEverywhere(buf.data(), *n);
            }
        }

        for (;;) {
            sockaddr_in from{};
            socklen_t fromLen = static_cast<socklen_t>(sizeof(from));
            const long long got = static_cast<long long>(
                recvfrom(socket_, reinterpret_cast<char*>(rx_.data()),
                         static_cast<int>(rx_.size()), 0,
                         reinterpret_cast<sockaddr*>(&from), &fromLen));
            // Non-blocking socket: <= 0 means drained (EWOULDBLOCK) or error.
            if (got <= 0) break;
            HandleDatagram(std::span<const std::byte>(rx_.data(), static_cast<std::size_t>(got)),
                           from, nowMs);
        }

        // Expire stale rooms.
        rooms_.erase(std::remove_if(rooms_.begin(), rooms_.end(),
                                    [&](const RoomInfo& r) {
                                        return nowMs > r.lastSeenMs &&
                                               nowMs - r.lastSeenMs > kRoomTtlMs;
                                    }),
                     rooms_.end());
    }

    std::span<const RoomInfo> Rooms() const override { return rooms_; }

    DiscoveryStatus Status() const override { return status_; }

private:
    // Bind the first free port in the range. Settling for another port is not a
    // failure: hosts still reach us through the broadcast fan-out, and a host
    // bound to a non-base port still answers queries on it.
    bool EnsureSocket() {
        if (socket_ != kNoSocket) return true;
        if (!SocketLayerReady()) return false;

        for (const std::uint16_t port : ports_) {
            // Exclusive first, so a lone instance owns the familiar port
            // outright. Only the rendezvous port may then be shared, and only
            // for the group channel - that is what lets a second instance on
            // the same machine still hear every announcement.
            socket_ = OpenBound(port, false);
            if (socket_ == kNoSocket && multicast_ && port == rendezvousPort_) {
                socket_ = OpenBound(port, true);
            }
            if (socket_ != kNoSocket) break;
        }
        if (socket_ == kNoSocket) {
            socket_ = OpenBound(0, false);
            status_.fallbackPort = socket_ != kNoSocket;
        }
        if (socket_ == kNoSocket) return false;

        sockaddr_in bound{};
        socklen_t boundLen = static_cast<socklen_t>(sizeof(bound));
        if (::getsockname(socket_, reinterpret_cast<sockaddr*>(&bound), &boundLen) == 0) {
            status_.boundPort = ntohs(bound.sin_port);
        }
        // Membership only pays off on the rendezvous port: multicast is
        // delivered per (group, port), so a socket that lost the race would
        // join a group nobody sends to.
        if (multicast_ && status_.boundPort == rendezvousPort_) {
            status_.multicast = JoinMulticastGroup(socket_, group_);
            SetMulticastHops(socket_, 1);
        }
        RefreshTargets(0, true);
        LogSocket();
        return true;
    }

    NativeSocket OpenBound(std::uint16_t port, bool shared) {
        const NativeSocket s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (s == kNoSocket) return kNoSocket;
        const int on = 1;
#if defined(_WIN32)
        // Only the rendezvous port is shared, and only because group traffic is
        // delivered to every joined socket. Anywhere else SO_REUSEADDR would
        // hand a second process a socket that binds "successfully" and then
        // never receives anything, instead of the clean EADDRINUSE that sends
        // us to the next port in the range.
        if (shared) {
            setsockopt(s, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&on),
                       sizeof(on));
        }
#else
        (void)shared;   // REUSEADDR does not alias a bound UDP port on POSIX
        setsockopt(s, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&on), sizeof(on));
#endif
        setsockopt(s, SOL_SOCKET, SO_BROADCAST, reinterpret_cast<const char*>(&on), sizeof(on));
        if (!SetNonBlocking(s)) {
            CloseSocket(s);
            return kNoSocket;
        }
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(port);
        if (bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            CloseSocket(s);
            return kNoSocket;
        }
        return s;
    }

    void Close() {
        if (socket_ != kNoSocket) {
            CloseSocket(socket_);
            socket_ = kNoSocket;
        }
    }

    // Interfaces come and go (cable in, VPN up), so the target list is rebuilt
    // on a timer rather than once per socket.
    void RefreshTargets(std::uint64_t nowMs, bool force = false) {
        if (!force && nowMs < lastTargetMs_ + kTargetRefreshMs) return;
        lastTargetMs_ = nowMs;
        std::vector<std::uint32_t> unicast;
        std::vector<std::uint32_t> broadcast{INADDR_BROADCAST, kLoopback};
        CollectLocalAddresses(socket_, unicast, broadcast);
        std::vector<std::uint32_t> raw = broadcast;
        raw.insert(raw.end(), unicast.begin(), unicast.end());
        targets_ = MergeTargets(raw);
        localAddrs_ = MergeTargets(unicast);
    }

    // A room's endpoint is one of the addresses its announcement arrived from.
    // Prefer one of our own interface addresses (that pair is guaranteed
    // routable, and it is the one a real peer would use too) over a virtual
    // adapter's address and over loopback.
    bool PreferNewEndpoint(std::uint32_t current, std::uint32_t candidate) const {
        if (current == candidate) return false;
        if (current == kLoopback) return true;
        return !IsLocal(current) && IsLocal(candidate);
    }

    bool IsLocal(std::uint32_t addr) const {
        return std::binary_search(localAddrs_.begin(), localAddrs_.end(), addr);
    }

    void SendEverywhere(const std::byte* data, std::size_t size) {
        if (multicast_) {
            sockaddr_in to{};
            to.sin_family = AF_INET;
            to.sin_addr.s_addr = htonl(group_);
            to.sin_port = htons(rendezvousPort_);
            SendRaw(to, data, size);
        }
        for (const std::uint32_t addr : targets_) {
            for (const std::uint16_t port : ports_) {
                sockaddr_in to{};
                to.sin_family = AF_INET;
                to.sin_addr.s_addr = htonl(addr);
                to.sin_port = htons(port);
                SendRaw(to, data, size);
            }
        }
    }

    void SendRaw(const sockaddr_in& to, const std::byte* data, std::size_t size) {
        const long long n = static_cast<long long>(
            sendto(socket_, reinterpret_cast<const char*>(data), static_cast<int>(size), 0,
                   reinterpret_cast<const sockaddr*>(&to), sizeof(to)));
        if (n < 0) {
            ++status_.sendFailures;
        } else {
            ++status_.sent;
        }
    }

    void HandleDatagram(std::span<const std::byte> bytes, const sockaddr_in& from,
                        std::uint64_t nowMs) {
        if (bytes.size() >= 6) {
            const std::uint8_t kind = static_cast<std::uint8_t>(bytes[5]);
            if (kind == kDiscoQuery) {
                // Someone is looking for rooms: answer with our announcement,
                // unicast to the port the query came from, so a searcher that
                // could not get a port in the range is still served.
                if (!advertising_) return;
                std::array<std::byte, 128> buf{};
                if (const auto n = EncodeAnnounce(advertise_, buf)) {
                    SendRaw(from, buf.data(), *n);
                }
                return;
            }
        }
        const auto ad = DecodeAnnounce(bytes);
        if (!ad) return;
        if (ad->protocolVersion != kProtocolVersion) return;
        // Our own announce comes back through loopback and every local address.
        if (advertising_ && ad->roomCode == advertise_.roomCode &&
            ad->playerId == advertise_.playerId) {
            return;
        }
        ++status_.received;

        Endpoint ep;
        ep.ipv4 = ntohl(from.sin_addr.s_addr);
        ep.port = ad->gamePort;

        for (RoomInfo& room : rooms_) {
            if (room.announce.roomCode != ad->roomCode) continue;
            if (room.announce.playerId != ad->playerId) continue;
            // One host reaches us up to three times (loopback, its interface
            // address, its broadcast address). Keep the most useful source: a
            // loopback route only works when both instances share a machine.
            if (PreferNewEndpoint(room.host.ipv4, ep.ipv4)) room.host = ep;
            room.announce = *ad;
            room.lastSeenMs = nowMs;
            return;
        }
        RoomInfo room;
        room.announce = *ad;
        room.host = ep;
        room.lastSeenMs = nowMs;
        rooms_.push_back(room);
    }

    void LogSocket() {
        char targets[192] = {};
        std::size_t used = 0;
        for (std::size_t i = 0; i < targets_.size() && used + 20 < sizeof(targets); ++i) {
            const std::string text = DescribeAddress(targets_[i]);
            const int written = std::snprintf(targets + used, sizeof(targets) - used, "%s%s",
                                             i == 0 ? "" : " ", text.c_str());
            if (written <= 0) break;
            used += static_cast<std::size_t>(written);
        }
        std::fprintf(stderr,
                     "[disco] udp %u%s%s  ports %u-%u  fan-out: %s\n",
                     static_cast<unsigned>(status_.boundPort),
                     status_.fallbackPort ? " (range busy, ephemeral)" : "",
                     status_.multicast ? "  mcast on" : "  mcast off",
                     static_cast<unsigned>(ports_.empty() ? 0 : ports_.front()),
                     static_cast<unsigned>(ports_.empty() ? 0 : ports_.back()), targets);
    }

    std::vector<std::uint16_t> ports_;
    std::uint16_t rendezvousPort_ = kDiscoveryPortDefault;
    std::uint32_t group_ = 0;
    bool multicast_ = true;
    NativeSocket socket_ = kNoSocket;
    bool advertising_ = false;
    std::uint64_t lastBroadcastMs_ = 0;
    std::uint64_t lastTargetMs_ = 0;
    AnnouncePacket advertise_{};
    std::vector<std::uint32_t> targets_;
    std::vector<std::uint32_t> localAddrs_;   // interface addresses, sorted
    std::vector<RoomInfo> rooms_;
    DiscoveryStatus status_{};
    std::array<std::byte, 256> rx_{};
};

}  // namespace

std::unique_ptr<IRoomDiscovery> MakeUdpDiscovery(std::uint16_t discoveryPort,
                                                DiscoveryOptions opts) {
    return std::make_unique<UdpDiscovery>(discoveryPort, opts);
}

std::unique_ptr<IRoomDiscovery> MakeNullDiscovery() { return std::make_unique<NullDiscovery>(); }

#else

std::unique_ptr<IRoomDiscovery> MakeUdpDiscovery(std::uint16_t, DiscoveryOptions) {
    return MakeNullDiscovery();
}
std::unique_ptr<IRoomDiscovery> MakeNullDiscovery() { return std::make_unique<NullDiscovery>(); }

#endif

}  // namespace apex::net
