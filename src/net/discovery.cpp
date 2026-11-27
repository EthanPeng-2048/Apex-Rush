// UDP room discovery. Winsock is confined to this file.

#include "net/discovery.h"

#include <algorithm>
#include <cstring>

#if !defined(__EMSCRIPTEN__)
#include <winsock2.h>
#include <ws2tcpip.h>
#endif

namespace apex::net {

namespace {

constexpr std::uint64_t kAnnounceIntervalMs = 1000;
constexpr std::uint64_t kRoomTtlMs = 3000;

}  // namespace

class NullDiscovery final : public IRoomDiscovery {
public:
    std::expected<void, NetError> Advertise(const AnnouncePacket&) override { return {}; }
    void StopAdvertising() override {}
    std::expected<void, NetError> BeginSearch() override { return {}; }
    void SendQuery(std::uint16_t) override {}
    void Poll(std::uint64_t) override {}
    std::span<const RoomInfo> Rooms() const override { return {}; }
};

#if !defined(__EMSCRIPTEN__)

namespace {

bool WsaReady() {
    static bool ready = false;
    static bool tried = false;
    if (!tried) {
        WSADATA data;
        ready = WSAStartup(MAKEWORD(2, 2), &data) == 0;
        tried = true;
    }
    return ready;
}

class UdpDiscovery final : public IRoomDiscovery {
public:
    explicit UdpDiscovery(std::uint16_t port) : port_(port) {}
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
        searching_ = true;
        return {};
    }

    void SendQuery(std::uint16_t roomCode) override {
        if (!EnsureSocket()) return;
        QueryPacket q;
        q.roomCode = roomCode;
        std::array<std::byte, 16> buf{};
        const auto n = EncodeQuery(q, buf);
        if (!n) return;
        SendBroadcast(buf.data(), *n);
    }

    void Poll(std::uint64_t nowMs) override {
        if (socket_ == INVALID_SOCKET) return;

        if (advertising_ && nowMs - lastBroadcastMs_ >= kAnnounceIntervalMs) {
            lastBroadcastMs_ = nowMs;
            std::array<std::byte, 128> buf{};
            if (const auto n = EncodeAnnounce(advertise_, buf)) {
                SendBroadcast(buf.data(), *n);
            }
        }

        for (;;) {
            sockaddr_in from{};
            int fromLen = static_cast<int>(sizeof(from));
            const int got = recvfrom(socket_, reinterpret_cast<char*>(rx_.data()), static_cast<int>(rx_.size()), 0,
                                     reinterpret_cast<sockaddr*>(&from), &fromLen);
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

private:
    bool EnsureSocket() {
        if (socket_ != INVALID_SOCKET) return true;
        if (!WsaReady()) return false;
        socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (socket_ == INVALID_SOCKET) return false;
        const BOOL reuse = TRUE;
        setsockopt(socket_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&reuse),
                   sizeof(reuse));
        const BOOL bcast = TRUE;
        setsockopt(socket_, SOL_SOCKET, SO_BROADCAST, reinterpret_cast<const char*>(&bcast),
                   sizeof(bcast));
        u_long nonBlocking = 1;
        ioctlsocket(socket_, FIONBIO, &nonBlocking);

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(port_);
        if (bind(socket_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR) {
            closesocket(socket_);
            socket_ = INVALID_SOCKET;
            return false;
        }
        return true;
    }

    void Close() {
        if (socket_ != INVALID_SOCKET) {
            closesocket(socket_);
            socket_ = INVALID_SOCKET;
        }
    }

    void SendBroadcast(const std::byte* data, std::size_t size) {
        sockaddr_in to{};
        to.sin_family = AF_INET;
        to.sin_addr.s_addr = htonl(INADDR_BROADCAST);
        to.sin_port = htons(port_);
        sendto(socket_, reinterpret_cast<const char*>(data), static_cast<int>(size), 0,
               reinterpret_cast<sockaddr*>(&to), sizeof(to));
    }

    void HandleDatagram(std::span<const std::byte> bytes, const sockaddr_in& from,
                        std::uint64_t nowMs) {
        if (bytes.size() >= 6) {
            const std::uint8_t kind = static_cast<std::uint8_t>(bytes[5]);
            if (kind == kDiscoQuery) {
                // Someone is looking for rooms: answer with our announcement,
                // unicast, so it survives broadcast filtering.
                if (!advertising_) return;
                std::array<std::byte, 128> buf{};
                if (const auto n = EncodeAnnounce(advertise_, buf)) {
                    sendto(socket_, reinterpret_cast<const char*>(buf.data()),
                           static_cast<int>(*n), 0, reinterpret_cast<const sockaddr*>(&from),
                           sizeof(from));
                }
                return;
            }
        }
        const auto ad = DecodeAnnounce(bytes);
        if (!ad) return;
        if (ad->protocolVersion != kProtocolVersion) return;

        Endpoint ep;
        ep.ipv4 = ntohl(from.sin_addr.s_addr);
        ep.port = ad->gamePort;

        for (RoomInfo& room : rooms_) {
            if (room.host.ipv4 == ep.ipv4 && room.announce.roomCode == ad->roomCode) {
                room.announce = *ad;
                room.lastSeenMs = nowMs;
                return;
            }
        }
        RoomInfo room;
        room.announce = *ad;
        room.host = ep;
        room.lastSeenMs = nowMs;
        rooms_.push_back(room);
    }

    std::uint16_t port_ = kDiscoveryPortDefault;
    SOCKET socket_ = INVALID_SOCKET;
    bool advertising_ = false;
    bool searching_ = false;
    std::uint64_t lastBroadcastMs_ = 0;
    AnnouncePacket advertise_{};
    std::vector<RoomInfo> rooms_;
    std::array<std::byte, 256> rx_{};
};

}  // namespace

std::unique_ptr<IRoomDiscovery> MakeUdpDiscovery(std::uint16_t discoveryPort) {
    return std::make_unique<UdpDiscovery>(discoveryPort);
}

std::unique_ptr<IRoomDiscovery> MakeNullDiscovery() { return std::make_unique<NullDiscovery>(); }

#else

std::unique_ptr<IRoomDiscovery> MakeUdpDiscovery(std::uint16_t) { return MakeNullDiscovery(); }
std::unique_ptr<IRoomDiscovery> MakeNullDiscovery() { return std::make_unique<NullDiscovery>(); }

#endif

}  // namespace apex::net
