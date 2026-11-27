// Transport implementations. This is the only translation unit that talks to
// ENet; the loopback pair keeps the session layer testable in-process.

#if defined(_WIN32) && !defined(__EMSCRIPTEN__)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#endif

#include "net/transport.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <memory>
#include <unordered_map>
#include <utility>

#if !defined(__EMSCRIPTEN__)
#include <enet/enet.h>
#endif

namespace apex::net {

namespace {
// ENet hands out `unsigned char*`; std::byte needs an explicit copy.
std::vector<std::byte> CopyBytes(const void* p, std::size_t n) {
    std::vector<std::byte> v(n);
    if (n != 0) std::memcpy(v.data(), p, n);
    return v;
}
}  // namespace

std::uint32_t LoopbackNet::NextRandom() {
    rng = rng * 1664525u + 1013904223u;
    return rng >> 8;
}

// ------------------------------------------------------------------- null ---

namespace {

class NullTransport final : public ITransport {
public:
    std::expected<void, NetError> Listen(const Endpoint&, int) override {
        listening_ = true;
        return {};
    }
    std::expected<void, NetError> Connect(const Endpoint&) override {
        connected_ = true;
        return {};
    }
    void Close() override {
        listening_ = false;
        connected_ = false;
    }
    std::expected<void, NetError> Send(PeerId, Channel, Delivery,
                                       std::span<const std::byte>) override {
        return {};
    }
    std::span<const NetEvent> Poll(std::uint64_t) override { return {}; }
    PeerStats Stats(PeerId) const override { return {}; }
    std::span<const PeerId> Peers() const override { return {}; }
    bool Listening() const override { return listening_; }
    bool Connected() const override { return connected_; }

private:
    bool listening_ = false;
    bool connected_ = false;
};

}  // namespace

std::unique_ptr<ITransport> MakeNullTransport() { return std::make_unique<NullTransport>(); }

// ------------------------------------------------------------------ enet ---

#if !defined(__EMSCRIPTEN__)

namespace {

// ENet flags are counter-intuitive: RELIABLE is explicit, "sequenced" is the
// default behaviour of an unreliable channel (flags 0), and UNSEQUENCED turns
// sequencing off. Map our semantics onto that explicitly.
ENetPacketFlag DeliveryFlags(Delivery mode) {
    switch (mode) {
        case Delivery::Reliable:
            return ENET_PACKET_FLAG_RELIABLE;
        case Delivery::UnreliableSequenced:
            return static_cast<ENetPacketFlag>(0);
        case Delivery::Unreliable:
        default:
            return ENET_PACKET_FLAG_UNSEQUENCED;
    }
}

class EnetTransport final : public ITransport {
public:
    ~EnetTransport() override { Close(); }

    std::expected<void, NetError> Listen(const Endpoint& bind, int maxPeers) override {
        if (host_) return std::unexpected(NetError::NotReady);
        ENetAddress addr{};
        addr.host = ENET_HOST_ANY;
        addr.port = bind.port;
        host_ = enet_host_create(&addr, static_cast<size_t>(std::max(1, maxPeers)), 2, 0, 0);
        if (!host_) {
            std::fprintf(stderr, "[net] enet_host_create(port=%u, peers=%d) failed, wsa=%d\n",
                         static_cast<unsigned>(addr.port), maxPeers, WSAGetLastError());
            return std::unexpected(NetError::NotReady);
        }
        role_ = Role::Host;
        return {};
    }

    std::expected<void, NetError> Connect(const Endpoint& hostEp) override {
        if (host_) return std::unexpected(NetError::NotReady);
        host_ = enet_host_create(nullptr, 1, 2, 0, 0);
        if (!host_) return std::unexpected(NetError::NotReady);
        ENetAddress addr{};
        // Endpoint.ipv4 is host byte order; ENetAddress.host is network order.
        addr.host = ENET_HOST_TO_NET_32(hostEp.ipv4);
        addr.port = hostEp.port;
        if (!enet_host_connect(host_, &addr, 2, 0)) {
            enet_host_destroy(host_);
            host_ = nullptr;
            return std::unexpected(NetError::NotReady);
        }
        role_ = Role::Client;
        return {};
    }

    void Close() override {
        if (!host_) return;
        for (auto& [peer, id] : peers_) {
            (void)id;
            enet_peer_disconnect_now(peer, 0);
        }
        peers_.clear();
        peerIds_.clear();
        ids_.clear();
        enet_host_flush(host_);
        enet_host_destroy(host_);
        host_ = nullptr;
        role_ = Role::None;
    }

    std::expected<void, NetError> Send(PeerId to, Channel ch, Delivery mode,
                                       std::span<const std::byte> packet) override {
        if (!host_) return std::unexpected(NetError::Closed);
        if (packet.size() > kMaxDatagram) return std::unexpected(NetError::PayloadTooLarge);
        ENetPeer* peer = PeerFromId(to);
        if (!peer) return std::unexpected(NetError::PeerGone);
        ENetPacket* pkt =
            enet_packet_create(packet.data(), packet.size(), DeliveryFlags(mode));
        if (!pkt) return std::unexpected(NetError::NotReady);
        if (enet_peer_send(peer, static_cast<std::uint8_t>(ch), pkt) != 0) {
            enet_packet_destroy(pkt);
            return std::unexpected(NetError::PeerGone);
        }
        bytesOut_[to.value] += packet.size();
        return {};
    }

    std::span<const NetEvent> Poll(std::uint64_t) override {
        events_.clear();
        payloads_.clear();
        if (!host_) return events_;
        for (;;) {
            ENetEvent ev;
            const int rc = enet_host_service(host_, &ev, 0);
            if (rc <= 0) break;
            if (ev.type == ENET_EVENT_TYPE_CONNECT) {
                NetEvent out;
                out.kind = NetEventKind::Connected;
                out.peer = AssignId(ev.peer);
                events_.push_back(out);
            } else if (ev.type == ENET_EVENT_TYPE_DISCONNECT) {
                NetEvent out;
                out.kind = (ev.data == 0) ? NetEventKind::TimedOut : NetEventKind::Disconnected;
                auto it = ids_.find(ev.peer);
                out.peer = (it != ids_.end()) ? it->second : PeerId{};
                if (it != ids_.end()) Forget(ev.peer, out.peer);
                events_.push_back(out);
            } else if (ev.type == ENET_EVENT_TYPE_RECEIVE) {
                auto it = ids_.find(ev.peer);
                PeerId id = (it != ids_.end()) ? it->second : PeerId{};
                payloads_.push_back(CopyBytes(ev.packet->data, ev.packet->dataLength));
                NetEvent out;
                out.kind = NetEventKind::Received;
                out.peer = id;
                out.channel = static_cast<Channel>(ev.channelID);
                out.sequence = 0;  // snapshots are ordered by the channel, not by our own counter
                out.payload = payloads_.back();
                bytesIn_[id.value] += ev.packet->dataLength;
                events_.push_back(out);
                enet_packet_destroy(ev.packet);
            }
        }
        return events_;
    }

    PeerStats Stats(PeerId peer) const override {
        PeerStats s;
        if (!peer.valid()) return s;
        ENetPeer* p = const_cast<EnetTransport*>(this)->PeerFromId(peer);
        if (p) {
            s.rttMs = p->roundTripTime;
            s.rttVarianceMs = p->roundTripTimeVariance;
            s.packetLossPct =
                static_cast<std::uint8_t>(std::min<std::uint32_t>(100, p->packetLoss / 655u));
        }
        if (peer.value < bytesIn_.size()) {
            s.bytesIn = bytesIn_[peer.value];
            s.bytesOut = bytesOut_[peer.value];
        }
        return s;
    }

    std::span<const PeerId> Peers() const override { return peerIds_; }
    bool Listening() const override { return host_ != nullptr && role_ == Role::Host; }
    bool Connected() const override { return host_ != nullptr && role_ == Role::Client; }

private:
    enum class Role { None, Host, Client };

    PeerId AssignId(ENetPeer* peer) {
        PeerId id;
        if (role_ == Role::Client) {
            id.value = 0;   // the client's only peer is the host
        } else {
            for (std::uint8_t candidate = 1; candidate < kMaxCars; ++candidate) {
                bool used = false;
                for (PeerId p : peerIds_) used = used || p.value == candidate;
                if (!used) {
                    id.value = candidate;
                    break;
                }
            }
        }
        ids_[peer] = id;
        peers_.emplace_back(peer, id);
        peerIds_.push_back(id);
        return id;
    }

    void Forget(ENetPeer* peer, PeerId id) {
        ids_.erase(peer);
        peers_.erase(std::remove_if(peers_.begin(), peers_.end(),
                                    [&](const auto& kv) { return kv.first == peer; }),
                     peers_.end());
        peerIds_.erase(std::remove_if(peerIds_.begin(), peerIds_.end(),
                                      [&](PeerId p) { return p == id; }),
                       peerIds_.end());
    }

    ENetPeer* PeerFromId(PeerId id) {
        for (auto& [peer, pid] : peers_) {
            if (pid == id) return peer;
        }
        return nullptr;
    }

    ENetHost* host_ = nullptr;
    Role role_ = Role::None;
    std::vector<std::pair<ENetPeer*, PeerId>> peers_;
    std::vector<PeerId> peerIds_;
    std::unordered_map<ENetPeer*, PeerId> ids_;
    std::vector<NetEvent> events_;
    std::vector<std::vector<std::byte>> payloads_;
    std::array<std::uint64_t, 256> bytesIn_{};
    std::array<std::uint64_t, 256> bytesOut_{};
};

}  // namespace

std::unique_ptr<ITransport> MakeEnetTransport() {
    static bool initialized = false;
    if (!initialized) {
        const int rc = enet_initialize();
        std::fprintf(stderr, "[net] enet_initialize rc=%d\n", rc);
        if (rc != 0) return nullptr;
        initialized = true;
    }
    return std::make_unique<EnetTransport>();
}

#else

std::unique_ptr<ITransport> MakeEnetTransport() { return MakeNullTransport(); }

#endif  // __EMSCRIPTEN__

// -------------------------------------------------------------- loopback ---

namespace {

// Two ends share a LoopbackNet; delivery happens on the next Poll.
class LoopbackTransport final : public ITransport {
public:
    LoopbackTransport(LoopbackNet& net, bool isHost) : net_(net), isHost_(isHost) {}

    std::expected<void, NetError> Listen(const Endpoint&, int) override {
        listening_ = true;
        return {};
    }
    std::expected<void, NetError> Connect(const Endpoint&) override {
        connected_ = true;
        pendingConnect_ = true;
        return {};
    }
    void Close() override {
        listening_ = false;
        connected_ = false;
        closed_ = true;
    }

    std::expected<void, NetError> Send(PeerId, Channel ch, Delivery,
                                       std::span<const std::byte> packet) override {
        if (closed_) return std::unexpected(NetError::Closed);
        if (packet.size() > kMaxDatagram) return std::unexpected(NetError::PayloadTooLarge);
        if (net_.lossPct > 0 &&
            (net_.NextRandom() % 100u) < static_cast<std::uint32_t>(net_.lossPct)) {
            return {};
        }
        LoopbackNet::Item item;
        item.channel = ch;
        item.sequence = ++sendSeq_;
        item.bytes.assign(packet.begin(), packet.end());
        if (isHost_) {
            net_.toClient.push_back(std::move(item));
        } else {
            net_.toHost.push_back(std::move(item));
        }
        return {};
    }

    std::span<const NetEvent> Poll(std::uint64_t) override {
        events_.clear();
        payloads_.clear();
        if (pendingConnect_) {
            pendingConnect_ = false;
            NetEvent ev;
            ev.kind = NetEventKind::Connected;
            ev.peer = PeerId{0};
            events_.push_back(ev);
            if (isHost_) connected_ = true;
        }
        auto& inbox = isHost_ ? net_.toHost : net_.toClient;
        for (auto& item : inbox) {
            payloads_.push_back(std::move(item.bytes));
            NetEvent ev;
            ev.kind = NetEventKind::Received;
            ev.peer = isHost_ ? PeerId{1} : PeerId{0};
            ev.channel = item.channel;
            ev.sequence = item.sequence;
            ev.payload = payloads_.back();
            events_.push_back(ev);
        }
        inbox.clear();
        return events_;
    }

    PeerStats Stats(PeerId) const override { return {}; }
    std::span<const PeerId> Peers() const override { return peers_; }
    bool Listening() const override { return listening_; }
    bool Connected() const override { return connected_; }

private:
    LoopbackNet& net_;
    bool isHost_ = false;
    bool listening_ = false;
    bool connected_ = false;
    bool closed_ = false;
    bool pendingConnect_ = false;
    std::uint16_t sendSeq_ = 0;
    std::vector<PeerId> peers_{PeerId{1}};
    std::vector<NetEvent> events_;
    std::vector<std::vector<std::byte>> payloads_;
};

}  // namespace

std::unique_ptr<ITransport> MakeLoopbackTransport(LoopbackNet& net, bool isHost) {
    return std::make_unique<LoopbackTransport>(net, isHost);
}

}  // namespace apex::net
