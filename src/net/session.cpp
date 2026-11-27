// Host/client session state machines (see docs/online/architecture.md §7).

#include "net/session.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>

namespace apex::net {
namespace {

std::uint64_t IntervalMs(int hz) { return hz > 0 ? static_cast<std::uint64_t>(1000 / hz) : 16u; }

std::vector<std::byte> EncodeToBuffer(auto&& encoder) {
    std::array<std::byte, kMaxDatagram> buf{};
    std::vector<std::byte> out;
    const auto n = encoder(buf);
    if (!n) return out;
    out.assign(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(*n));
    return out;
}

// --------------------------------------------------------------- host ------

class HostSession final : public INetSession {
public:
    HostSession(const NetConfig& cfg, ITransport& transport, IRoomDiscovery& discovery)
        : cfg_(cfg), transport_(transport), discovery_(discovery) {
        for (auto& seq : appliedSeq_) seq.store(0);
    }

    ~HostSession() override { discovery_.StopAdvertising(); }

    void Start() {
        Endpoint bind;
        bind.port = cfg_.gamePort;
        if (!transport_.Listen(bind, std::max(1, cfg_.maxPlayers - 1))) {
            state_ = SessionState::Closed;
            return;
        }
        AnnouncePacket ad;
        ad.roomCode = cfg_.roomCode;
        ad.gamePort = cfg_.gamePort;
        ad.buildSig = cfg_.buildSig;
        ad.mapHash = cfg_.mapHash;
        ad.mapName = cfg_.mapName;
        ad.hostName = cfg_.playerName;
        ad.players = 1;
        ad.maxPlayers = static_cast<std::uint8_t>(cfg_.maxPlayers);
        ad.phase = 0;
        (void)discovery_.Advertise(ad);
        state_ = SessionState::Advertising;
    }

    void Tick(std::uint64_t nowMs) override {
        discovery_.Poll(nowMs);
        for (const NetEvent& ev : transport_.Poll(nowMs)) {
            switch (ev.kind) {
                case NetEventKind::Connected:
                    break;   // wait for Hello
                case NetEventKind::Disconnected:
                case NetEventKind::TimedOut:
                    DropPeer(ev.peer, true);
                    break;
                case NetEventKind::Received:
                    HandlePacket(ev, nowMs);
                    break;
            }
        }
        if (worldValid_ && nowMs - lastSnapshotMs_ >= IntervalMs(cfg_.snapshotHz)) {
            lastSnapshotMs_ = nowMs;
            SendSnapshots();
        }
    }

    std::span<const SessionEvent> DrainEvents() override {
        drainBuffer_ = std::move(pending_);
        pending_.clear();
        return drainBuffer_;
    }

    SessionState State() const override { return state_; }
    bool IsHost() const override { return true; }
    std::uint8_t LocalSlot() const override { return 0; }

    void PublishSnapshot(const HostWorld& world) override {
        world_ = world;
        worldValid_ = true;
    }

    void SetAppliedInputSeq(std::uint8_t slot, std::uint16_t seq) override {
        if (slot < kMaxCars) appliedSeq_[slot].store(seq);
    }

    void SetRules(const RulesPacket& rules) override { cfg_.rules = rules; }

    void StartRace(std::uint16_t countdownCs) override {
        RaceStartPacket p;
        p.startHostTick = world_.packet.hostTick;
        p.countdownCs = countdownCs;
        p.rules = cfg_.rules;
        auto bytes = EncodeToBuffer([&](std::span<std::byte> b) { return EncodeRaceStart(p, b); });
        if (!bytes.empty()) SendToAll(bytes, Delivery::Reliable, Channel::Control);
        state_ = SessionState::Racing;
        UpdateAnnouncePhase(2);
    }

    void RestartRace(std::uint16_t countdownCs) override {
        RestartPacket p;
        p.startHostTick = world_.packet.hostTick;
        p.countdownCs = countdownCs;
        auto bytes = EncodeToBuffer([&](std::span<std::byte> b) { return EncodeRestart(p, b); });
        if (!bytes.empty()) SendToAll(bytes, Delivery::Reliable, Channel::Control);
        state_ = SessionState::Racing;
    }

    void EndRace(std::vector<RaceEndEntry> results) override {
        RaceEndPacket p;
        p.entries = std::move(results);
        auto bytes = EncodeToBuffer([&](std::span<std::byte> b) { return EncodeRaceEnd(p, b); });
        if (!bytes.empty()) SendToAll(bytes, Delivery::Reliable, Channel::Control);
        state_ = SessionState::Results;
        UpdateAnnouncePhase(3);
    }

    void BroadcastLobby() override {
        LobbyUpdatePacket p;
        LobbyPlayer host;
        host.peerId = 0;
        host.slot = 0;
        host.name = cfg_.playerName;
        host.color = 0;
        host.ready = 1;
        p.players.push_back(host);
        for (const PeerSlot& peer : peers_) {
            LobbyPlayer pl;
            pl.peerId = peer.id.value;
            pl.slot = peer.slot;
            pl.name = peer.name;
            pl.color = peer.color;
            pl.ready = 1;
            p.players.push_back(pl);
        }
        auto bytes = EncodeToBuffer([&](std::span<std::byte> b) { return EncodeLobbyUpdate(p, b); });
        if (!bytes.empty()) SendToAll(bytes, Delivery::Reliable, Channel::Control);
        // The host's own UI needs the same list the clients just received.
        SessionEvent changed;
        changed.kind = SessionEventKind::LobbyChanged;
        changed.lobby = p.players;
        pending_.push_back(changed);
        UpdateAnnouncePhase(state_ == SessionState::Racing ? 2 : 0);
    }

    void SendLapComplete(const LapCompletePacket& p) override { SendControl([&](auto b) { return EncodeLapComplete(p, b); }); }
    void SendPlayerFinished(const PlayerFinishedPacket& p) override { SendControl([&](auto b) { return EncodePlayerFinished(p, b); }); }
    void SendCarRecovered(const CarRecoveredPacket& p) override { SendControl([&](auto b) { return EncodeCarRecovered(p, b); }); }
    void SendPartDamaged(const PartDamagedPacket& p) override { SendControl([&](auto b) { return EncodePartDamaged(p, b); }); }

    InputSample LatestInput(std::uint8_t slot) const override {
        for (const PeerSlot& peer : peers_) {
            if (peer.slot == slot && peer.hasInput) return peer.latest;
        }
        return {};
    }

    bool HasFreshInput(std::uint8_t slot) const override {
        for (const PeerSlot& peer : peers_) {
            if (peer.slot != slot) continue;
            if (!peer.hasInput) return false;
            return lastTickMs_ - peer.lastInputMs < 250;
        }
        return false;
    }

    std::uint8_t PeerCount() const { return static_cast<std::uint8_t>(peers_.size()); }

private:
    struct PeerSlot {
        PeerId id{};
        std::uint8_t slot = 0;
        std::uint8_t color = 0;
        std::string name;
        InputSample latest{};
        std::uint16_t lastSeq = 0;
        bool hasInput = false;
        std::uint64_t lastInputMs = 0;
    };

    template <typename Fn>
    void SendControl(Fn&& encoder) {
        auto bytes = EncodeToBuffer(encoder);
        if (!bytes.empty()) SendToAll(bytes, Delivery::Reliable, Channel::Control);
    }

    void SendToAll(const std::vector<std::byte>& bytes, Delivery mode, Channel ch) {
        for (const PeerSlot& peer : peers_) {
            (void)transport_.Send(peer.id, ch, mode, bytes);
        }
    }

    void HandlePacket(const NetEvent& ev, std::uint64_t nowMs) {
        lastTickMs_ = nowMs;
        const auto type = PeekMessageType(ev.payload);
        if (!type) return;
        switch (*type) {
            case MsgType::Hello:
                HandleHello(ev, nowMs);
                break;
            case MsgType::Input:
                HandleInput(ev, nowMs);
                break;
            case MsgType::Bye:
                DropPeer(ev.peer, false);
                break;
            default:
                break;
        }
    }

    void HandleHello(const NetEvent& ev, std::uint64_t nowMs) {
        (void)nowMs;
        const auto hello = DecodeHello(ev.payload);
        if (!hello) return;
        for (const PeerSlot& peer : peers_) {
            if (peer.id == ev.peer) {
                SendReject(ev.peer, RejectReason::DuplicatePeer);
                return;
            }
        }
        if (state_ == SessionState::Racing || state_ == SessionState::Results) {
            SendReject(ev.peer, RejectReason::RaceInProgress);
            return;
        }
        if (static_cast<int>(peers_.size()) + 1 >= cfg_.maxPlayers) {
            SendReject(ev.peer, RejectReason::RoomFull);
            return;
        }
        if (hello->buildSig != cfg_.buildSig) {
            SendReject(ev.peer, RejectReason::BuildMismatch);
            return;
        }
        if (hello->mapHash != cfg_.mapHash) {
            SendReject(ev.peer, RejectReason::MapMismatch);
            return;
        }

        PeerSlot peer;
        peer.id = ev.peer;
        peer.slot = static_cast<std::uint8_t>(peers_.size() + 1);   // slot 0 = host
        peer.name = hello->name.empty() ? ("P" + std::to_string(peer.slot)) : hello->name;
        peer.color = static_cast<std::uint8_t>(peer.slot);
        peers_.push_back(peer);

        WelcomePacket welcome;
        welcome.peerId = peer.id.value;
        welcome.slot = peer.slot;
        welcome.sessionId = 1;
        welcome.rules = cfg_.rules;
        welcome.mapHash = cfg_.mapHash;
        welcome.mapName = cfg_.mapName;
        auto welcomeBytes =
            EncodeToBuffer([&](std::span<std::byte> b) { return EncodeWelcome(welcome, b); });
        if (!welcomeBytes.empty()) {
            (void)transport_.Send(peer.id, Channel::Control, Delivery::Reliable, welcomeBytes);
        }

        SessionEvent join;
        join.kind = SessionEventKind::PeerJoined;
        join.peer = peer.id;
        join.slot = peer.slot;
        join.text = peer.name;
        pending_.push_back(join);

        BroadcastLobby();
        if (state_ == SessionState::Advertising) state_ = SessionState::Lobby;
    }

    void HandleInput(const NetEvent& ev, std::uint64_t nowMs) {
        PeerSlot* peer = nullptr;
        for (PeerSlot& p : peers_) {
            if (p.id == ev.peer) peer = &p;
        }
        if (!peer) return;
        const auto packet = DecodeInput(ev.payload);
        if (!packet) return;
        if (packet->count == 0) return;
        const std::uint16_t newestSeq =
            static_cast<std::uint16_t>(packet->firstSeq - (packet->count - 1));
        if (peer->hasInput && !SeqNewer(newestSeq, peer->lastSeq)) return;
        const InputFrame& f = packet->frames[packet->count - 1];
        peer->latest.throttle = static_cast<float>(f.throttle) / 255.f;
        peer->latest.brake = static_cast<float>(f.brake) / 255.f;
        peer->latest.steer = static_cast<float>(f.steer) / 127.f;
        peer->latest.handbrake = (f.buttons & kBtnHandbrake) != 0;
        peer->latest.reverse = (f.buttons & kBtnReverse) != 0;
        peer->latest.drsPress = packet->drsPress;
        peer->latest.wingPress = packet->wingPress;
        peer->latest.overtakePress = packet->overtakePress;
        peer->lastSeq = newestSeq;
        peer->hasInput = true;
        peer->lastInputMs = nowMs;
    }

    void DropPeer(PeerId id, bool announce) {
        auto it = std::find_if(peers_.begin(), peers_.end(),
                               [&](const PeerSlot& p) { return p.id == id; });
        if (it == peers_.end()) return;
        const std::uint8_t slot = it->slot;
        peers_.erase(it);
        if (announce) {
            SessionEvent left;
            left.kind = SessionEventKind::PeerLeft;
            left.peer = id;
            left.slot = slot;
            pending_.push_back(left);
            PeerLeftPacket p;
            p.peerId = id.value;
            p.reason = 1;
            SendControl([&](std::span<std::byte> b) { return EncodePeerLeft(p, b); });
        }
        BroadcastLobby();
    }

    void SendReject(PeerId id, RejectReason reason) {
        RejectPacket p;
        p.reason = reason;
        auto bytes = EncodeToBuffer([&](std::span<std::byte> b) { return EncodeReject(p, b); });
        if (!bytes.empty()) (void)transport_.Send(id, Channel::Control, Delivery::Reliable, bytes);
    }

    void SendSnapshots() {
        for (const PeerSlot& peer : peers_) {
            SnapshotPacket p = world_.packet;
            p.snapSeq = ++snapSeq_;
            p.appliedInputSeq = appliedSeq_[peer.slot].load();
            if (world_.selfValid[peer.slot] != 0) {
                p.hasSelf = true;
                p.self = world_.self[peer.slot];
            } else {
                p.hasSelf = false;
            }
            std::array<std::byte, kMaxDatagram> buf{};
            const auto n = EncodeSnapshot(p, buf);
            if (!n) continue;
            (void)transport_.Send(peer.id, Channel::State, Delivery::UnreliableSequenced,
                                  std::span<const std::byte>(buf.data(), *n));
        }
    }

    void UpdateAnnouncePhase(std::uint8_t phase) {
        if (state_ == SessionState::Closed) return;
        AnnouncePacket ad;
        ad.roomCode = cfg_.roomCode;
        ad.gamePort = cfg_.gamePort;
        ad.buildSig = cfg_.buildSig;
        ad.mapHash = cfg_.mapHash;
        ad.mapName = cfg_.mapName;
        ad.hostName = cfg_.playerName;
        ad.players = static_cast<std::uint8_t>(peers_.size() + 1);
        ad.maxPlayers = static_cast<std::uint8_t>(cfg_.maxPlayers);
        ad.phase = phase;
        (void)discovery_.Advertise(ad);
    }

    NetConfig cfg_;
    ITransport& transport_;
    IRoomDiscovery& discovery_;
    SessionState state_ = SessionState::Idle;

    std::vector<PeerSlot> peers_;
    HostWorld world_{};
    bool worldValid_ = false;
    std::uint16_t snapSeq_ = 0;
    std::uint64_t lastSnapshotMs_ = 0;
    std::uint64_t lastTickMs_ = 0;
    std::array<std::atomic<std::uint16_t>, kMaxCars> appliedSeq_{};
    std::vector<SessionEvent> pending_;
    std::vector<SessionEvent> drainBuffer_;
};

// -------------------------------------------------------------- client -----

class ClientSession final : public INetSession {
public:
    ClientSession(const NetConfig& cfg, ITransport& transport, IRoomDiscovery& discovery,
                  Endpoint host)
        : cfg_(cfg), transport_(transport), discovery_(discovery), host_(host) {}

    void Start() {
        state_ = SessionState::Joining;
        (void)transport_.Connect(host_);
        joinMs_ = 0;
    }

    void Tick(std::uint64_t nowMs) override {
        if (joinMs_ == 0) joinMs_ = static_cast<std::uint32_t>(nowMs);
        lastPollMs_ = nowMs;
        discovery_.Poll(nowMs);
        for (const NetEvent& ev : transport_.Poll(nowMs)) {
            switch (ev.kind) {
                case NetEventKind::Connected:
                    SendHello();
                    break;
                case NetEventKind::Disconnected:
                case NetEventKind::TimedOut:
                    if (state_ != SessionState::Closed) {
                        SessionEvent e;
                        e.kind = SessionEventKind::HostGone;
                        pending_.push_back(e);
                        state_ = SessionState::Closed;
                    }
                    break;
                case NetEventKind::Received:
                    HandlePacket(ev);
                    break;
            }
        }
        if (state_ == SessionState::Joining && nowMs - joinMs_ > 5000) {
            SessionEvent e;
            e.kind = SessionEventKind::HostGone;
            pending_.push_back(e);
            state_ = SessionState::Closed;
        }
        // Snapshot starvation only means "host gone" once a race is running: in
        // the lobby the host publishes no snapshots by design.
        if ((state_ == SessionState::Racing || state_ == SessionState::Results) &&
            lastSnapshotMs_ != 0 && nowMs - lastSnapshotMs_ > 3000) {
            SessionEvent e;
            e.kind = SessionEventKind::HostGone;
            pending_.push_back(e);
            state_ = SessionState::Closed;
        }
        if ((state_ == SessionState::Racing || state_ == SessionState::Lobby) &&
            nowMs - lastInputSendMs_ >= IntervalMs(cfg_.inputHz)) {
            lastInputSendMs_ = nowMs;
            SendInput();
        }
    }

    std::span<const SessionEvent> DrainEvents() override {
        drainBuffer_ = std::move(pending_);
        pending_.clear();
        return drainBuffer_;
    }

    SessionState State() const override { return state_; }
    bool IsHost() const override { return false; }
    std::uint8_t LocalSlot() const override { return slot_; }

    void SubmitLocalInput(const InputSample& sample) override {
        sample_ = sample;
        const InputFrame frame = ToFrame(sample);
        history_[historyCount_ % 3] = frame;
        ++historyCount_;
        ++seq_;
    }

    const SnapshotPacket* LatestSnapshot() const override {
        return snapshots_.empty() ? nullptr : &snapshots_.back().packet;
    }

    std::span<const TimedSnapshot> RecentSnapshots() const override { return snapshots_; }

    std::uint16_t AppliedInputSeq() const override {
        return snapshots_.empty() ? 0 : snapshots_.back().packet.appliedInputSeq;
    }

    std::uint32_t LatestHostTick() const override { return latestHostTick_; }
    PeerStats LocalStats() const override { return transport_.Stats(PeerId{0}); }
    std::uint32_t JoinMs() const override { return joinMs_; }

private:
    static InputFrame ToFrame(const InputSample& s) {
        InputFrame f;
        f.throttle = static_cast<std::uint8_t>(
            std::clamp(static_cast<int>(s.throttle * 255.f + 0.5f), 0, 255));
        f.brake = static_cast<std::uint8_t>(
            std::clamp(static_cast<int>(s.brake * 255.f + 0.5f), 0, 255));
        f.steer = static_cast<std::int8_t>(
            std::clamp(static_cast<int>(s.steer * 127.f), -127, 127));
        std::uint8_t buttons = 0;
        if (s.handbrake) buttons |= kBtnHandbrake;
        if (s.reverse) buttons |= kBtnReverse;
        f.buttons = buttons;
        return f;
    }

    void SendHello() {
        HelloPacket h;
        h.buildSig = cfg_.buildSig;
        h.mapHash = cfg_.mapHash;
        h.name = cfg_.playerName.substr(0, kNameBytes - 1);
        std::array<std::byte, kMaxDatagram> buf{};
        const auto n = EncodeHello(h, buf);
        if (!n) return;
        (void)transport_.Send(PeerId{0}, Channel::Control, Delivery::Reliable,
                              std::span<const std::byte>(buf.data(), *n));
    }

    void SendInput() {
        InputPacket p;
        p.firstSeq = seq_;
        p.count = static_cast<std::uint8_t>(std::min<std::uint32_t>(3, historyCount_));
        for (std::uint8_t i = 0; i < p.count; ++i) {
            // Oldest first: the newest sample is at (historyCount_ - 1).
            const std::uint32_t idx = historyCount_ - p.count + i;
            p.frames[i] = history_[idx % 3];
        }
        p.drsPress = sample_.drsPress;
        p.wingPress = sample_.wingPress;
        p.overtakePress = sample_.overtakePress;
        p.ackSnapshotSeq = snapshots_.empty() ? 0 : snapshots_.back().packet.snapSeq;
        std::array<std::byte, kMaxDatagram> buf{};
        const auto n = EncodeInput(p, buf);
        if (!n) return;
        (void)transport_.Send(PeerId{0}, Channel::State, Delivery::Unreliable,
                              std::span<const std::byte>(buf.data(), *n));
    }

    void HandlePacket(const NetEvent& ev) {
        const auto type = PeekMessageType(ev.payload);
        if (!type) return;
        switch (*type) {
            case MsgType::Welcome: {
                const auto w = DecodeWelcome(ev.payload);
                if (!w) return;
                slot_ = w->slot;
                rules_ = w->rules;
                sessionId_ = w->sessionId;
                state_ = SessionState::Lobby;
                SessionEvent e;
                e.kind = SessionEventKind::RulesReceived;
                e.slot = slot_;
                e.rules = rules_;
                pending_.push_back(e);
                break;
            }
            case MsgType::Reject: {
                const auto r = DecodeReject(ev.payload);
                if (!r) return;
                SessionEvent e;
                e.kind = SessionEventKind::JoinRejected;
                e.reason = r->reason;
                pending_.push_back(e);
                state_ = SessionState::Closed;
                break;
            }
            case MsgType::LobbyUpdate: {
                const auto l = DecodeLobbyUpdate(ev.payload);
                if (!l) return;
                SessionEvent e;
                e.kind = SessionEventKind::LobbyChanged;
                e.lobby = l->players;
                pending_.push_back(e);
                break;
            }
            case MsgType::Snapshot: {
                const auto s = DecodeSnapshot(ev.payload);
                if (!s) return;
                TimedSnapshot t;
                t.packet = *s;
                t.arrivalMs = lastPollMs_;
                snapshots_.push_back(t);
                if (snapshots_.size() > 32) snapshots_.erase(snapshots_.begin());
                latestHostTick_ = s->hostTick;
                lastSnapshotMs_ = lastPollMs_;
                state_ = PhaseToState(s->phase);
                break;
            }
            case MsgType::RaceStart: {
                const auto r = DecodeRaceStart(ev.payload);
                if (!r) return;
                rules_ = r->rules;
                countdownCs_ = r->countdownCs;
                state_ = SessionState::Racing;
                SessionEvent e;
                e.kind = SessionEventKind::RaceStarted;
                e.rules = rules_;
                e.hostTick = r->startHostTick;
                e.countdownCs = r->countdownCs;
                pending_.push_back(e);
                break;
            }
            case MsgType::Restart: {
                const auto r = DecodeRestart(ev.payload);
                if (!r) return;
                SessionEvent e;
                e.kind = SessionEventKind::Restart;
                e.hostTick = r->startHostTick;
                e.countdownCs = r->countdownCs;
                pending_.push_back(e);
                state_ = SessionState::Racing;
                break;
            }
            case MsgType::LapComplete: {
                const auto l = DecodeLapComplete(ev.payload);
                if (!l) return;
                SessionEvent e;
                e.kind = SessionEventKind::LapComplete;
                e.slot = l->slot;
                e.value = l->lap;
                e.ms = l->lapTimeMs;
                e.bestMs = l->bestLapMs;
                pending_.push_back(e);
                break;
            }
            case MsgType::PlayerFinished: {
                const auto f = DecodePlayerFinished(ev.payload);
                if (!f) return;
                SessionEvent e;
                e.kind = SessionEventKind::PlayerFinished;
                e.slot = f->slot;
                e.value = f->position;
                e.ms = f->raceTimeMs;
                e.bestMs = f->bestLapMs;
                pending_.push_back(e);
                break;
            }
            case MsgType::RaceEnd: {
                const auto r = DecodeRaceEnd(ev.payload);
                if (!r) return;
                SessionEvent e;
                e.kind = SessionEventKind::RaceEnded;
                e.results = r->entries;
                pending_.push_back(e);
                state_ = SessionState::Results;
                break;
            }
            case MsgType::CarRecovered: {
                const auto c = DecodeCarRecovered(ev.payload);
                if (!c) return;
                SessionEvent e;
                e.kind = SessionEventKind::CarRecovered;
                e.slot = c->slot;
                e.value = c->reason;
                pending_.push_back(e);
                break;
            }
            case MsgType::PartDamaged: {
                const auto d = DecodePartDamaged(ev.payload);
                if (!d) return;
                SessionEvent e;
                e.kind = SessionEventKind::PartDamaged;
                e.slot = d->slot;
                e.value = d->part;
                e.ms = d->impactKmh;
                pending_.push_back(e);
                break;
            }
            case MsgType::PeerLeft: {
                const auto p = DecodePeerLeft(ev.payload);
                if (!p) return;
                SessionEvent e;
                e.kind = SessionEventKind::PeerLeft;
                e.peer = PeerId{p->peerId};
                pending_.push_back(e);
                break;
            }
            case MsgType::SessionEnd: {
                SessionEvent e;
                e.kind = SessionEventKind::SessionEnded;
                pending_.push_back(e);
                state_ = SessionState::Closed;
                break;
            }
            default:
                break;
        }
    }

    static SessionState PhaseToState(std::uint8_t phase) {
        switch (static_cast<NetPhase>(phase)) {
            case NetPhase::Lobby:
                return SessionState::Lobby;
            case NetPhase::Countdown:
            case NetPhase::Racing:
                return SessionState::Racing;
            case NetPhase::Finished:
                return SessionState::Results;
        }
        return SessionState::Racing;
    }

    NetConfig cfg_;
    ITransport& transport_;
    IRoomDiscovery& discovery_;
    Endpoint host_{};
    SessionState state_ = SessionState::Searching;
    std::uint8_t slot_ = 0;
    std::uint32_t sessionId_ = 0;
    RulesPacket rules_{};
    std::uint16_t countdownCs_ = 350;

    InputSample sample_{};
    InputFrame history_[3]{};
    std::uint32_t historyCount_ = 0;
    std::uint16_t seq_ = 0;
    std::uint64_t lastInputSendMs_ = 0;
    std::uint64_t lastSnapshotMs_ = 0;
    std::uint64_t lastPollMs_ = 0;
    std::uint32_t latestHostTick_ = 0;
    std::uint32_t joinMs_ = 0;
    std::vector<TimedSnapshot> snapshots_;
    std::vector<SessionEvent> pending_;
    std::vector<SessionEvent> drainBuffer_;
};

}  // namespace

std::unique_ptr<INetSession> CreateHostSession(const NetConfig& config, ITransport& transport,
                                               IRoomDiscovery& discovery) {
    auto session = std::make_unique<HostSession>(config, transport, discovery);
    session->Start();
    return session;
}

std::unique_ptr<INetSession> CreateClientSession(const NetConfig& config, ITransport& transport,
                                                 IRoomDiscovery& discovery, Endpoint host) {
    auto session = std::make_unique<ClientSession>(config, transport, discovery, host);
    session->Start();
    return session;
}

}  // namespace apex::net
