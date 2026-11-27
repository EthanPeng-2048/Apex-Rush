// Wire codec: pure functions, no IO. See docs/online/protocol.md.

#include "net/protocol/wire.h"

#include "net/protocol/wire_quantize.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace apex::net {
namespace {

using q::kDegToRad;
using q::kRadToDeg;
using q::kTwoPi;
using q::Safe;

// ---------------------------------------------------------------- byte IO ---

class Writer {
public:
    explicit Writer(std::span<std::byte> out) : out_(out) {}

    bool ok() const { return ok_; }
    std::size_t size() const { return n_; }

    void u8(std::uint8_t v) {
        if (!ok_) return;
        if (n_ >= out_.size()) {
            ok_ = false;
            return;
        }
        out_[n_++] = static_cast<std::byte>(v);
    }
    void i8(std::int8_t v) { u8(static_cast<std::uint8_t>(v)); }
    void u16(std::uint16_t v) {
        u8(static_cast<std::uint8_t>(v));
        u8(static_cast<std::uint8_t>(v >> 8));
    }
    void i16(std::int16_t v) { u16(static_cast<std::uint16_t>(v)); }
    void u32(std::uint32_t v) {
        u16(static_cast<std::uint16_t>(v));
        u16(static_cast<std::uint16_t>(v >> 16));
    }
    // Little-endian by construction (two u32 halves), like every other field.
    void u64(std::uint64_t v) {
        u32(static_cast<std::uint32_t>(v));
        u32(static_cast<std::uint32_t>(v >> 32));
    }
    void f32(float f) {
        std::uint32_t bits = 0;
        std::memcpy(&bits, &f, sizeof(bits));
        u32(bits);
    }
    void bytes(const void* p, std::size_t n) {
        const auto* b = static_cast<const std::byte*>(p);
        for (std::size_t i = 0; i < n; ++i) u8(static_cast<std::uint8_t>(b[i]));
    }
    void fixedStr(const std::string& s, std::size_t n) {
        const std::size_t copy = std::min(n > 0 ? n - 1 : 0, s.size());
        for (std::size_t i = 0; i < copy; ++i) u8(static_cast<std::uint8_t>(s[i]));
        for (std::size_t i = copy; i < n; ++i) u8(0);
    }

private:
    std::span<std::byte> out_;
    std::size_t n_ = 0;
    bool ok_ = true;
};

class Reader {
public:
    explicit Reader(std::span<const std::byte> in) : in_(in) {}

    bool ok() const { return ok_; }
    bool done() const { return ok_ && i_ == in_.size(); }
    std::size_t remaining() const { return i_ <= in_.size() ? in_.size() - i_ : 0; }

    std::uint8_t u8() {
        if (!ok_ || i_ >= in_.size()) {
            ok_ = false;
            return 0;
        }
        return static_cast<std::uint8_t>(in_[i_++]);
    }
    std::int8_t i8() { return static_cast<std::int8_t>(u8()); }
    std::uint16_t u16() {
        const std::uint16_t lo = u8();
        const std::uint16_t hi = u8();
        return static_cast<std::uint16_t>(lo | (hi << 8));
    }
    std::int16_t i16() { return static_cast<std::int16_t>(u16()); }
    std::uint32_t u32() {
        const std::uint32_t lo = u16();
        const std::uint32_t hi = u16();
        return lo | (hi << 16);
    }
    std::uint64_t u64() {
        const std::uint64_t lo = u32();
        const std::uint64_t hi = u32();
        return lo | (hi << 32);
    }
    float f32() {
        const std::uint32_t bits = u32();
        if (!std::isfinite(*reinterpret_cast<const float*>(&bits))) ok_ = false;
        float f = 0.f;
        std::memcpy(&f, &bits, sizeof(f));
        return f;
    }
    std::string fixedStr(std::size_t n) {
        std::string s;
        s.reserve(n);
        for (std::size_t i = 0; i < n; ++i) {
            const char c = static_cast<char>(u8());
            if (c != '\0' && s.size() == i) s.push_back(c);
        }
        return s;
    }

private:
    std::span<const std::byte> in_;
    std::size_t i_ = 0;
    bool ok_ = true;
};

void WriteHeader(Writer& w, MsgType type) {
    w.u8(kProtocolVersion);
    w.u8(static_cast<std::uint8_t>(type));
}

// Reads header, validates version + expected type.
bool ReadHeader(Reader& r, MsgType expect) {
    const std::uint8_t ver = r.u8();
    const std::uint8_t type = r.u8();
    return r.ok() && ver == kProtocolVersion && type == static_cast<std::uint8_t>(expect);
}

// -------------------------------------------------------------- quantizers --

constexpr float kPosScale = 32.f;       // 1/32 m
constexpr float kVelScale = 64.f;       // 1/64 m/s
constexpr float kYawRateScale = 16.f;   // 1/16 deg/s
constexpr float kAngRateScale = 1.f;    // 1 deg/s
constexpr float kSteerScale = 2.f;      // 0.5 deg
constexpr float kSpeedScale = 20.f;     // 0.05 km/h
constexpr float kRpmScale = 1.f;
constexpr float kPitchRollScale = 4.f;  // 0.25 deg
constexpr float kUnitScale = 255.f;
constexpr float kSlipRatioScale = 127.f;
constexpr float kSlipAngleScale = 2.f;   // 0.5 deg
constexpr float kWheelSpinScale = 16.f;  // 1/16 rad/s

void WriteCar(Writer& w, const CarNetState& c, const float anchor[3]) {
    w.u8(c.slot);
    w.i16(q::ToI16(Safe(c.pos[0] - anchor[0]), kPosScale));
    w.i16(q::ToI16(Safe(c.pos[1] - anchor[1]), kPosScale));
    w.i16(q::ToI16(Safe(c.pos[2] - anchor[2]), kPosScale));

    constexpr float kYawSteps = 65536.f / kTwoPi;
    w.u16(static_cast<std::uint16_t>(
        static_cast<std::uint32_t>(q::WrapYaw(Safe(c.yaw)) * kYawSteps) & 0xFFFFu));
    w.i8(q::ToI8(Safe(c.pitch) * kRadToDeg, kPitchRollScale));
    w.i8(q::ToI8(Safe(c.roll) * kRadToDeg, kPitchRollScale));

    w.i16(q::ToI16(Safe(c.vel[0]), kVelScale));
    w.i16(q::ToI16(Safe(c.vel[1]), kVelScale));
    w.i16(q::ToI16(Safe(c.vel[2]), kVelScale));

    w.i16(q::ToI16(Safe(c.yawRate) * kRadToDeg, kYawRateScale));
    w.i8(q::ToI8(Safe(c.pitchRate) * kRadToDeg, kAngRateScale));
    w.i8(q::ToI8(Safe(c.rollRate) * kRadToDeg, kAngRateScale));

    w.i8(q::ToI8(Safe(c.steerDeg), kSteerScale));
    w.u16(q::ToU16(Safe(c.speedKmh), kSpeedScale));
    w.u16(q::ToU16(Safe(c.rpm), kRpmScale));

    std::uint8_t flags1 = static_cast<std::uint8_t>(c.gear & 0x0Fu);
    flags1 |= static_cast<std::uint8_t>((c.wingMode & 0x01u) << 4);
    flags1 |= static_cast<std::uint8_t>((c.drsActive ? 1u : 0u) << 5);
    flags1 |= static_cast<std::uint8_t>((c.ersDeploying ? 1u : 0u) << 6);
    flags1 |= static_cast<std::uint8_t>((c.finished ? 1u : 0u) << 7);
    w.u8(flags1);

    w.u8(q::ToU8(Safe(c.ersCharge), kUnitScale));

    std::uint8_t flags2 = static_cast<std::uint8_t>(c.groundedMask & 0x0Fu);
    flags2 |= static_cast<std::uint8_t>((c.drsAvailable ? 1u : 0u) << 4);
    flags2 |= static_cast<std::uint8_t>((c.retired ? 1u : 0u) << 5);
    flags2 |= static_cast<std::uint8_t>((c.onTrack ? 1u : 0u) << 6);
    w.u8(flags2);

    w.u8(c.lap);
    w.u8(c.position);
    for (float t : c.wheelTravel) w.u8(q::ToU8(Safe(t), kUnitScale));
}

CarNetState ReadCar(Reader& r, const float anchor[3]) {
    CarNetState c;
    constexpr float kYawSteps = 65536.f / kTwoPi;
    c.slot = r.u8();
    c.pos[0] = anchor[0] + q::From(r.i16(), kPosScale);
    c.pos[1] = anchor[1] + q::From(r.i16(), kPosScale);
    c.pos[2] = anchor[2] + q::From(r.i16(), kPosScale);
    c.yaw = q::WrapYaw(r.u16() / kYawSteps);
    c.pitch = q::From(r.i8(), kPitchRollScale) * kDegToRad;
    c.roll = q::From(r.i8(), kPitchRollScale) * kDegToRad;
    c.vel[0] = q::From(r.i16(), kVelScale);
    c.vel[1] = q::From(r.i16(), kVelScale);
    c.vel[2] = q::From(r.i16(), kVelScale);
    c.yawRate = q::From(r.i16(), kYawRateScale) * kDegToRad;
    c.pitchRate = q::From(r.i8(), kAngRateScale) * kDegToRad;
    c.rollRate = q::From(r.i8(), kAngRateScale) * kDegToRad;
    c.steerDeg = q::From(r.i8(), kSteerScale);
    c.speedKmh = q::From(r.u16(), kSpeedScale);
    c.rpm = q::From(r.u16(), kRpmScale);
    const std::uint8_t flags1 = r.u8();
    c.gear = static_cast<std::uint8_t>(flags1 & 0x0Fu);
    c.wingMode = static_cast<std::uint8_t>((flags1 >> 4) & 0x01u);
    c.drsActive = ((flags1 >> 5) & 1u) != 0;
    c.ersDeploying = ((flags1 >> 6) & 1u) != 0;
    c.finished = ((flags1 >> 7) & 1u) != 0;
    c.ersCharge = q::From(r.u8(), kUnitScale);
    const std::uint8_t flags2 = r.u8();
    c.groundedMask = static_cast<std::uint8_t>(flags2 & 0x0Fu);
    c.drsAvailable = ((flags2 >> 4) & 1u) != 0;
    c.retired = ((flags2 >> 5) & 1u) != 0;
    c.onTrack = ((flags2 >> 6) & 1u) != 0;
    c.lap = r.u8();
    c.position = r.u8();
    for (float& t : c.wheelTravel) t = q::From(r.u8(), kUnitScale);
    return c;
}

void WriteSelf(Writer& w, const SelfState& s) {
    for (float v : s.slipRatio) w.i8(q::ToI8(Safe(v), kSlipRatioScale));
    for (float v : s.slipAngleDeg) w.i8(q::ToI8(Safe(v), kSlipAngleScale));
    for (float v : s.wheelSpinRate) w.i16(q::ToI16(Safe(v), kWheelSpinScale));
    for (float v : s.wear) w.u8(q::ToU8(Safe(v), kUnitScale));
    for (float v : s.tempC) w.u8(q::ToU8(Safe(v), 1.f));
    for (float v : s.partHealth) w.u8(q::ToU8(Safe(v), kUnitScale));
    w.u8(q::ToU8(Safe(s.damageTotal), kUnitScale));
    w.u8(s.shiftCooldownTicks);
    w.u8(s.ersDeployTicks);
    w.u32(s.rngState);
}

SelfState ReadSelf(Reader& r) {
    SelfState s;
    for (float& v : s.slipRatio) v = q::From(r.i8(), kSlipRatioScale);
    for (float& v : s.slipAngleDeg) v = q::From(r.i8(), kSlipAngleScale);
    for (float& v : s.wheelSpinRate) v = q::From(r.i16(), kWheelSpinScale);
    for (float& v : s.wear) v = q::From(r.u8(), kUnitScale);
    for (float& v : s.tempC) v = q::From(r.u8(), 1.f);
    for (float& v : s.partHealth) v = q::From(r.u8(), kUnitScale);
    s.damageTotal = q::From(r.u8(), kUnitScale);
    s.shiftCooldownTicks = r.u8();
    s.ersDeployTicks = r.u8();
    s.rngState = r.u32();
    return s;
}

void WriteRules(Writer& w, const RulesPacket& r) {
    w.u8(r.laps);
    w.u8(r.carCount);
    w.u8(r.damage);
    w.u8(r.seasonBits);
    w.u8(r.absLevel);
    w.u8(r.tcLevel);
    w.u8(r.escLevel);
    w.u8(r.steerLevel);
    w.u8(r.physicsEngine);
    w.u8(r.snapshotHz);
    // Roster: fixed kMaxCars records (25 B each) so Rules keeps one size no
    // matter how many are seated and the decoder needs no length trust.
    w.u8(r.roster.count);
    for (int i = 0; i < kMaxCars; ++i) {
        const RosterEntry& e = r.roster.entries[static_cast<std::size_t>(i)];
        w.u64(e.playerId);
        w.u8(e.seat);
        w.fixedStr(e.name, kNameBytes);
    }
}

RulesPacket ReadRules(Reader& r) {
    RulesPacket p;
    p.laps = r.u8();
    p.carCount = r.u8();
    p.damage = r.u8();
    p.seasonBits = r.u8();
    p.absLevel = r.u8();
    p.tcLevel = r.u8();
    p.escLevel = r.u8();
    p.steerLevel = r.u8();
    p.physicsEngine = r.u8();
    p.snapshotHz = r.u8();
    p.roster.count = r.u8();
    for (int i = 0; i < kMaxCars; ++i) {
        RosterEntry& e = p.roster.entries[static_cast<std::size_t>(i)];
        e.playerId = r.u64();
        e.seat = r.u8();
        e.name = r.fixedStr(kNameBytes);
    }
    return p;
}

}  // namespace

// ------------------------------------------------------------ ypr <-> quat ---

namespace q {

void QuatToYpr(float qx, float qy, float qz, float qw, float& yaw, float& pitch, float& roll) {
    const float r02 = 2.f * (qx * qz + qw * qy);
    const float r22 = 1.f - 2.f * (qx * qx + qy * qy);
    const float r12 = 2.f * (qy * qz - qw * qx);
    const float r10 = 2.f * (qx * qy + qw * qz);
    const float r11 = 1.f - 2.f * (qx * qx + qz * qz);
    pitch = std::asin(Clamp(-r12, -1.f, 1.f));
    yaw = std::atan2(r02, r22);
    roll = std::atan2(r10, r11);
}

void YprToQuat(float yaw, float pitch, float roll, float& qx, float& qy, float& qz, float& qw) {
    const float cy = std::cos(yaw * 0.5f), sy = std::sin(yaw * 0.5f);
    const float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
    const float cr = std::cos(roll * 0.5f), sr = std::sin(roll * 0.5f);
    // q = qy * qx * qz
    const float x1 = sp, y1 = 0.f, z1 = 0.f, w1 = cp;      // pitch about X
    const float x2 = 0.f, y2 = sy, z2 = 0.f, w2 = cy;      // yaw about Y
    const float x3 = 0.f, y3 = 0.f, z3 = sr, w3 = cr;      // roll about Z
    // (qy * qx)
    const float ax = w2 * x1 + x2 * w1 + y2 * z1 - z2 * y1;
    const float ay = w2 * y1 - x2 * z1 + y2 * w1 + z2 * x1;
    const float az = w2 * z1 + x2 * y1 - y2 * x1 + z2 * w1;
    const float aw = w2 * w1 - x2 * x1 - y2 * y1 - z2 * z1;
    // (...) * qz
    qx = aw * x3 + ax * w3 + ay * z3 - az * y3;
    qy = aw * y3 - ax * z3 + ay * w3 + az * x3;
    qz = aw * z3 + ax * y3 - ay * x3 + az * w3;
    qw = aw * w3 - ax * x3 - ay * y3 - az * z3;
}

}  // namespace q

// ------------------------------------------------------------------ input ---

std::expected<std::size_t, NetError> EncodeInput(const InputPacket& p, std::span<std::byte> out) {
    Writer w(out);
    WriteHeader(w, MsgType::Input);
    w.u16(p.firstSeq);
    const std::uint8_t count = static_cast<std::uint8_t>(std::min<std::size_t>(p.count, 3));
    w.u8(count);
    for (std::uint8_t i = 0; i < count; ++i) {
        const InputFrame& f = p.frames[i];
        w.u8(f.throttle);
        w.u8(f.brake);
        w.i8(f.steer);
        w.u8(f.buttons);
    }
    // Unused redundancy slots are zero-filled so the packet stays fixed size.
    for (std::uint8_t i = count; i < 3; ++i) {
        w.u32(0);
    }
    w.u8(p.drsPress);
    w.u8(p.wingPress);
    w.u8(p.overtakePress);
    w.u16(p.ackSnapshotSeq);
    if (!w.ok()) return std::unexpected(NetError::PayloadTooLarge);
    return w.size();
}

std::expected<InputPacket, NetError> DecodeInput(std::span<const std::byte> in) {
    Reader r(in);
    if (!ReadHeader(r, MsgType::Input)) return std::unexpected(NetError::Malformed);
    InputPacket p;
    p.firstSeq = r.u16();
    p.count = r.u8();
    if (p.count > 3) return std::unexpected(NetError::Malformed);
    for (std::uint8_t i = 0; i < 3; ++i) {
        InputFrame f;
        f.throttle = r.u8();
        f.brake = r.u8();
        f.steer = r.i8();
        f.buttons = r.u8();
        if (i < p.count) p.frames[i] = f;
    }
    p.drsPress = r.u8();
    p.wingPress = r.u8();
    p.overtakePress = r.u8();
    p.ackSnapshotSeq = r.u16();
    if (!r.done()) return std::unexpected(NetError::Malformed);
    return p;
}

// --------------------------------------------------------------- snapshot ---

std::expected<std::size_t, NetError> EncodeSnapshot(const SnapshotPacket& p, std::span<std::byte> out) {
    Writer w(out);
    WriteHeader(w, MsgType::Snapshot);
    w.u32(p.hostTick);
    w.u16(p.snapSeq);
    w.u8(p.phase);
    w.u16(p.phaseTimerCs);
    w.u32(p.raceClockMs);
    const std::uint8_t carCount = static_cast<std::uint8_t>(std::min<int>(p.carCount, kMaxCars));
    w.u8(carCount);
    w.f32(Safe(p.anchor[0]));
    w.f32(Safe(p.anchor[1]));
    w.f32(Safe(p.anchor[2]));
    w.u16(p.appliedInputSeq);
    for (std::uint8_t i = 0; i < carCount; ++i) WriteCar(w, p.cars[i], p.anchor);
    if (p.hasSelf) WriteSelf(w, p.self);
    if (!w.ok()) return std::unexpected(NetError::PayloadTooLarge);
    return w.size();
}

std::expected<SnapshotPacket, NetError> DecodeSnapshot(std::span<const std::byte> in) {
    Reader r(in);
    if (!ReadHeader(r, MsgType::Snapshot)) return std::unexpected(NetError::Malformed);
    SnapshotPacket p;
    p.hostTick = r.u32();
    p.snapSeq = r.u16();
    p.phase = r.u8();
    p.phaseTimerCs = r.u16();
    p.raceClockMs = r.u32();
    p.carCount = r.u8();
    if (p.carCount > kMaxCars) return std::unexpected(NetError::Malformed);
    p.anchor[0] = r.f32();
    p.anchor[1] = r.f32();
    p.anchor[2] = r.f32();
    p.appliedInputSeq = r.u16();
    if (!r.ok()) return std::unexpected(NetError::Malformed);
    for (std::uint8_t i = 0; i < p.carCount; ++i) {
        p.cars[i] = ReadCar(r, p.anchor);
        if (p.cars[i].slot >= kMaxCars) return std::unexpected(NetError::Malformed);
    }
    const std::size_t rest = r.remaining();
    if (rest == 35) {
        p.hasSelf = true;
        p.self = ReadSelf(r);
    } else if (rest != 0) {
        return std::unexpected(NetError::Malformed);
    }
    if (!r.done()) return std::unexpected(NetError::Malformed);
    return p;
}

// ---------------------------------------------------------------- control ---

std::expected<std::size_t, NetError> EncodeHello(const HelloPacket& p, std::span<std::byte> out) {
    Writer w(out);
    WriteHeader(w, MsgType::Hello);
    w.u32(p.buildSig);
    w.u32(p.mapHash);
    w.fixedStr(p.name, kNameBytes);
    w.u8(p.colorPref);
    w.u64(p.playerId);
    if (!w.ok()) return std::unexpected(NetError::PayloadTooLarge);
    return w.size();
}

std::expected<HelloPacket, NetError> DecodeHello(std::span<const std::byte> in) {
    Reader r(in);
    if (!ReadHeader(r, MsgType::Hello)) return std::unexpected(NetError::Malformed);
    HelloPacket p;
    p.buildSig = r.u32();
    p.mapHash = r.u32();
    p.name = r.fixedStr(kNameBytes);
    p.colorPref = r.u8();
    p.playerId = r.u64();
    if (!r.done()) return std::unexpected(NetError::Malformed);
    return p;
}

std::expected<std::size_t, NetError> EncodeWelcome(const WelcomePacket& p, std::span<std::byte> out) {
    Writer w(out);
    WriteHeader(w, MsgType::Welcome);
    w.u8(p.peerId);
    w.u8(p.slot);
    w.u32(p.sessionId);
    WriteRules(w, p.rules);
    w.u32(p.mapHash);
    w.fixedStr(p.mapName, kMapNameBytes);
    w.f32(Safe(p.gridPos[0]));
    w.f32(Safe(p.gridPos[1]));
    w.f32(Safe(p.gridPos[2]));
    w.u16(p.gridYaw);
    w.u64(p.playerId);
    if (!w.ok()) return std::unexpected(NetError::PayloadTooLarge);
    return w.size();
}

std::expected<WelcomePacket, NetError> DecodeWelcome(std::span<const std::byte> in) {
    Reader r(in);
    if (!ReadHeader(r, MsgType::Welcome)) return std::unexpected(NetError::Malformed);
    WelcomePacket p;
    p.peerId = r.u8();
    p.slot = r.u8();
    p.sessionId = r.u32();
    p.rules = ReadRules(r);
    p.mapHash = r.u32();
    p.mapName = r.fixedStr(kMapNameBytes);
    p.gridPos[0] = r.f32();
    p.gridPos[1] = r.f32();
    p.gridPos[2] = r.f32();
    p.gridYaw = r.u16();
    p.playerId = r.u64();
    if (p.rules.roster.count > kMaxCars) return std::unexpected(NetError::Malformed);
    if (!r.done()) return std::unexpected(NetError::Malformed);
    return p;
}

std::expected<std::size_t, NetError> EncodeReject(const RejectPacket& p, std::span<std::byte> out) {
    Writer w(out);
    WriteHeader(w, MsgType::Reject);
    w.u8(static_cast<std::uint8_t>(p.reason));
    w.u8(p.detail);
    if (!w.ok()) return std::unexpected(NetError::PayloadTooLarge);
    return w.size();
}

std::expected<RejectPacket, NetError> DecodeReject(std::span<const std::byte> in) {
    Reader r(in);
    if (!ReadHeader(r, MsgType::Reject)) return std::unexpected(NetError::Malformed);
    RejectPacket p;
    p.reason = static_cast<RejectReason>(r.u8());
    p.detail = r.u8();
    if (!r.done()) return std::unexpected(NetError::Malformed);
    return p;
}

std::expected<std::size_t, NetError> EncodeLobbyUpdate(const LobbyUpdatePacket& p, std::span<std::byte> out) {
    Writer w(out);
    WriteHeader(w, MsgType::LobbyUpdate);
    const std::size_t n = std::min<std::size_t>(p.players.size(), kMaxCars);
    w.u8(static_cast<std::uint8_t>(n));
    for (std::size_t i = 0; i < n; ++i) {
        const LobbyPlayer& pl = p.players[i];
        w.u8(pl.peerId);
        w.u8(pl.slot);
        w.fixedStr(pl.name, kNameBytes);
        w.u8(pl.color);
        w.u8(pl.ready);
    }
    if (!w.ok()) return std::unexpected(NetError::PayloadTooLarge);
    return w.size();
}

std::expected<LobbyUpdatePacket, NetError> DecodeLobbyUpdate(std::span<const std::byte> in) {
    Reader r(in);
    if (!ReadHeader(r, MsgType::LobbyUpdate)) return std::unexpected(NetError::Malformed);
    LobbyUpdatePacket p;
    const std::uint8_t n = r.u8();
    if (n > kMaxCars) return std::unexpected(NetError::Malformed);
    for (std::uint8_t i = 0; i < n; ++i) {
        LobbyPlayer pl;
        pl.peerId = r.u8();
        pl.slot = r.u8();
        pl.name = r.fixedStr(kNameBytes);
        pl.color = r.u8();
        pl.ready = r.u8();
        p.players.push_back(std::move(pl));
    }
    if (!r.done()) return std::unexpected(NetError::Malformed);
    return p;
}

std::expected<std::size_t, NetError> EncodeRaceStart(const RaceStartPacket& p, std::span<std::byte> out) {
    Writer w(out);
    WriteHeader(w, MsgType::RaceStart);
    w.u32(p.startHostTick);
    w.u16(p.countdownCs);
    WriteRules(w, p.rules);
    if (!w.ok()) return std::unexpected(NetError::PayloadTooLarge);
    return w.size();
}

std::expected<RaceStartPacket, NetError> DecodeRaceStart(std::span<const std::byte> in) {
    Reader r(in);
    if (!ReadHeader(r, MsgType::RaceStart)) return std::unexpected(NetError::Malformed);
    RaceStartPacket p;
    p.startHostTick = r.u32();
    p.countdownCs = r.u16();
    p.rules = ReadRules(r);
    if (p.rules.roster.count > kMaxCars) return std::unexpected(NetError::Malformed);
    if (!r.done()) return std::unexpected(NetError::Malformed);
    return p;
}

std::expected<std::size_t, NetError> EncodeLapComplete(const LapCompletePacket& p, std::span<std::byte> out) {
    Writer w(out);
    WriteHeader(w, MsgType::LapComplete);
    w.u8(p.slot);
    w.u8(p.lap);
    w.u32(p.lapTimeMs);
    w.u32(p.bestLapMs);
    if (!w.ok()) return std::unexpected(NetError::PayloadTooLarge);
    return w.size();
}

std::expected<LapCompletePacket, NetError> DecodeLapComplete(std::span<const std::byte> in) {
    Reader r(in);
    if (!ReadHeader(r, MsgType::LapComplete)) return std::unexpected(NetError::Malformed);
    LapCompletePacket p;
    p.slot = r.u8();
    p.lap = r.u8();
    p.lapTimeMs = r.u32();
    p.bestLapMs = r.u32();
    if (!r.done()) return std::unexpected(NetError::Malformed);
    return p;
}

std::expected<std::size_t, NetError> EncodePlayerFinished(const PlayerFinishedPacket& p, std::span<std::byte> out) {
    Writer w(out);
    WriteHeader(w, MsgType::PlayerFinished);
    w.u8(p.slot);
    w.u8(p.position);
    w.u32(p.raceTimeMs);
    w.u32(p.bestLapMs);
    if (!w.ok()) return std::unexpected(NetError::PayloadTooLarge);
    return w.size();
}

std::expected<PlayerFinishedPacket, NetError> DecodePlayerFinished(std::span<const std::byte> in) {
    Reader r(in);
    if (!ReadHeader(r, MsgType::PlayerFinished)) return std::unexpected(NetError::Malformed);
    PlayerFinishedPacket p;
    p.slot = r.u8();
    p.position = r.u8();
    p.raceTimeMs = r.u32();
    p.bestLapMs = r.u32();
    if (!r.done()) return std::unexpected(NetError::Malformed);
    return p;
}

std::expected<std::size_t, NetError> EncodeRaceEnd(const RaceEndPacket& p, std::span<std::byte> out) {
    Writer w(out);
    WriteHeader(w, MsgType::RaceEnd);
    const std::size_t n = std::min<std::size_t>(p.entries.size(), kMaxCars);
    w.u8(static_cast<std::uint8_t>(n));
    for (std::size_t i = 0; i < n; ++i) {
        const RaceEndEntry& e = p.entries[i];
        w.u8(e.slot);
        w.u8(e.position);
        w.u8(e.finished);
        w.u32(e.raceTimeMs);
        w.u32(e.bestLapMs);
    }
    if (!w.ok()) return std::unexpected(NetError::PayloadTooLarge);
    return w.size();
}

std::expected<RaceEndPacket, NetError> DecodeRaceEnd(std::span<const std::byte> in) {
    Reader r(in);
    if (!ReadHeader(r, MsgType::RaceEnd)) return std::unexpected(NetError::Malformed);
    RaceEndPacket p;
    const std::uint8_t n = r.u8();
    if (n > kMaxCars) return std::unexpected(NetError::Malformed);
    for (std::uint8_t i = 0; i < n; ++i) {
        RaceEndEntry e;
        e.slot = r.u8();
        e.position = r.u8();
        e.finished = r.u8();
        e.raceTimeMs = r.u32();
        e.bestLapMs = r.u32();
        p.entries.push_back(e);
    }
    if (!r.done()) return std::unexpected(NetError::Malformed);
    return p;
}

std::expected<std::size_t, NetError> EncodeCarRecovered(const CarRecoveredPacket& p, std::span<std::byte> out) {
    Writer w(out);
    WriteHeader(w, MsgType::CarRecovered);
    w.u8(p.slot);
    w.u8(p.reason);
    if (!w.ok()) return std::unexpected(NetError::PayloadTooLarge);
    return w.size();
}

std::expected<CarRecoveredPacket, NetError> DecodeCarRecovered(std::span<const std::byte> in) {
    Reader r(in);
    if (!ReadHeader(r, MsgType::CarRecovered)) return std::unexpected(NetError::Malformed);
    CarRecoveredPacket p;
    p.slot = r.u8();
    p.reason = r.u8();
    if (!r.done()) return std::unexpected(NetError::Malformed);
    return p;
}

std::expected<std::size_t, NetError> EncodePartDamaged(const PartDamagedPacket& p, std::span<std::byte> out) {
    Writer w(out);
    WriteHeader(w, MsgType::PartDamaged);
    w.u8(p.slot);
    w.u8(p.part);
    w.u8(p.health);
    w.u8(p.impactKmh);
    if (!w.ok()) return std::unexpected(NetError::PayloadTooLarge);
    return w.size();
}

std::expected<PartDamagedPacket, NetError> DecodePartDamaged(std::span<const std::byte> in) {
    Reader r(in);
    if (!ReadHeader(r, MsgType::PartDamaged)) return std::unexpected(NetError::Malformed);
    PartDamagedPacket p;
    p.slot = r.u8();
    p.part = r.u8();
    p.health = r.u8();
    p.impactKmh = r.u8();
    if (!r.done()) return std::unexpected(NetError::Malformed);
    return p;
}

std::expected<std::size_t, NetError> EncodePeerLeft(const PeerLeftPacket& p, std::span<std::byte> out) {
    Writer w(out);
    WriteHeader(w, MsgType::PeerLeft);
    w.u8(p.peerId);
    w.u8(p.reason);
    if (!w.ok()) return std::unexpected(NetError::PayloadTooLarge);
    return w.size();
}

std::expected<PeerLeftPacket, NetError> DecodePeerLeft(std::span<const std::byte> in) {
    Reader r(in);
    if (!ReadHeader(r, MsgType::PeerLeft)) return std::unexpected(NetError::Malformed);
    PeerLeftPacket p;
    p.peerId = r.u8();
    p.reason = r.u8();
    if (!r.done()) return std::unexpected(NetError::Malformed);
    return p;
}

std::expected<std::size_t, NetError> EncodeSessionEnd(const SessionEndPacket& p, std::span<std::byte> out) {
    Writer w(out);
    WriteHeader(w, MsgType::SessionEnd);
    w.u8(p.reason);
    if (!w.ok()) return std::unexpected(NetError::PayloadTooLarge);
    return w.size();
}

std::expected<SessionEndPacket, NetError> DecodeSessionEnd(std::span<const std::byte> in) {
    Reader r(in);
    if (!ReadHeader(r, MsgType::SessionEnd)) return std::unexpected(NetError::Malformed);
    SessionEndPacket p;
    p.reason = r.u8();
    if (!r.done()) return std::unexpected(NetError::Malformed);
    return p;
}

std::expected<std::size_t, NetError> EncodeRestart(const RestartPacket& p, std::span<std::byte> out) {
    Writer w(out);
    WriteHeader(w, MsgType::Restart);
    w.u32(p.startHostTick);
    w.u16(p.countdownCs);
    if (!w.ok()) return std::unexpected(NetError::PayloadTooLarge);
    return w.size();
}

std::expected<RestartPacket, NetError> DecodeRestart(std::span<const std::byte> in) {
    Reader r(in);
    if (!ReadHeader(r, MsgType::Restart)) return std::unexpected(NetError::Malformed);
    RestartPacket p;
    p.startHostTick = r.u32();
    p.countdownCs = r.u16();
    if (!r.done()) return std::unexpected(NetError::Malformed);
    return p;
}

std::expected<std::size_t, NetError> EncodeBye(std::span<std::byte> out) {
    Writer w(out);
    WriteHeader(w, MsgType::Bye);
    if (!w.ok()) return std::unexpected(NetError::PayloadTooLarge);
    return w.size();
}

// -------------------------------------------------------------- discovery ---

namespace {
constexpr std::uint8_t kMagic[4] = {'A', 'P', 'X', 'R'};
}

std::expected<std::size_t, NetError> EncodeAnnounce(const AnnouncePacket& p, std::span<std::byte> out) {
    Writer w(out);
    w.bytes(kMagic, 4);
    w.u8(kDiscoveryVersion);
    w.u8(kDiscoAnnounce);
    w.u8(p.protocolVersion);
    w.u32(p.buildSig);
    w.u32(p.mapHash);
    w.u16(p.roomCode);
    w.u16(p.gamePort);
    w.u8(p.players);
    w.u8(p.maxPlayers);
    w.u8(p.phase);
    w.fixedStr(p.mapName, kMapNameBytes);
    w.fixedStr(p.hostName, kNameBytes);
    w.u64(p.playerId);
    if (!w.ok()) return std::unexpected(NetError::PayloadTooLarge);
    return w.size();
}

std::expected<AnnouncePacket, NetError> DecodeAnnounce(std::span<const std::byte> in) {
    Reader r(in);
    for (std::uint8_t m : kMagic) {
        if (r.u8() != m) return std::unexpected(NetError::Malformed);
    }
    if (r.u8() != kDiscoveryVersion) return std::unexpected(NetError::Malformed);
    if (r.u8() != kDiscoAnnounce) return std::unexpected(NetError::Malformed);
    AnnouncePacket p;
    p.protocolVersion = r.u8();
    p.buildSig = r.u32();
    p.mapHash = r.u32();
    p.roomCode = r.u16();
    p.gamePort = r.u16();
    p.players = r.u8();
    p.maxPlayers = r.u8();
    p.phase = r.u8();
    p.mapName = r.fixedStr(kMapNameBytes);
    p.hostName = r.fixedStr(kNameBytes);
    p.playerId = r.u64();
    if (!r.done()) return std::unexpected(NetError::Malformed);
    return p;
}

std::expected<std::size_t, NetError> EncodeQuery(const QueryPacket& p, std::span<std::byte> out) {
    Writer w(out);
    w.bytes(kMagic, 4);
    w.u8(kDiscoveryVersion);
    w.u8(kDiscoQuery);
    w.u16(p.roomCode);
    if (!w.ok()) return std::unexpected(NetError::PayloadTooLarge);
    return w.size();
}

std::expected<QueryPacket, NetError> DecodeQuery(std::span<const std::byte> in) {
    Reader r(in);
    for (std::uint8_t m : kMagic) {
        if (r.u8() != m) return std::unexpected(NetError::Malformed);
    }
    if (r.u8() != kDiscoveryVersion) return std::unexpected(NetError::Malformed);
    if (r.u8() != kDiscoQuery) return std::unexpected(NetError::Malformed);
    QueryPacket p;
    p.roomCode = r.u16();
    if (!r.done()) return std::unexpected(NetError::Malformed);
    return p;
}

std::expected<MsgType, NetError> PeekMessageType(std::span<const std::byte> in) {
    if (in.size() < 2) return std::unexpected(NetError::Malformed);
    if (static_cast<std::uint8_t>(in[0]) != kProtocolVersion) {
        return std::unexpected(NetError::UnsupportedVersion);
    }
    return static_cast<MsgType>(static_cast<std::uint8_t>(in[1]));
}

}  // namespace apex::net
