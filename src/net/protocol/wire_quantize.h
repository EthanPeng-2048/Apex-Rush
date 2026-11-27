#pragma once

// Fixed-point quantization helpers shared by the wire codec and the game side.
// Kept free of engine headers so the protocol layer stays engine-agnostic.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>

namespace apex::net::q {

constexpr float kPi = std::numbers::pi_v<float>;
constexpr float kTwoPi = 2.f * kPi;
constexpr float kRadToDeg = 180.f / kPi;
constexpr float kDegToRad = kPi / 180.f;

template <typename T>
inline T Clamp(T v, T lo, T hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Float -> signed quantized with a scale factor, clamped to the type range.
inline std::int8_t ToI8(float v, float scale) {
    const float q = Clamp(v * scale, -127.f, 127.f);
    return static_cast<std::int8_t>(std::lround(q));
}
inline std::int16_t ToI16(float v, float scale) {
    const float q = Clamp(v * scale, -32767.f, 32767.f);
    return static_cast<std::int16_t>(std::lround(q));
}
inline std::uint8_t ToU8(float v, float scale) {
    const float q = Clamp(v * scale, 0.f, 255.f);
    return static_cast<std::uint8_t>(std::lround(q));
}
inline std::uint16_t ToU16(float v, float scale) {
    const float q = Clamp(v * scale, 0.f, 65535.f);
    return static_cast<std::uint16_t>(std::lround(q));
}

inline float From(std::int8_t q, float scale) { return q / scale; }
inline float From(std::int16_t q, float scale) { return q / scale; }
inline float From(std::uint8_t q, float scale) { return q / scale; }
inline float From(std::uint16_t q, float scale) { return q / scale; }

// NaN/Inf sanitisation: quantizing garbage must not produce garbage.
inline float Safe(float v) { return std::isfinite(v) ? v : 0.f; }

// --- yaw / pitch / roll <-> quaternion ---------------------------------------
// Composition order R = Ry(yaw) * Rx(pitch) * Rz(roll). Cars never approach
// the gimbal singularity (pitch ±90deg), so this decomposition is stable.
void QuatToYpr(float qx, float qy, float qz, float qw, float& yaw, float& pitch, float& roll);
void YprToQuat(float yaw, float pitch, float roll, float& qx, float& qy, float& qz, float& qw);

// Normalise yaw into [0, 2pi).
inline float WrapYaw(float yaw) {
    float y = std::fmod(yaw, kTwoPi);
    if (y < 0.f) y += kTwoPi;
    return y;
}

// Shortest signed difference b - a on a circle of `period`.
inline float DeltaAngle(float a, float b, float period) {
    float d = std::fmod(b - a, period);
    if (d > period * 0.5f) d -= period;
    if (d < -period * 0.5f) d += period;
    return d;
}

}  // namespace apex::net::q
