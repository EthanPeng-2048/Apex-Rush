#pragma once

#include <cmath>
#include <cstdint>

namespace apex {

// Engine-agnostic math types. Render/physics backends convert to their own
// representations at the boundary so core code never depends on a library.
struct Vec3 {
    float x = 0.f, y = 0.f, z = 0.f;

    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
};

inline Vec3 operator*(float s, const Vec3& v) { return v * s; }

inline float Dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

inline Vec3 Cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline float LengthSq(const Vec3& v) { return Dot(v, v); }
inline float Length(const Vec3& v) { return std::sqrt(LengthSq(v)); }

inline Vec3 Normalize(const Vec3& v) {
    const float len = Length(v);
    return len > 1e-6f ? v / len : Vec3{0.f, 0.f, 0.f};
}

// Quaternion, Hamilton convention, rotates vectors: v' = q * v * q^-1.
struct Quat {
    float x = 0.f, y = 0.f, z = 0.f, w = 1.f;

    Quat() = default;
    Quat(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}

    static Quat Identity() { return {}; }

    static Quat FromAxisAngle(const Vec3& axis, float angleRad) {
        const Vec3 n = Normalize(axis);
        const float h = angleRad * 0.5f;
        const float s = std::sin(h);
        return {n.x * s, n.y * s, n.z * s, std::cos(h)};
    }
};

inline Quat operator*(const Quat& a, const Quat& b) {
    return {
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
}

inline Vec3 Rotate(const Quat& q, const Vec3& v) {
    const Vec3 u{q.x, q.y, q.z};
    const Vec3 t = Cross(u, v) * 2.f;
    return v + t * q.w + Cross(u, t);
}

// Inverse of a unit quaternion is its conjugate.
inline Quat Conjugate(const Quat& q) { return {-q.x, -q.y, -q.z, q.w}; }

inline Vec3 RotateInv(const Quat& q, const Vec3& v) { return Rotate(Conjugate(q), v); }

struct Transform {
    Vec3 position;
    Quat rotation;
};

inline Vec3 TransformPoint(const Transform& t, const Vec3& p) {
    return Rotate(t.rotation, p) + t.position;
}

inline Vec3 TransformDirection(const Transform& t, const Vec3& d) {
    return Rotate(t.rotation, d);
}

constexpr float kPi = 3.14159265358979f;
constexpr float kDegToRad = kPi / 180.f;
constexpr float kRadToDeg = 180.f / kPi;

inline float Clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float Lerp(float a, float b, float t) { return a + (b - a) * t; }
inline float Signf(float v) { return v >= 0.f ? 1.f : -1.f; }

}  // namespace apex
