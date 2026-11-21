#pragma once

import rush.engine.core;

namespace apex {

// Per-wheel tire model:
//  - longitudinal: wheel-spin dynamics (omega) with an implicitly damped
//    contact force, so kappa is a real slip ratio (omega*r - v)/vRef and the
//    wheel can lock / spin. This is the hook TC and ABS modulate.
//  - lateral: load-sensitive Magic Formula on the transient (relaxation)
//    slip angle, reduced by longitudinal usage (friction ellipse) plus a
//    simplified camber-thrust term.
//  - wear accrues from sliding dissipation and lowers peak grip.
class TireModel {
public:
    struct Params {
        // Longitudinal: force = clamp(kStiff * peak * kappa, +/- peak), with
        // the implicit contact damper below. Peak near kappa ~ 1/kStiff.
        float kStiff = 11.f;

        // Lateral Magic Formula: D0*Fz0*(Fz/Fz0)^n *
        //   sin(C*atan(By*s - E*(By*s - atan(By*s)))),  s = tan(alpha)
        float C = 1.35f;
        float By = 11.5f;
        float E = 0.97f;
        float D0 = 1.60f;       // friction coefficient at reference load
        float Fz0 = 1840.f;     // reference vertical load (static corner) [N]
        float loadExp = 0.85f;  // n<1 => mu falls as load grows

        // Wheel spin dynamics: Iw * domega = Tdrive - Tbrake - Fx * r.
        float wheelInertia = 1.3f;   // kg m^2
        float wheelRadius = 0.33f;   // m

        // Lateral transient slip: first-order lag with this length [m].
        float relaxLength = 0.40f;

        // Simplified camber: thrust = camberCoef * Fz * lean (rad).
        float camberCoef = 3.0f;

        float wearRate = 2.5e-6f;
        float wearGripLoss = 0.35f;
        float minGrip = 0.55f;

        // F1-style split calibrated to real racing-slick behaviour: the
        // lateral curve peaks at 6.5-7.5 deg slip (By at C=1.35), the front
        // is the steering tire (slower build-up), the rear is stiffer and
        // more load-sensitive so heavy throttle / load transfer walks the
        // car into gentle oversteer.
        static Params Front() {
            Params p;
            p.By = 18.f;
            p.D0 = 1.58f;
            p.loadExp = 0.87f;
            p.wheelInertia = 1.2f;
            p.relaxLength = 0.35f;
            return p;
        }
        static Params Rear() {
            Params p;
            p.By = 20.f;
            p.D0 = 1.62f;
            p.loadExp = 0.83f;
            p.wheelInertia = 1.5f;
            p.relaxLength = 0.45f;
            return p;
        }
    };

    struct Output {
        Vec3 force;              // world-space tire force
        float slipRatio = 0.f;   // kappa = (omega*r - vLong)/vRef
        float slipAngleDeg = 0.f;
        float slipMagnitude = 0.f;
        float wearDelta = 0.f;
    };

    // Separate default ctor: clang rejects `= {}` as a default argument here
    // (default member initializers of Params are not yet available at that
    // point in the enclosing class).
    TireModel() = default;
    explicit TireModel(const Params& params);

    // contactVelocity: velocity of the contact patch (world).
    // forward/right: unit wheel frame axes in world (right = +X chassis axis,
    // i.e. the geometric LEFT side — see Vehicle.cpp frame notes).
    // driveForce/brakeForce: contact-plane forces [N] for this wheel.
    // camberLean: wheel-top lean toward the geometric right [rad].
    Output Compute(float verticalLoad, const Vec3& contactVelocity, const Vec3& forward,
                   const Vec3& right, float driveForce, float brakeForce, float dt,
                   float camberLean = 0.f);

    float Wear() const { return wear_; }
    void SetWear(float wear) { wear_ = Clampf(wear, 0.f, 1.f); }
    float GripMultiplier() const;
    float Omega() const { return omega_; }
    void Reset();

private:
    Params params_;
    float wear_ = 0.f;
    float omega_ = 0.f;      // wheel spin [rad/s]
    float alpha_ = 0.f;      // relaxed lateral slip angle [rad]
};

}  // namespace apex
