#pragma once

#include "core/Math.h"

namespace apex {

// Combined-slip tire with load sensitivity and wear. One instance per wheel.
// Sliding dissipation accumulates wear which lowers peak grip and rounds the
// curve (m(W) below).
class TireModel {
public:
    struct Params {
        // Magic Formula shape for F(sigma): D sin(C atan(B s - E(B s - atan(B s))))
        float B = 11.f;        // stiffness factor
        float C = 1.9f;        // shape factor
        float D0 = 1.55f;      // friction coefficient at reference load
        float E = 0.97f;       // curvature factor
        float Fz0 = 1840.f;    // reference vertical load = static corner load (750 kg / 4) [N]
        float loadExp = 0.85f; // n in F(D) = D0 * Fz0 * (Fz/Fz0)^n; n<1 => mu falls with load
        float wearRate = 2.5e-6f;
        float wearGripLoss = 0.35f;   // grip multiplier drop at W=1
        float minGrip = 0.55f;        // floor multiplier
    };

    struct Output {
        Vec3 force;         // world-space tire force
        float slipRatio = 0.f;
        float slipAngleDeg = 0.f;
        float slipMagnitude = 0.f;
        float wearDelta = 0.f;
    };

    explicit TireModel(const Params& params = {});

    // contactVelocity: velocity of the contact patch (world).
    // forward/right: unit wheel frame axes in world (forward = rolling dir).
    Output Compute(float verticalLoad, const Vec3& contactVelocity, const Vec3& forward,
                   const Vec3& right, float driveForce, float brakeForce, float dt);

    float Wear() const { return wear_; }
    void SetWear(float wear) { wear_ = Clampf(wear, 0.f, 1.f); }
    float GripMultiplier() const;

private:
    Params params_;
    float wear_ = 0.f;
};

}  // namespace apex
