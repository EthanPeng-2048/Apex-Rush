#include "vehicle/TireModel.h"

#include <algorithm>
#include <cmath>

namespace apex {

namespace {

// Normalized Magic Formula shape in [-1, 1]: peak at s ~ tan(pi/(2C))/By.
float Shape(float stiffness, float shapeC, float e, float s) {
    const float Bs = stiffness * s;
    return std::sin(shapeC * std::atan(Bs - e * (Bs - std::atan(Bs))));
}

}  // namespace

TireModel::TireModel(const Params& params) : params_(params) {}

float TireModel::GripMultiplier() const {
    return std::max(params_.minGrip, 1.f - params_.wearGripLoss * wear_);
}

void TireModel::Reset() {
    wear_ = 0.f;
    omega_ = 0.f;
    alpha_ = 0.f;
}

TireModel::Output TireModel::Compute(float verticalLoad, const Vec3& contactVelocity,
                                     const Vec3& forward, const Vec3& right, float driveForce,
                                     float brakeForce, float dt, float camberLean) {
    Output out;
    const float r = params_.wheelRadius;
    const float Iw = params_.wheelInertia;

    // Wheel-frame velocity at the contact patch.
    const float vLong = Dot(contactVelocity, forward);
    const float vLat = Dot(contactVelocity, right);

    // Airborne: spin the wheel from drive/brake torque alone (no ground force).
    if (verticalLoad <= 1.f) {
        const float spinSign = std::fabs(omega_) > 0.5f ? Signf(omega_) : Signf(vLong + 1e-3f);
        omega_ += dt * (driveForce * r - brakeForce * r * spinSign) / Iw;
        omega_ = Clampf(omega_, -260.f, 260.f);
        return out;
    }

    const float vRef = std::max(std::fabs(vLong), 2.f);

    // --- Lateral: transient slip angle (relaxation length) ---------------
    const float alphaTarget = std::atan2(vLat, std::max(std::fabs(vLong), 2.f));
    const float relaxRate = std::min(1.f, dt * std::max(std::fabs(vLong), 4.f) / params_.relaxLength);
    alpha_ += (alphaTarget - alpha_) * relaxRate;
    const float tanA = std::tan(alpha_);

    // --- Peak force with load sensitivity --------------------------------
    const float loadRatio = verticalLoad / params_.Fz0;
    const float peak = params_.D0 * params_.Fz0 *
                       std::pow(std::max(loadRatio, 1e-3f), params_.loadExp) * GripMultiplier();

    const float shapeY = Shape(params_.By, params_.C, params_.E, tanA);

    // --- Longitudinal: implicit wheel-spin dynamics ----------------------
    //   Iw*domega = Tdrive - Tbrake - Fx*r,  Fx ~ kStiff*peak*kappa.
    // Backward Euler yields an unconditionally stable contact damper:
    //   Fx = dv / (vRef/(cap*kStiff) + dt*r^2/Iw)
    const float spinSign = std::fabs(omega_) > 0.5f ? Signf(omega_) : Signf(vLong + 1e-3f);
    const float omegaFree = omega_ + dt * (driveForce * r - brakeForce * r * spinSign) / Iw;
    const float dv = omegaFree * r - vLong;
    const float kappaFree = dv / vRef;

    // Friction ellipse: lateral usage eats longitudinal capacity; deep slip
    // rounds the peak off (a sliding tire loses ~20% of peak).
    const float latUse = std::fabs(shapeY);
    float longCap = peak * std::sqrt(std::max(0.12f, 1.f - 0.85f * latUse * latUse));
    longCap *= 1.f - 0.23f * Clampf((std::fabs(kappaFree) - 0.18f) / 0.67f, 0.f, 1.f);

    float fx = dv / (vRef / (longCap * params_.kStiff) + dt * r * r / Iw);
    fx = Clampf(fx, -longCap, longCap);

    omega_ = omegaFree - dt * fx * r / Iw;
    omega_ = Clampf(omega_, -260.f, 260.f);
    // Service brakes hold the wheel against the rolling direction: no spin
    // reversal while braking forward (or backward) motion.
    if (brakeForce > 1.f && vLong > 0.5f) omega_ = std::max(omega_, 0.f);
    if (brakeForce > 1.f && vLong < -0.5f) omega_ = std::min(omega_, 0.f);

    const float kappa = (omega_ * r - vLong) / vRef;

    // --- Lateral force ---------------------------------------------------
    // Longitudinal slip also bleeds lateral grip (combined slip). Force
    // along `right` (+X chassis = geometric left): it opposes vLat and the
    // camber thrust acts toward the wheel-top lean (geometric right is -).
    const float slipBleed = 1.f - 0.55f * Clampf((std::fabs(kappa) - 0.15f) / 0.7f, 0.f, 1.f);
    float fyMax = peak * std::sqrt(std::max(0.11f, 1.f - 0.92f * (fx / peak) * (fx / peak)));
    fyMax *= slipBleed;
    float fyLat = -fyMax * shapeY - params_.camberCoef * verticalLoad * camberLean;
    fyLat = Clampf(fyLat, -1.15f * peak, 1.15f * peak);

    out.force = forward * fx + right * fyLat;

    // Wear accrues from sliding dissipation.
    const float sliding = std::fabs(vLong * kappa) + std::fabs(vLat * tanA);
    out.slipRatio = kappa;
    out.slipAngleDeg = alpha_ * kRadToDeg;
    out.slipMagnitude = std::sqrt(kappa * kappa + tanA * tanA);
    out.wearDelta = params_.wearRate * sliding * dt * (out.slipMagnitude > 0.35f ? 3.f : 1.f);
    wear_ = Clampf(wear_ + out.wearDelta, 0.f, 1.f);

    return out;
}

}  // namespace apex
