#include "vehicle/TireModel.h"

#include <algorithm>
#include <cmath>

namespace apex {

TireModel::TireModel(const Params& params) : params_(params) {}

float TireModel::GripMultiplier() const {
    return std::max(params_.minGrip, 1.f - params_.wearGripLoss * wear_);
}

TireModel::Output TireModel::Compute(float verticalLoad, const Vec3& contactVelocity,
                                     const Vec3& forward, const Vec3& right, float driveForce,
                                     float brakeForce, float dt) {
    Output out;
    if (verticalLoad <= 1.f) return out;

    // Wheel-frame velocity components at the contact patch.
    const float vLong = Dot(contactVelocity, forward);
    const float vLat = Dot(contactVelocity, right);

    // Slip ratio kappa = (drive-induced speed error) / reference speed.
    // Without explicit wheel angular dynamics we approximate the rolling
    // speed from body velocity and treat drive/brake forces as the slip source.
    const float vRef = std::max(std::fabs(vLong), 1.5f);
    float kappa = 0.f;
    if (driveForce != 0.f) kappa += driveForce / (verticalLoad * 8.f);
    if (brakeForce != 0.f) kappa -= brakeForce / (verticalLoad * 8.f) * Signf(vLong + 1e-3f);
    kappa = Clampf(kappa, -1.5f, 1.5f);

    // Slip angle from lateral velocity.
    const float slipAngle = std::atan2(vLat, vRef);
    const float tanAlpha = std::tan(slipAngle);

    // Combined slip magnitude (friction-ellipse style).
    const float sigma = std::sqrt(kappa * kappa + tanAlpha * tanAlpha);
    out.slipRatio = kappa;
    out.slipAngleDeg = slipAngle * kRadToDeg;
    out.slipMagnitude = sigma;

    // Load-sensitive peak: F = D0 * Fz0 * (Fz/Fz0)^n with n < 1, so the
    // friction coefficient mu = D0 * (Fz0/Fz)^(1-n) falls as load grows
    // (load sensitivity, like a real tire). D0 is mu at Fz0.
    const float loadRatio = verticalLoad / params_.Fz0;
    const float peak = params_.D0 * params_.Fz0 *
                       std::pow(std::max(loadRatio, 1e-3f), params_.loadExp) * GripMultiplier();

    // Magic Formula along the combined slip.
    const float Bs = params_.B * sigma;
    const float F = peak * std::sin(params_.C * std::atan(Bs - params_.E * (Bs - std::atan(Bs))));

    // Force direction in the wheel frame. kappa is wheel slip (omega*r - v):
    // positive kappa (driven wheel) must push the car forward. Lateral slip is
    // the contact patch velocity, so lateral force opposes it.
    Vec3 dir{0.f, 0.f, 0.f};
    if (sigma > 1e-5f) {
        const float x = kappa / sigma;        // longitudinal: +kappa => +forward
        const float y = -tanAlpha / sigma;    // lateral (opposes sliding)
        dir = forward * x + right * y;
    }
    out.force = dir * F;

    // Wear accrues from sliding dissipation.
    const float sliding = std::fabs(vLong * kappa) + std::fabs(vLat * tanAlpha);
    out.wearDelta = params_.wearRate * sliding * dt * (sigma > 0.35f ? 3.f : 1.f);
    wear_ = Clampf(wear_ + out.wearDelta, 0.f, 1.f);

    return out;
}

}  // namespace apex
