#pragma once

#include <memory>

#include "physics/IPhysicsWorld.h"

namespace apex {

// nudge implementation of IPhysicsWorld. Boxes + raycast + contact summaries
// cover the vehicle/track needs; surface tags ride nudge's material palette
// (Material.user_data), so RayHit / ContactEvent report them like qu3e's
// per-body userData. Force/torque accumulate into velocity deltas because
// nudge exposes no force API; rotation sets recreate the body (no setter).
class NudgeWorld final : public IPhysicsWorld {
public:
    NudgeWorld();
    ~NudgeWorld() override;

    void SetGravity(const Vec3& gravity) override;
    void Step(float dt) override;

    BodyHandle CreateBody(const BodyDesc& desc) override;
    void DestroyBody(BodyHandle body) override;

    Transform GetTransform(BodyHandle body) const override;
    void SetTransform(BodyHandle body, const Transform& transform) override;

    Vec3 GetLinearVelocity(BodyHandle body) const override;
    Vec3 GetAngularVelocity(BodyHandle body) const override;
    void SetLinearVelocity(BodyHandle body, const Vec3& velocity) override;
    void SetAngularVelocity(BodyHandle body, const Vec3& velocity) override;
    Vec3 GetVelocityAtPoint(BodyHandle body, const Vec3& worldPoint) const override;

    void ApplyForce(BodyHandle body, const Vec3& force, const Vec3& worldPoint) override;
    void ApplyImpulse(BodyHandle body, const Vec3& impulse, const Vec3& worldPoint) override;
    void ApplyTorque(BodyHandle body, const Vec3& torque) override;

    RayHit Raycast(const Vec3& origin, const Vec3& direction, float maxDistance,
                   BodyHandle ignore = kInvalidBody) const override;

    std::vector<ContactEvent> DrainContacts() override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace apex
