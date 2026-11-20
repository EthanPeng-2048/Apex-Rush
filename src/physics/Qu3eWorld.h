#pragma once

#include <memory>
#include <vector>

#include "physics/IPhysicsWorld.h"

namespace apex {

// qu3e adapter implementing IPhysicsWorld. Chassis + track boundaries are
// boxes; suspension uses Raycast. Contact points are captured for damage
// part mapping. Swappable for a Jolt adapter later.
class Qu3eWorld final : public IPhysicsWorld {
public:
    Qu3eWorld();
    ~Qu3eWorld() override;

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
                   BodyHandle ignore) const override;

    std::vector<ContactEvent> DrainContacts() override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace apex
