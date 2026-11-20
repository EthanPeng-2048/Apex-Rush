#pragma once

#include <vector>

#include "core/Math.h"
#include "physics/IPhysicsWorld.h"
#include "render/IRenderer.h"

namespace apex {

// A closed circuit generated from control points. Produces:
//  - a render mesh (road surface + kerbs)
//  - static collision boxes along both walls
//  - start/finish line and DRS zone ranges expressed as spline arc length
class Track {
public:
    struct ControlPoint {
        Vec3 position;
        float width = 12.f;
    };

    // Build the circuit from control points (closed loop, Catmull-Rom).
    void Build(const std::vector<ControlPoint>& controlPoints, IPhysicsWorld& world,
               IRenderer& renderer);

    // Spline helpers (arc-length parameterized).
    Vec3 PositionAt(float distance) const;
    Vec3 ForwardAt(float distance) const;
    float TotalLength() const { return totalLength_; }
    float WidthAt(float distance) const;

    // Progress helpers: project a world position onto the centerline.
    float ProjectDistance(const Vec3& worldPos) const;

    // Zones: [0] start/finish at distance 0. DRS zones are [start, end].
    struct Zone {
        float startDistance = 0.f;
        float endDistance = 0.f;
        bool Contains(float d) const {
            return d >= startDistance && d <= endDistance;
        }
    };
    const std::vector<Zone>& DrsZones() const { return drsZones_; }
    const std::vector<Zone>& OvertakeZones() const { return overtakeZones_; }

    Vec3 StartPosition(int gridSlot) const;
    Quat StartRotation() const;

    // Debug/visual pass.
    void Render(IRenderer& renderer) const;

private:
    struct Sample {
        Vec3 position;
        Vec3 forward;
        Vec3 right;
        float width;
        float distance;
    };

    void Resample(const std::vector<ControlPoint>& controlPoints, float spacing);
    void BuildPhysics(IPhysicsWorld& world);
    void BuildMesh(IRenderer& renderer);

    std::vector<Sample> samples_;
    std::vector<Zone> drsZones_;
    std::vector<Zone> overtakeZones_;
    std::vector<BodyHandle> wallBodies_;
    float totalLength_ = 0.f;
    float spacing_ = 4.f;
    MeshHandle roadMesh_ = kInvalidMesh;
    MeshHandle kerbMesh_ = kInvalidMesh;
};

// Three built-in circuits (small/medium/techy) generated procedurally.
std::vector<Track::ControlPoint> MakeCircuit(int index);

}  // namespace apex
