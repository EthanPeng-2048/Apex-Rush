#pragma once

#include <vector>

#include "core/Math.h"
#include "game/MapData.h"
#include "physics/IPhysicsWorld.h"
#include "render/IRenderer.h"

namespace apex {

// A closed circuit built from authored map data (MapData). Produces:
//  - layered render meshes: asphalt + edge lines + kerbs, per-side grass and
//    gravel runoff bands, barrier walls at the outer band edge, terrain base
//    grid (height posts + track-corridor blend) and tree decor
//  - layered static collision bodies with SurfaceType userData (road / grass
//    / gravel / wall) so the vehicle picks grip per surface
//  - start/finish line and DRS zone ranges expressed as spline arc length
class Track {
public:
    // Build from map data. Safe to call again on an existing track (settings
    // change): previous meshes are destroyed and body lists cleared first.
    void Build(const MapData& map, IPhysicsWorld& world, IRenderer& renderer);

    // Release render meshes (called by Build before rebuilding).
    void Dispose(IRenderer& renderer);

    // Spline helpers (arc-length parameterized).
    Vec3 PositionAt(float distance) const;
    Vec3 ForwardAt(float distance) const;
    float TotalLength() const { return totalLength_; }
    float WidthAt(float distance) const;
    // Local curvature (1/radius, horizontal) — AI corner-speed planning.
    float CurvatureAt(float distance) const;

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
        float width = 12.f;
        float grassL = 0.f, gravelL = 0.f;
        float grassR = 0.f, gravelR = 0.f;
        float distance = 0.f;
        float curvature = 0.f;  // 1/radius, horizontal
    };

    void Resample(const MapData& map, float spacing);
    void BuildPhysics(const MapData& map, IPhysicsWorld& world);
    void BuildMesh(const MapData& map, IRenderer& renderer);

    std::vector<Sample> samples_;
    std::vector<Zone> drsZones_;
    std::vector<Zone> overtakeZones_;
    std::vector<BodyHandle> wallBodies_;
    std::vector<BodyHandle> groundBodies_;
    float totalLength_ = 0.f;
    float spacing_ = 4.f;
    MeshHandle roadMesh_ = kInvalidMesh;
    MeshHandle edgeLineMesh_ = kInvalidMesh;
    MeshHandle kerbRedMesh_ = kInvalidMesh;
    MeshHandle kerbWhiteMesh_ = kInvalidMesh;
    MeshHandle wallMesh_ = kInvalidMesh;
    MeshHandle grassBandMesh_ = kInvalidMesh;
    MeshHandle gravelBandMesh_ = kInvalidMesh;
    MeshHandle terrainMesh_ = kInvalidMesh;
    MeshHandle treeMesh_ = kInvalidMesh;
    MeshHandle treeMeshLarge_ = kInvalidMesh;
    MeshHandle startLineWhiteMesh_ = kInvalidMesh;
    MeshHandle startLineBlackMesh_ = kInvalidMesh;

    // Tree instances split by size bucket (Transform has no scale).
    struct TreeInstance {
        Vec3 position;
        float scale;
    };
    std::vector<TreeInstance> trees_;

    // Build-time helper: nearest centerline sample by horizontal distance.
    const Sample* NearestSample(const Vec3& pos, float& dist) const;
};

}  // namespace apex
