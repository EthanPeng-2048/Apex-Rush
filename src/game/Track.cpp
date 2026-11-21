#include "game/Track.h"

#include <algorithm>
#include <cmath>

namespace apex {

namespace {

// Uniform Catmull-Rom through 4 control points at parameter t in [0,1].
Vec3 CatmullRom(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, float t) {
    const float t2 = t * t;
    const float t3 = t2 * t;
    return (p1 * 2.f + (p2 - p0) * t + (p0 * 2.f - p1 * 5.f + p2 * 4.f - p3) * t2 +
            (p0 * -1.f + p1 * 3.f - p2 * 3.f + p3) * t3) *
           0.5f;
}

float WrapIndex(int i, int n) {
    i %= n;
    return static_cast<float>(i < 0 ? i + n : i);
}

}  // namespace

void Track::Resample(const std::vector<ControlPoint>& controlPoints, float spacing) {
    samples_.clear();
    const int n = static_cast<int>(controlPoints.size());
    if (n < 3) return;

    // Dense interpolation, then resample by arc length.
    std::vector<Vec3> dense;
    std::vector<float> widths;
    const int perSegment = 24;
    for (int i = 0; i < n; ++i) {
        const Vec3& p0 = controlPoints[static_cast<int>(WrapIndex(i - 1, n))].position;
        const Vec3& p1 = controlPoints[i].position;
        const Vec3& p2 = controlPoints[static_cast<int>(WrapIndex(i + 1, n))].position;
        const Vec3& p3 = controlPoints[static_cast<int>(WrapIndex(i + 2, n))].position;
        const float w0 = controlPoints[static_cast<int>(WrapIndex(i - 1, n))].width;
        const float w1 = controlPoints[i].width;
        const float w2 = controlPoints[static_cast<int>(WrapIndex(i + 1, n))].width;
        const float w3 = controlPoints[static_cast<int>(WrapIndex(i + 2, n))].width;
        for (int s = 0; s < perSegment; ++s) {
            const float t = static_cast<float>(s) / perSegment;
            dense.push_back(CatmullRom(p0, p1, p2, p3, t));
            // Linear width blend is enough for the road ribbon.
            widths.push_back(Lerp(w1, w2, t));
        }
    }
    dense.push_back(dense.front());
    widths.push_back(widths.front());

    // Arc-length resample. `sampledDist` is the absolute distance of the last
    // placed sample and must persist across segments — resetting it per
    // segment makes the inner loop re-walk the whole prefix each time.
    float accum = 0.f;
    float sampledDist = 0.f;
    Sample prev;
    prev.position = dense.front();
    prev.width = widths.front();
    prev.distance = 0.f;
    prev.forward = Normalize(dense[1] - dense[0]);
    prev.right = Normalize(Cross(Vec3{0.f, 1.f, 0.f}, prev.forward));
    samples_.push_back(prev);

    spacing_ = spacing;
    for (size_t i = 1; i < dense.size(); ++i) {
        const Vec3 seg = dense[i] - dense[i - 1];
        const float segLen = Length(seg);
        if (segLen < 1e-4f) continue;
        const float segStart = accum;
        accum += segLen;

        while (sampledDist + spacing <= accum) {
            const float t = (sampledDist + spacing - segStart) / segLen;
            Sample s;
            s.position = dense[i - 1] + seg * t;
            s.width = Lerp(widths[i - 1], widths[i], t);
            s.distance = sampledDist + spacing;
            s.forward = Normalize(seg);
            s.right = Normalize(Cross(Vec3{0.f, 1.f, 0.f}, s.forward));
            samples_.push_back(s);
            prev = s;
            sampledDist += spacing;
        }
        prev.forward = Normalize(seg);
    }

    // Close the loop: last segment back to start.
    totalLength_ = samples_.empty() ? 0.f : accum;
    if (!samples_.empty()) {
        samples_.front().forward = Normalize(samples_[1].position - samples_.back().position);
        samples_.front().right =
            Normalize(Cross(Vec3{0.f, 1.f, 0.f}, samples_.front().forward));
    }

    // Per-sample horizontal curvature (1/radius) from consecutive headings,
    // smoothed over a few samples. Pitch changes must not count as curvature.
    const size_t sampleCount = samples_.size();
    if (sampleCount > 2) {
        std::vector<float> raw(sampleCount, 0.f);
        for (size_t i = 0; i < sampleCount; ++i) {
            Vec3 f0 = samples_[(i + sampleCount - 1) % sampleCount].forward;
            Vec3 f1 = samples_[i].forward;
            f0.y = 0.f;
            f1.y = 0.f;
            if (LengthSq(f0) > 1e-8f && LengthSq(f1) > 1e-8f) {
                raw[i] = std::acos(Clampf(Dot(Normalize(f0), Normalize(f1)), -1.f, 1.f)) /
                         std::max(spacing_, 1e-3f);
            }
        }
        for (size_t i = 0; i < sampleCount; ++i) {
            float acc = 0.f;
            for (int k = -2; k <= 2; ++k) {
                acc += raw[(i + static_cast<size_t>(k) + sampleCount) % sampleCount];
            }
            samples_[i].curvature = acc / 5.f;
        }
    }
}

void Track::Dispose(IRenderer& renderer) {
    const MeshHandle meshes[] = {grassMesh_,  roadMesh_,        edgeLineMesh_, kerbRedMesh_,
                                 kerbWhiteMesh_, wallMesh_,     startLineWhiteMesh_,
                                 startLineBlackMesh_};
    for (MeshHandle mesh : meshes) {
        if (mesh != kInvalidMesh) renderer.DestroyMesh(mesh);
    }
    grassMesh_ = roadMesh_ = edgeLineMesh_ = kerbRedMesh_ = kerbWhiteMesh_ =
        wallMesh_ = startLineWhiteMesh_ = startLineBlackMesh_ = kInvalidMesh;
    wallBodies_.clear();
    drsZones_.clear();
    overtakeZones_.clear();
}

void Track::Build(const std::vector<ControlPoint>& controlPoints, IPhysicsWorld& world,
                  IRenderer& renderer) {
    Dispose(renderer);
    Resample(controlPoints, 4.f);
    BuildPhysics(world);
    BuildMesh(renderer);

    // Default zones: one DRS zone on the main straight, one overtake zone.
    if (totalLength_ > 200.f) {
        drsZones_.push_back({totalLength_ * 0.02f, totalLength_ * 0.16f});
        overtakeZones_.push_back({totalLength_ * 0.30f, totalLength_ * 0.42f});
    }
}

Vec3 Track::PositionAt(float distance) const {
    if (samples_.empty()) return {};
    const float d = std::fmod(distance, totalLength_);
    const float idx = d / spacing_;
    const size_t i0 = static_cast<size_t>(idx) % samples_.size();
    const size_t i1 = (i0 + 1) % samples_.size();
    const float t = idx - std::floor(idx);
    return Vec3{Lerp(samples_[i0].position.x, samples_[i1].position.x, t),
                Lerp(samples_[i0].position.y, samples_[i1].position.y, t),
                Lerp(samples_[i0].position.z, samples_[i1].position.z, t)};
}

Vec3 Track::ForwardAt(float distance) const {
    if (samples_.empty()) return {0.f, 0.f, 1.f};
    const float d = std::fmod(distance, totalLength_);
    const size_t i = static_cast<size_t>(d / spacing_) % samples_.size();
    return samples_[i].forward;
}

float Track::WidthAt(float distance) const {
    if (samples_.empty()) return 12.f;
    const float d = std::fmod(distance, totalLength_);
    const size_t i = static_cast<size_t>(d / spacing_) % samples_.size();
    return samples_[i].width;
}

float Track::CurvatureAt(float distance) const {
    if (samples_.empty()) return 0.f;
    const float d = std::fmod(distance, totalLength_);
    const size_t i = static_cast<size_t>(d / spacing_) % samples_.size();
    return samples_[i].curvature;
}

float Track::ProjectDistance(const Vec3& worldPos) const {
    if (samples_.empty()) return 0.f;
    float bestDist = 0.f;
    float bestErr = 1e30f;
    for (size_t i = 0; i < samples_.size(); ++i) {
        const float err = LengthSq(samples_[i].position - worldPos);
        if (err < bestErr) {
            bestErr = err;
            bestDist = samples_[i].distance;
        }
    }
    return bestDist;
}

void Track::BuildPhysics(IPhysicsWorld& world) {
    // Grass ground plane just under the road surface: cars that leave the
    // track keep wheel contact (the vehicle applies reduced grip below y=0)
    // instead of beaching on their chassis.
    BodyDesc ground;
    ground.transform.position = {0.f, -2.05f, 0.f};
    ground.halfExtents = {600.f, 2.f, 600.f};
    ground.mass = 0.f;
    ground.friction = 0.7f;
    world.CreateBody(ground);

    // Road slab per sample: thin wide boxes so raycast suspension always
    // finds the surface. Walls as taller boxes on both edges.
    for (const Sample& s : samples_) {
        const float halfWidth = s.width * 0.5f;

        BodyDesc road;
        road.transform.position = s.position - Vec3{0.f, 0.25f, 0.f};
        const float yaw = std::atan2(s.forward.x, s.forward.z);
        road.transform.rotation = Quat::FromAxisAngle(Vec3{0.f, 1.f, 0.f}, yaw);
        road.halfExtents = {halfWidth, 0.25f, 2.2f};
        road.mass = 0.f;
        road.friction = 0.9f;
        road.restitution = 0.f;
        world.CreateBody(road);

        for (int side = -1; side <= 1; side += 2) {
            BodyDesc wall;
            const Vec3 wallPos = s.position + s.right * (halfWidth + 0.6f) * static_cast<float>(side);
            wall.transform.position = wallPos + Vec3{0.f, 0.5f, 0.f};
            wall.transform.rotation = road.transform.rotation;
            wall.halfExtents = {0.6f, 0.7f, 2.2f};
            wall.mass = 0.f;
            wall.friction = 0.55f;
            wall.restitution = 0.12f;
            wallBodies_.push_back(world.CreateBody(wall));
        }
    }
}

void Track::BuildMesh(IRenderer& renderer) {
    const size_t n = samples_.size();
    if (n < 2) return;

    MeshData road, edgeLines, kerbRed, kerbWhite, walls, grass;

    // Quad helper: (a,b) and (c,d) are opposite edges; winding is fixed so the
    // face normal points along `normal`.
    auto addQuad = [](MeshData& mesh, const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d,
                      const Vec3& normal) {
        const uint32_t base = static_cast<uint32_t>(mesh.positions.size());
        mesh.positions.push_back(a);
        mesh.positions.push_back(b);
        mesh.positions.push_back(c);
        mesh.positions.push_back(d);
        for (int k = 0; k < 4; ++k) mesh.normals.push_back(normal);
        const bool flip = Dot(Cross(b - a, c - a), normal) < 0.f;
        if (!flip) {
            mesh.indices.push_back(base + 0);
            mesh.indices.push_back(base + 1);
            mesh.indices.push_back(base + 2);
            mesh.indices.push_back(base + 1);
            mesh.indices.push_back(base + 3);
            mesh.indices.push_back(base + 2);
        } else {
            mesh.indices.push_back(base + 0);
            mesh.indices.push_back(base + 2);
            mesh.indices.push_back(base + 1);
            mesh.indices.push_back(base + 2);
            mesh.indices.push_back(base + 3);
            mesh.indices.push_back(base + 1);
        }
    };

    const Vec3 up{0.f, 1.f, 0.f};
    const float kerbDepth = 0.8f;
    const float lineDepth = 0.22f;
    const float lift = 0.015f;  // keep coplanar strips from z-fighting

    for (size_t i = 0; i < n; ++i) {
        const Sample& s0 = samples_[i];
        const Sample& s1 = samples_[(i + 1) % n];
        const float w0 = s0.width * 0.5f;
        const float w1 = s1.width * 0.5f;

        // Asphalt ribbon.
        addQuad(road, s0.position - s0.right * w0, s0.position + s0.right * w0,
                s1.position - s1.right * w1, s1.position + s1.right * w1, up);

        for (int side = -1; side <= 1; side += 2) {
            const float sgn = static_cast<float>(side);

            // White edge line just inboard of the kerb.
            const Vec3 lineIn0 = s0.position + s0.right * sgn * (w0 - kerbDepth - lineDepth);
            const Vec3 lineOut0 = s0.position + s0.right * sgn * (w0 - kerbDepth);
            const Vec3 lineIn1 = s1.position + s1.right * sgn * (w1 - kerbDepth - lineDepth);
            const Vec3 lineOut1 = s1.position + s1.right * sgn * (w1 - kerbDepth);
            addQuad(edgeLines, lineIn0 + up * lift, lineOut0 + up * lift, lineIn1 + up * lift,
                    lineOut1 + up * lift, up);

            // Kerb strip at the asphalt edge, alternating red/white per segment.
            const Vec3 kerbIn0 = s0.position + s0.right * sgn * (w0 - kerbDepth);
            const Vec3 kerbOut0 = s0.position + s0.right * sgn * w0;
            const Vec3 kerbIn1 = s1.position + s1.right * sgn * (w1 - kerbDepth);
            const Vec3 kerbOut1 = s1.position + s1.right * sgn * w1;
            MeshData& kerb = (i % 2 == 0) ? kerbRed : kerbWhite;
            addQuad(kerb, kerbIn0 + up * lift, kerbOut0 + up * lift, kerbIn1 + up * lift,
                    kerbOut1 + up * lift, up);

            // Barrier wall on the outer face of the kerb (matches the physics
            // wall boxes that sit just outside the road edge). Drawn double
            // sided so it reads from outside the circuit too.
            const float wallH = 1.2f;
            const Vec3 base0 = s0.position + s0.right * sgn * w0;
            const Vec3 base1 = s1.position + s1.right * sgn * w1;
            addQuad(walls, base0, base1, base0 + up * wallH, base1 + up * wallH, -s0.right * sgn);
            addQuad(walls, base0, base1, base0 + up * wallH, base1 + up * wallH, s0.right * sgn);
        }
    }

    // Grass apron under everything.
    const float field = 420.f;
    addQuad(grass, Vec3{-field, -0.02f, -field}, Vec3{field, -0.02f, -field},
            Vec3{-field, -0.02f, field}, Vec3{field, -0.02f, field}, up);

    // Start/finish checker band across the road, oriented with the centerline.
    {
        MeshData lineWhite, lineBlack;
        const Sample& s = samples_.front();
        const float halfWidth = s.width * 0.5f;
        for (int i = -10; i < 10; ++i) {
            const float off0 = static_cast<float>(i) * 0.55f;
            const float off1 = off0 + 0.55f;
            if (off0 > halfWidth || off1 < -halfWidth) continue;
            const Vec3 p0 = s.position + s.right * off0;
            const Vec3 p1 = s.position + s.right * off1;
            MeshData& cell = (i % 2 == 0) ? lineWhite : lineBlack;
            addQuad(cell, p0 - s.forward * 0.3f + up * lift, p1 - s.forward * 0.3f + up * lift,
                    p0 + s.forward * 0.3f + up * lift, p1 + s.forward * 0.3f + up * lift, up);
        }
        startLineWhiteMesh_ = renderer.CreateMesh(lineWhite);
        startLineBlackMesh_ = renderer.CreateMesh(lineBlack);
    }

    roadMesh_ = renderer.CreateMesh(road);
    edgeLineMesh_ = renderer.CreateMesh(edgeLines);
    kerbRedMesh_ = renderer.CreateMesh(kerbRed);
    kerbWhiteMesh_ = renderer.CreateMesh(kerbWhite);
    wallMesh_ = renderer.CreateMesh(walls);
    grassMesh_ = renderer.CreateMesh(grass);
}

Vec3 Track::StartPosition(int gridSlot) const {
    if (samples_.empty()) return {0.f, 1.f, 0.f};
    // Stagger grid slots back along the centerline, alternating sides.
    const float back = 8.f + static_cast<float>(gridSlot) * 9.f;
    const float d = totalLength_ - back;
    const Vec3 center = PositionAt(d);
    const Vec3 fwd = ForwardAt(d);
    const Vec3 right = Normalize(Cross(Vec3{0.f, 1.f, 0.f}, fwd));
    const float side = (gridSlot % 2 == 0) ? -2.6f : 2.6f;
    return center + right * side + Vec3{0.f, 0.55f, 0.f};
}

Quat Track::StartRotation() const {
    const Vec3 fwd = ForwardAt(totalLength_ - 8.f);
    return Quat::FromAxisAngle(Vec3{0.f, 1.f, 0.f}, std::atan2(fwd.x, fwd.z));
}

void Track::Render(IRenderer& renderer) const {
    const Transform origin{};
    if (grassMesh_ != kInvalidMesh) {
        renderer.DrawMesh(grassMesh_, origin, Color{0.20f, 0.44f, 0.22f, 1.f});
    }
    if (roadMesh_ != kInvalidMesh) {
        renderer.DrawMesh(roadMesh_, origin, Color{0.23f, 0.24f, 0.27f, 1.f});
    }
    if (edgeLineMesh_ != kInvalidMesh) {
        renderer.DrawMesh(edgeLineMesh_, origin, Color{0.88f, 0.88f, 0.9f, 1.f});
    }
    if (kerbRedMesh_ != kInvalidMesh) {
        renderer.DrawMesh(kerbRedMesh_, origin, Color{0.82f, 0.22f, 0.18f, 1.f});
    }
    if (kerbWhiteMesh_ != kInvalidMesh) {
        renderer.DrawMesh(kerbWhiteMesh_, origin, Color{0.92f, 0.92f, 0.92f, 1.f});
    }
    if (wallMesh_ != kInvalidMesh) {
        renderer.DrawMesh(wallMesh_, origin, Color{0.75f, 0.76f, 0.78f, 1.f});
    }
    if (startLineWhiteMesh_ != kInvalidMesh) {
        renderer.DrawMesh(startLineWhiteMesh_, origin, Color::White());
        renderer.DrawMesh(startLineBlackMesh_, origin, Color{0.1f, 0.1f, 0.1f, 1.f});
    }
}

std::vector<Track::ControlPoint> MakeCircuit(int index) {
    std::vector<Track::ControlPoint> points;
    const float radius = (index == 0) ? 120.f : (index == 1 ? 160.f : 130.f);
    const int count = (index == 0) ? 10 : (index == 1 ? 14 : 12);

    for (int i = 0; i < count; ++i) {
        const float angle = (static_cast<float>(i) / count) * 2.f * kPi;
        Track::ControlPoint p;
        const float wobble = (index == 2) ? 42.f : 24.f;
        const float r = radius + std::sin(angle * 3.f) * wobble;
        p.position = {std::cos(angle) * r, 0.f, std::sin(angle) * r};
        p.width = (i % 5 == 0) ? 15.f : 12.f;
        points.push_back(p);
    }
    return points;
}

}  // namespace apex
