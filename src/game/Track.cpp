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
}

void Track::Build(const std::vector<ControlPoint>& controlPoints, IPhysicsWorld& world,
                  IRenderer& renderer) {
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
    // Road ribbon: quad strip along the centerline.
    MeshData road;
    const size_t n = samples_.size();
    if (n < 2) return;

    road.positions.reserve(n * 2);
    road.normals.reserve(n * 2);
    road.indices.reserve(n * 6);

    for (size_t i = 0; i < n; ++i) {
        const Sample& s = samples_[i];
        const float halfWidth = s.width * 0.5f;
        const Vec3 left = s.position - s.right * halfWidth;
        const Vec3 right = s.position + s.right * halfWidth;
        road.positions.push_back(left);
        road.positions.push_back(right);
        road.normals.push_back({0.f, 1.f, 0.f});
        road.normals.push_back({0.f, 1.f, 0.f});
    }
    for (size_t i = 0; i < n; ++i) {
        const size_t a = i * 2;
        const size_t b = a + 1;
        const size_t c = ((i + 1) % n) * 2;
        const size_t d = c + 1;
        road.indices.push_back(static_cast<uint32_t>(a));
        road.indices.push_back(static_cast<uint32_t>(c));
        road.indices.push_back(static_cast<uint32_t>(b));
        road.indices.push_back(static_cast<uint32_t>(b));
        road.indices.push_back(static_cast<uint32_t>(c));
        road.indices.push_back(static_cast<uint32_t>(d));
    }
    roadMesh_ = renderer.CreateMesh(road);

    // Kerb stripes every few samples on both sides (visual only).
    MeshData kerb;
    for (size_t i = 0; i + 1 < n; i += 2) {
        const Sample& s = samples_[i];
        const float halfWidth = s.width * 0.5f;
        for (int side = -1; side <= 1; side += 2) {
            const Vec3 inner = s.position + s.right * (halfWidth - 0.35f) * static_cast<float>(side);
            const Vec3 outer = s.position + s.right * (halfWidth + 0.35f) * static_cast<float>(side);
            const uint32_t base = static_cast<uint32_t>(kerb.positions.size());
            kerb.positions.push_back(inner);
            kerb.positions.push_back(outer);
            kerb.positions.push_back(inner + s.forward * 3.5f);
            kerb.positions.push_back(outer + s.forward * 3.5f);
            for (int k = 0; k < 4; ++k) kerb.normals.push_back({0.f, 1.f, 0.f});
            kerb.indices.push_back(base);
            kerb.indices.push_back(base + 2);
            kerb.indices.push_back(base + 1);
            kerb.indices.push_back(base + 1);
            kerb.indices.push_back(base + 2);
            kerb.indices.push_back(base + 3);
        }
    }
    if (!kerb.positions.empty()) kerbMesh_ = renderer.CreateMesh(kerb);
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
    if (roadMesh_ != kInvalidMesh) {
        renderer.DrawMesh(roadMesh_, Transform{}, Color{0.32f, 0.33f, 0.36f, 1.f});
    }
    if (kerbMesh_ != kInvalidMesh) {
        renderer.DrawMesh(kerbMesh_, Transform{}, Color{0.82f, 0.28f, 0.22f, 1.f});
    }

    // Start/finish line marker.
    if (!samples_.empty()) {
        const Sample& s = samples_.front();
        const float halfWidth = s.width * 0.5f;
        for (int i = -4; i < 4; ++i) {
            const float off = static_cast<float>(i) * 0.9f;
            const Vec3 a = s.position + s.right * off - s.forward * 0.25f;
            const Vec3 b = s.position + s.right * off + s.forward * 0.25f;
            renderer.DrawLine3D(a + Vec3{0.f, 0.02f, 0.f}, b + Vec3{0.f, 0.02f, 0.f},
                                (i % 2 == 0) ? Color::White() : Color::Black());
        }
        (void)halfWidth;
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
