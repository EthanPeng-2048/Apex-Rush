#include "game/Track.h"

#include <algorithm>
#include <vector>
#include <cmath>

namespace apex {

namespace {

// Cubic Hermite with chord-length-bounded tangents (the road-spline
// standard): the tangent at each knot is the neighbor-chord DIRECTION scaled
// to the segment length, so wildly different segment lengths (long straights
// next to short corners — real circuits always have both) can never whip a
// hook into the curve the way finite-difference Catmull-Rom tangents do.
Vec3 HermiteCR(const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& p3, float u) {
    const float chord = std::max(Length(p2 - p1), 1e-4f);
    const Vec3 d0 = p2 - p0;
    const Vec3 d1 = p3 - p1;
    const Vec3 m1 = (LengthSq(d0) > 1e-8f) ? Normalize(d0) * chord : Vec3{};
    const Vec3 m2 = (LengthSq(d1) > 1e-8f) ? Normalize(d1) * chord : Vec3{};
    const float u2 = u * u;
    const float u3 = u2 * u;
    return p1 * (2.f * u3 - 3.f * u2 + 1.f) + m1 * (u3 - 2.f * u2 + u) +
           p2 * (-2.f * u3 + 3.f * u2) + m2 * (u3 - u2);
}

float HermiteCR1D(float p0, float p1, float p2, float p3, float u) {
    const float chord = std::max(std::fabs(p2 - p1), 1e-4f);
    const float d0 = p2 - p0;
    const float d1 = p3 - p1;
    const float m1 = (std::fabs(d0) > 1e-6f) ? (d0 > 0.f ? chord : -chord) : 0.f;
    const float m2 = (std::fabs(d1) > 1e-6f) ? (d1 > 0.f ? chord : -chord) : 0.f;
    const float u2 = u * u;
    const float u3 = u2 * u;
    return p1 * (2.f * u3 - 3.f * u2 + 1.f) + m1 * (u3 - 2.f * u2 + u) +
           p2 * (-2.f * u3 + 3.f * u2) + m2 * (u3 - u2);
}

float WrapIndex(int i, int n) {
    i %= n;
    return static_cast<float>(i < 0 ? i + n : i);
}

// Slab orientation: yaw about +Y, then pitch about the horizontal right axis
// so layered road/band boxes follow the 3D tangent (track undulation).
Quat SlabRotation(const Vec3& forward, const Vec3& right) {
    const float yaw = std::atan2(forward.x, forward.z);
    const float pitch = -std::asin(Clampf(forward.y, -1.f, 1.f));
    return Quat::FromAxisAngle(right, pitch) * Quat::FromAxisAngle(Vec3{0.f, 1.f, 0.f}, yaw);
}

}  // namespace

void Track::Resample(const MapData& map, float spacing) {
    samples_.clear();
    const auto& cps = map.centerline;
    const int n = static_cast<int>(cps.size());
    if (n < 3) return;

    // Dense interpolation (Hermite CR on the centerline and every band
    // column), then resample by arc length.
    struct Dense {
        Vec3 position;
        float width, grassL, gravelL, grassR, gravelR;
    };
    std::vector<Dense> dense;

    const int perSegment = 24;
    for (int i = 0; i < n; ++i) {
        const auto& p0 = cps[static_cast<size_t>(WrapIndex(i - 1, n))];
        const auto& p1 = cps[static_cast<size_t>(i)];
        const auto& p2 = cps[static_cast<size_t>(WrapIndex(i + 1, n))];
        const auto& p3 = cps[static_cast<size_t>(WrapIndex(i + 2, n))];
        for (int s = 0; s < perSegment; ++s) {
            const float u = static_cast<float>(s) / perSegment;
            Dense d;
            d.position = HermiteCR(p0.position, p1.position, p2.position, p3.position, u);
            d.width = HermiteCR1D(p0.width, p1.width, p2.width, p3.width, u);
            d.grassL = HermiteCR1D(p0.grassLeft, p1.grassLeft, p2.grassLeft, p3.grassLeft, u);
            d.gravelL = HermiteCR1D(p0.gravelLeft, p1.gravelLeft, p2.gravelLeft, p3.gravelLeft, u);
            d.grassR = HermiteCR1D(p0.grassRight, p1.grassRight, p2.grassRight, p3.grassRight, u);
            d.gravelR = HermiteCR1D(p0.gravelRight, p1.gravelRight, p2.gravelRight, p3.gravelRight, u);
            dense.push_back(d);
        }
    }
    dense.push_back(dense.front());

    // Arc-length resample. `sampledDist` is the absolute distance of the last
    // placed sample and must persist across segments — resetting it per
    // segment makes the inner loop re-walk the whole prefix each time.
    float accum = 0.f;
    float sampledDist = 0.f;
    auto makeSample = [](const Dense& d, float dist) {
        Sample s;
        s.position = d.position;
        s.width = d.width;
        s.grassL = std::max(d.grassL, 0.f);
        s.gravelL = std::max(d.gravelL, 0.f);
        s.grassR = std::max(d.grassR, 0.f);
        s.gravelR = std::max(d.gravelR, 0.f);
        s.distance = dist;
        return s;
    };

    Sample prev = makeSample(dense.front(), 0.f);
    prev.forward = Normalize(dense[1].position - dense[0].position);
    prev.right = Normalize(Cross(Vec3{0.f, 1.f, 0.f}, prev.forward));
    samples_.push_back(prev);

    spacing_ = spacing;
    for (size_t i = 1; i < dense.size(); ++i) {
        const Vec3 seg = dense[i].position - dense[i - 1].position;
        const float segLen = Length(seg);
        if (segLen < 1e-4f) continue;
        const float segStart = accum;
        accum += segLen;

        while (sampledDist + spacing <= accum) {
            const float t = (sampledDist + spacing - segStart) / segLen;
            Dense d;
            d.position = dense[i - 1].position + seg * t;
            d.width = Lerp(dense[i - 1].width, dense[i].width, t);
            d.grassL = Lerp(dense[i - 1].grassL, dense[i].grassL, t);
            d.gravelL = Lerp(dense[i - 1].gravelL, dense[i].gravelL, t);
            d.grassR = Lerp(dense[i - 1].grassR, dense[i].grassR, t);
            d.gravelR = Lerp(dense[i - 1].gravelR, dense[i].gravelR, t);
            Sample s = makeSample(d, sampledDist + spacing);
            s.forward = Normalize(seg);
            s.right = Normalize(Cross(Vec3{0.f, 1.f, 0.f}, s.forward));
            samples_.push_back(s);
            prev = s;
            sampledDist += spacing;
        }
    }

    totalLength_ = samples_.empty() ? 0.f : accum;
    if (!samples_.empty()) {
        samples_.front().forward =
            Normalize(samples_[1].position - samples_.back().position);
        samples_.front().right =
            Normalize(Cross(Vec3{0.f, 1.f, 0.f}, samples_.front().forward));
    }

    // Per-sample horizontal curvature (1/radius) from consecutive headings,
    // lightly smoothed. A wide window halves real hairpin peaks and lets the
    // AI overshoot tight corners, so keep it at three taps. Pitch changes
    // must not count as curvature.
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
            for (int k = -1; k <= 1; ++k) {
                acc += raw[(i + static_cast<size_t>(k) + sampleCount) % sampleCount];
            }
            samples_[i].curvature = acc / 3.f;
        }
    }
}

void Track::Dispose(IRenderer& renderer) {
    const MeshHandle meshes[] = {
        grassBandMesh_, roadMesh_,       edgeLineMesh_,   kerbRedMesh_,   kerbWhiteMesh_,
        gravelBandMesh_, wallMesh_,      terrainMesh_,    treeMesh_,      treeMeshLarge_,
        startLineWhiteMesh_, startLineBlackMesh_};
    for (MeshHandle mesh : meshes) {
        if (mesh != kInvalidMesh) renderer.DestroyMesh(mesh);
    }
    grassBandMesh_ = roadMesh_ = edgeLineMesh_ = kerbRedMesh_ = kerbWhiteMesh_ =
        gravelBandMesh_ = wallMesh_ = terrainMesh_ = treeMesh_ = treeMeshLarge_ =
            startLineWhiteMesh_ = startLineBlackMesh_ = kInvalidMesh;
    wallBodies_.clear();
    groundBodies_.clear();
    trees_.clear();
    drsZones_.clear();
}

void Track::Build(const MapData& map, IPhysicsWorld& world, IRenderer& renderer) {
    Dispose(renderer);
    BuildOffline(map, world);
    BuildMeshes(map, renderer);
}

void Track::BuildOffline(const MapData& map, IPhysicsWorld& world) {
    Resample(map, 4.f);
    BuildPhysics(map, world);

    // DRS activation zones come from map data (map DRS lap fractions).
    // A map without zones.csv has no DRS — never guess a zone.
    for (const MapData::DrsZone& zone : map.drsZones) {
        drsZones_.push_back({zone.start * totalLength_, zone.end * totalLength_});
    }
}

void Track::BuildMeshes(const MapData& map, IRenderer& renderer) { BuildMesh(map, renderer); }

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
    for (const Sample& s : samples_) {
        const float err = LengthSq(s.position - worldPos);
        if (err < bestErr) {
            bestErr = err;
            bestDist = s.distance;
        }
    }
    return bestDist;
}

bool Track::InDrsZone(float distance) const {
    for (const Zone& zone : drsZones_) {
        if (zone.Contains(distance)) return true;
    }
    return false;
}

const Track::Sample* Track::NearestSample(const Vec3& pos, float& dist) const {
    // True HORIZONTAL distance to the centerline. Minimizing the lateral
    // projection (Dot(d, right)) instead makes every point in the loop
    // interior look "laterally aligned" with some far-away sample (the
    // projection of a radial vector onto the tangent frame is ~0), so the
    // terrain would adopt an arbitrary station's height and bury the road.
    const Sample* best = nullptr;
    float bestD2 = 1e30f;
    for (const Sample& s : samples_) {
        const float dx = pos.x - s.position.x;
        const float dz = pos.z - s.position.z;
        const float d2 = dx * dx + dz * dz;
        if (d2 < bestD2) {
            bestD2 = d2;
            best = &s;
        }
    }
    dist = std::sqrt(bestD2);
    return best;
}

void Track::BuildPhysics(const MapData& map, IPhysicsWorld& world) {
    // Safety plane below the whole scene (terrain minimum) so a car that
    // somehow escapes the walls never falls forever. Tagged Grass.
    BodyDesc ground;
    ground.transform.position = {0.f, map.MinTerrainY() - 2.5f, 0.f};
    ground.halfExtents = {1200.f, 2.f, 1200.f};
    ground.mass = 0.f;
    ground.friction = 0.7f;
    ground.userData = SurfaceTag(SurfaceType::Grass);
    if (auto groundId = world.CreateBody(ground)) {
        groundBodies_.push_back(*groundId);
    }

    // Layered slabs per sample: asphalt, then per-side grass and gravel
    // runoff bands (road edge outward), then the barrier wall. Boxes follow
    // the 3D tangent so elevation changes stay smooth for the raycast
    // suspension. userData carries the SurfaceType for grip / damage.
    for (const Sample& s : samples_) {
        const Vec3 up{0.f, 1.f, 0.f};
        const float w = s.width * 0.5f;
        const Quat rot = SlabRotation(s.forward, s.right);

        BodyDesc road;
        road.transform.position = s.position - up * 0.25f;
        road.transform.rotation = rot;
        road.halfExtents = {w, 0.25f, 2.2f};
        road.mass = 0.f;
        road.friction = 0.9f;
        road.restitution = 0.f;
        road.userData = SurfaceTag(SurfaceType::Road);
        (void)world.CreateBody(road);

        for (int side = -1; side <= 1; side += 2) {
            const float sgn = static_cast<float>(side);
            const float grass = (side > 0) ? s.grassR : s.grassL;
            const float gravel = (side > 0) ? s.gravelR : s.gravelL;
            float offset = w;

            if (grass > 0.3f) {
                BodyDesc slab;
                slab.transform.position =
                    s.position + s.right * (sgn * (offset + grass * 0.5f)) - up * 0.27f;
                slab.transform.rotation = rot;
                slab.halfExtents = {grass * 0.5f, 0.25f, 2.2f};
                slab.mass = 0.f;
                slab.friction = 0.7f;
                slab.userData = SurfaceTag(SurfaceType::Grass);
                (void)world.CreateBody(slab);
            }
            offset += grass;

            if (gravel > 0.3f) {
                BodyDesc slab;
                slab.transform.position =
                    s.position + s.right * (sgn * (offset + gravel * 0.5f)) - up * 0.30f;
                slab.transform.rotation = rot;
                slab.halfExtents = {gravel * 0.5f, 0.25f, 2.2f};
                slab.mass = 0.f;
                slab.friction = 0.5f;
                slab.userData = SurfaceTag(SurfaceType::Gravel);
                (void)world.CreateBody(slab);
            }
            offset += gravel;

            // Barrier wall just outside the outermost band.
            BodyDesc wall;
            wall.transform.position =
                s.position + s.right * (sgn * (offset + 0.6f)) + up * 0.5f;
            wall.transform.rotation = rot;
            wall.halfExtents = {0.6f, 0.7f, 2.2f};
            wall.mass = 0.f;
            wall.friction = 0.55f;
            wall.restitution = 0.12f;
            wall.userData = SurfaceTag(SurfaceType::Wall);
            if (auto wallId = world.CreateBody(wall)) {
                wallBodies_.push_back(*wallId);
            }
        }
    }
}

void Track::BuildMesh(const MapData& map, IRenderer& renderer) {
    const size_t n = samples_.size();
    if (n < 2) return;

    MeshData road, edgeLines, kerbRed, kerbWhite, walls, grassBands, gravelBands;

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
        // Surface normal of the ribbon: seg x right points UP (right x seg
        // points down and the quad gets backface-culled into invisibility).
        const Vec3 normal = Normalize(Cross(s1.position - s0.position, s0.right));

        // Asphalt ribbon.
        addQuad(road, s0.position - s0.right * w0, s0.position + s0.right * w0,
                s1.position - s1.right * w1, s1.position + s1.right * w1, normal);

        for (int side = -1; side <= 1; side += 2) {
            const float sgn = static_cast<float>(side);
            const float g0 = (side > 0) ? s0.grassR : s0.grassL;
            const float g1 = (side > 0) ? s1.grassR : s1.grassL;
            const float v0 = (side > 0) ? s0.gravelR : s0.gravelL;
            const float v1 = (side > 0) ? s1.gravelR : s1.gravelL;

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

            // Grass runoff band: road edge -> road edge + grass, just under
            // the asphalt plane (the physics slab tops sit 2 cm lower).
            if (g0 > 0.05f || g1 > 0.05f) {
                const Vec3 a0 = s0.position + s0.right * sgn * w0 - up * 0.02f;
                const Vec3 b0 = s0.position + s0.right * sgn * (w0 + g0) - up * 0.02f;
                const Vec3 a1 = s1.position + s1.right * sgn * w1 - up * 0.02f;
                const Vec3 b1 = s1.position + s1.right * sgn * (w1 + g1) - up * 0.02f;
                addQuad(grassBands, a0, b0, a1, b1, normal);
            }

            // Gravel trap beyond the grass band.
            if (v0 > 0.05f || v1 > 0.05f) {
                const Vec3 a0 = s0.position + s0.right * sgn * (w0 + g0) - up * 0.05f;
                const Vec3 b0 = s0.position + s0.right * sgn * (w0 + g0 + v0) - up * 0.05f;
                const Vec3 a1 = s1.position + s1.right * sgn * (w1 + g1) - up * 0.05f;
                const Vec3 b1 = s1.position + s1.right * sgn * (w1 + g1 + v1) - up * 0.05f;
                addQuad(gravelBands, a0, b0, a1, b1, normal);
            }

            // Barrier wall on the outer face of the runoff (matches the
            // physics wall boxes). Drawn double sided.
            const float wallH = 1.2f;
            const float outer0 = w0 + g0 + v0;
            const float outer1 = w1 + g1 + v1;
            const Vec3 base0 = s0.position + s0.right * sgn * outer0;
            const Vec3 base1 = s1.position + s1.right * sgn * outer1;
            addQuad(walls, base0, base1, base0 + up * wallH, base1 + up * wallH, -s0.right * sgn);
            addQuad(walls, base0, base1, base0 + up * wallH, base1 + up * wallH, s0.right * sgn);
        }
    }

    // Terrain base grid: bilinear height posts sampled on a 25 m lattice (the
    // posts stay sparse data; mesh resolution is an engine choice), blended
    // toward the track height near the corridor so the road never floats or
    // gets buried. The corridor plane sits below the gravel band (-0.06) so
    // the layered strips never z-fight with it, and the lattice is fine
    // enough that the undulating road cannot poke through the terrain.
    MeshData terrain;
    if (map.gridX.size() >= 2 && map.gridZ.size() >= 2) {
        const float inner = 62.f, outer = 115.f;
        auto corridorInfo = [&](float x, float z, float& dist, float& corridorY, float& lateral,
                                float& bandOuter) {
            const Sample* s = NearestSample(Vec3{x, 0.f, z}, dist);
            corridorY = s->position.y;
            const Vec3 d = Vec3{x, 0.f, z} - s->position;
            lateral = Dot(d, s->right);
            bandOuter = s->width * 0.5f +
                        ((lateral > 0.f) ? s->grassR + s->gravelR : s->grassL + s->gravelL);
        };
        auto blendedHeight = [&](float x, float z) {
            float dist = 0.f, corridorY = 0.f, lateral = 0.f, bandOuter = 0.f;
            corridorInfo(x, z, dist, corridorY, lateral, bandOuter);
            float h = map.TerrainHeight(x, z);
            if (dist < outer) {
                float t = Clampf((dist - inner) / (outer - inner), 0.f, 1.f);
                t = t * t * (3.f - 2.f * t);
                h = Lerp(corridorY - 0.12f, h, t);
            }
            return h;
        };

        const float step = 25.f;
        const float minX = map.gridX.front(), maxX = map.gridX.back();
        const float minZ = map.gridZ.front(), maxZ = map.gridZ.back();
        const size_t cols = static_cast<size_t>((maxX - minX) / step) + 1;
        const size_t rows = static_cast<size_t>((maxZ - minZ) / step) + 1;
        std::vector<Vec3> verts(cols * rows);
        for (size_t r = 0; r < rows; ++r) {
            for (size_t c = 0; c < cols; ++c) {
                const float x = std::min(minX + static_cast<float>(c) * step, maxX);
                const float z = std::min(minZ + static_cast<float>(r) * step, maxZ);
                verts[r * cols + c] = Vec3{x, blendedHeight(x, z), z};
            }
        }
        for (size_t r = 0; r + 1 < rows; ++r) {
            for (size_t c = 0; c + 1 < cols; ++c) {
                addQuad(terrain, verts[r * cols + c], verts[r * cols + c + 1],
                        verts[(r + 1) * cols + c], verts[(r + 1) * cols + c + 1], up);
            }
        }

        // Bake tree decor into two size buckets (Transform carries no scale).
        // Trees that end up inside the runoff bands are dropped: the built
        // spline curves wider than the authored polyline, so map data can
        // legitimately place decor where the real corridor ends up.
        trees_.clear();
        for (const MapData::Decor& d : map.decor) {
            float dist = 0.f, corridorY = 0.f, lateral = 0.f, bandOuter = 0.f;
            corridorInfo(d.position.x, d.position.z, dist, corridorY, lateral, bandOuter);
            if (std::fabs(lateral) < bandOuter + 2.5f) continue;
            TreeInstance t;
            t.position =
                Vec3{d.position.x, blendedHeight(d.position.x, d.position.z), d.position.z};
            t.scale = d.scale;
            trees_.push_back(t);
        }
    }

    // Unit tree (height 6 m): trunk box + two-tier low-poly canopy, baked at
    // instance scale/position with vertex colors.
    auto addTree = [&](MeshData& mesh, const Vec3& pos, float scale) {
        const uint32_t base = static_cast<uint32_t>(mesh.positions.size());
        const Color trunk{0.42f, 0.30f, 0.20f, 1.f};
        const Color leaf{0.14f, 0.36f, 0.16f, 1.f};
        auto pushV = [&](const Vec3& p, const Vec3& nrm, const Color& col) {
            mesh.positions.push_back(p);
            mesh.normals.push_back(nrm);
            mesh.colors.push_back(static_cast<uint8_t>(col.r * 255.f));
            mesh.colors.push_back(static_cast<uint8_t>(col.g * 255.f));
            mesh.colors.push_back(static_cast<uint8_t>(col.b * 255.f));
            mesh.colors.push_back(255);
        };
        // Trunk: 4-sided prism.
        const float tw = 0.16f * scale, th = 2.3f * scale;
        const Vec3 corners[4] = {
            pos + Vec3{-tw, 0.f, -tw}, pos + Vec3{tw, 0.f, -tw},
            pos + Vec3{tw, 0.f, tw}, pos + Vec3{-tw, 0.f, tw}};
        for (int k = 0; k < 4; ++k) {
            const Vec3& a = corners[k];
            const Vec3& b = corners[(k + 1) % 4];
            const Vec3 nrm = Normalize(Cross(b - a, Vec3{0.f, 1.f, 0.f}));
            pushV(a, nrm, trunk);
            pushV(b, nrm, trunk);
            pushV(a + Vec3{0.f, th, 0.f}, nrm, trunk);
            pushV(b + Vec3{0.f, th, 0.f}, nrm, trunk);
            const uint32_t i0 = base + static_cast<uint32_t>(k) * 4;
            mesh.indices.push_back(i0 + 0);
            mesh.indices.push_back(i0 + 1);
            mesh.indices.push_back(i0 + 2);
            mesh.indices.push_back(i0 + 1);
            mesh.indices.push_back(i0 + 3);
            mesh.indices.push_back(i0 + 2);
        }
        // Canopy: hexagon ring at 2.2 m, smaller ring at 4.1 m, apex at 5.8 m.
        const uint32_t canopyBase = static_cast<uint32_t>(mesh.positions.size());
        auto ring = [&](float radius, float y, const Vec3& nrmHint) {
            for (int k = 0; k < 6; ++k) {
                const float a = (static_cast<float>(k) / 6.f) * 2.f * kPi;
                const Vec3 dir{std::cos(a), 0.f, std::sin(a)};
                pushV(pos + dir * (radius * scale) + Vec3{0.f, y * scale, 0.f},
                      Normalize(dir + nrmHint), leaf);
            }
        };
        ring(1.55f, 2.15f, Vec3{0.f, 0.4f, 0.f});
        ring(1.05f, 4.05f, Vec3{0.f, 0.5f, 0.f});
        pushV(pos + Vec3{0.f, 5.8f * scale, 0.f}, Vec3{0.f, 1.f, 0.f}, leaf);
        for (int k = 0; k < 6; ++k) {
            const uint32_t a0 = canopyBase + static_cast<uint32_t>(k);
            const uint32_t a1 = canopyBase + static_cast<uint32_t>((k + 1) % 6);
            const uint32_t b0 = canopyBase + 6 + static_cast<uint32_t>(k);
            const uint32_t b1 = canopyBase + 6 + static_cast<uint32_t>((k + 1) % 6);
            mesh.indices.push_back(a0);
            mesh.indices.push_back(a1);
            mesh.indices.push_back(b0);
            mesh.indices.push_back(a1);
            mesh.indices.push_back(b1);
            mesh.indices.push_back(b0);
            mesh.indices.push_back(b0);
            mesh.indices.push_back(b1);
            mesh.indices.push_back(canopyBase + 12);
        }
    };
    MeshData treesSmall, treesLarge;
    for (const TreeInstance& t : trees_) {
        addTree(t.scale < 1.25f ? treesSmall : treesLarge, t.position, t.scale);
    }

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
    grassBandMesh_ = renderer.CreateMesh(grassBands);
    gravelBandMesh_ = renderer.CreateMesh(gravelBands);
    terrainMesh_ = renderer.CreateMesh(terrain);
    if (!treesSmall.positions.empty()) treeMesh_ = renderer.CreateMesh(treesSmall);
    if (!treesLarge.positions.empty()) treeMeshLarge_ = renderer.CreateMesh(treesLarge);
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
    if (terrainMesh_ != kInvalidMesh) {
        renderer.DrawMesh(terrainMesh_, origin, Color{0.20f, 0.42f, 0.20f, 1.f});
    }
    if (grassBandMesh_ != kInvalidMesh) {
        renderer.DrawMesh(grassBandMesh_, origin, Color{0.24f, 0.48f, 0.22f, 1.f});
    }
    if (gravelBandMesh_ != kInvalidMesh) {
        renderer.DrawMesh(gravelBandMesh_, origin, Color{0.58f, 0.50f, 0.36f, 1.f});
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
    if (treeMesh_ != kInvalidMesh) {
        renderer.DrawMesh(treeMesh_, origin, Color::White());
    }
    if (treeMeshLarge_ != kInvalidMesh) {
        renderer.DrawMesh(treeMeshLarge_, origin, Color::White());
    }
    if (startLineWhiteMesh_ != kInvalidMesh) {
        renderer.DrawMesh(startLineWhiteMesh_, origin, Color::White());
        renderer.DrawMesh(startLineBlackMesh_, origin, Color{0.1f, 0.1f, 0.1f, 1.f});
    }
}

}  // namespace apex