#include "render/NullBackend.h"

#include <cstdio>

namespace apex {

bool NullBackend::Init(int width, int height, const std::string&) {
    width_ = width;
    height_ = height;
    return true;
}

void NullBackend::Shutdown() { meshes_.clear(); }

bool NullBackend::ShouldClose() const { return closeRequested_; }

void NullBackend::BeginFrame() { time_ += frameDt; }
void NullBackend::BeginScene() {}
void NullBackend::EndScene() {}
void NullBackend::EndFrame() {}

void NullBackend::SetCamera(const CameraDesc&) {}
int NullBackend::ScreenWidth() const { return width_; }
int NullBackend::ScreenHeight() const { return height_; }

MeshHandle NullBackend::CreateMesh(const MeshData& data) {
    const MeshHandle handle = nextMesh_++;
    meshes_[handle] = data;
    return handle;
}

void NullBackend::DestroyMesh(MeshHandle handle) { meshes_.erase(handle); }

void NullBackend::DrawMesh(MeshHandle handle, const Transform& transform, const Color& tint) {
    if (captureDraws) draws_.push_back({handle, transform, tint});
}
void NullBackend::DrawBox(const Vec3&, const Transform&, const Color&) {}
void NullBackend::DrawLine3D(const Vec3&, const Vec3&, const Color&) {}
void NullBackend::DrawGrid(int, float) {}
void NullBackend::DrawRect2D(float, float, float, float, const Color&) {}
void NullBackend::DrawText2D(const std::string&, float, float, int, const Color&) {}

Vec2f NullBackend::MeasureText2D(const std::string& text, int size) const {
    return {static_cast<float>(text.size()) * size * 0.5f, static_cast<float>(size)};
}

double NullBackend::TimeSeconds() const { return time_; }
float NullBackend::FrameTime() const { return frameDt; }
void NullBackend::CaptureScreenshot(const std::string&) {}

InputState NullBackend::Poll() { return scripted; }

void NullBackend::DumpObj(const std::string& path) const {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return;
    int base = 1;
    for (size_t d = 0; d < draws_.size(); ++d) {
        const DrawCmd& cmd = draws_[d];
        const auto it = meshes_.find(cmd.mesh);
        if (it == meshes_.end()) continue;
        const MeshData& mesh = it->second;
        std::fprintf(f, "g draw%zu\n# tint %.3f %.3f %.3f\n", d, cmd.tint.r, cmd.tint.g, cmd.tint.b);
        for (const Vec3& p : mesh.positions) {
            const Vec3 w = Rotate(cmd.transform.rotation, p) + cmd.transform.position;
            std::fprintf(f, "v %.4f %.4f %.4f\n", w.x, w.y, w.z);
        }
        for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
            std::fprintf(f, "f %d %d %d\n", base + static_cast<int>(mesh.indices[i]),
                         base + static_cast<int>(mesh.indices[i + 1]),
                         base + static_cast<int>(mesh.indices[i + 2]));
        }
        base += static_cast<int>(mesh.positions.size());
    }
    std::fclose(f);
}

}  // namespace apex
