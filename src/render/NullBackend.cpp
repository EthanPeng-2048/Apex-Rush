#include "render/NullBackend.h"

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

void NullBackend::DrawMesh(MeshHandle, const Transform&, const Color&) {}
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

}  // namespace apex
