#pragma once

#include "render/IRenderer.h"

namespace apex {

// Raylib implementation of the unified render/input interfaces. All raylib
// types stay inside this class so the game layer never sees them.
class RaylibBackend final : public IRenderer, public IInput {
public:
    ~RaylibBackend() override;

    // IRenderer
    bool Init(int width, int height, const std::string& title) override;
    void Shutdown() override;
    bool ShouldClose() const override;
    void BeginFrame() override;
    void BeginScene() override;
    void EndScene() override;
    void EndFrame() override;
    void SetCamera(const CameraDesc& camera) override;
    int ScreenWidth() const override;
    int ScreenHeight() const override;
    MeshHandle CreateMesh(const MeshData& data) override;
    void DestroyMesh(MeshHandle handle) override;
    void DrawMesh(MeshHandle handle, const Transform& transform, const Color& tint) override;
    void DrawBox(const Vec3& halfExtents, const Transform& transform, const Color& color) override;
    void DrawLine3D(const Vec3& start, const Vec3& end, const Color& color) override;
    void DrawGrid(int slices, float spacing) override;
    void DrawRect2D(float x, float y, float w, float h, const Color& color) override;
    void DrawText2D(const std::string& text, float x, float y, int size, const Color& color) override;
    Vec2f MeasureText2D(const std::string& text, int size) const override;
    double TimeSeconds() const override;
    float FrameTime() const override;
    void CaptureScreenshot(const std::string& path) override;

    // IInput
    InputState Poll() override;

private:
    struct Impl;
    Impl* impl_ = nullptr;
};

}  // namespace apex
