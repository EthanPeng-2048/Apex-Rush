#pragma once

#include <string>
#include <unordered_map>

#include "render/IRenderer.h"

namespace apex {

// Headless renderer + scripted input. Implements the same unified interfaces
// as RaylibBackend with no window, so the game logic (physics, race rules,
// AI) can run in automated tests and on machines without a display. Doubles
// as proof that the render/input layer is replaceable.
class NullBackend final : public IRenderer, public IInput {
public:
    // Returned verbatim by Poll(); tests set this to drive the player car.
    InputState scripted;

    // Fixed timestep reported by FrameTime(); physics runs deterministically.
    float frameDt = 1.f / 60.f;

    // IRenderer -----------------------------------------------------------
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

    // IInput --------------------------------------------------------------
    InputState Poll() override;

    // Debug capture: record DrawMesh calls so the composed scene can be
    // exported (DumpObj) and inspected without a GPU/window.
    bool captureDraws = false;
    void DumpObj(const std::string& path) const;

private:
    struct DrawCmd {
        MeshHandle mesh = kInvalidMesh;
        Transform transform;
        Color tint;
    };

    int width_ = 1280;
    int height_ = 720;
    double time_ = 0.0;
    MeshHandle nextMesh_ = 1;
    std::unordered_map<MeshHandle, MeshData> meshes_;
    std::vector<DrawCmd> draws_;
    bool closeRequested_ = false;
};

}  // namespace apex
