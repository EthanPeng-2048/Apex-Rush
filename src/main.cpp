#include <cstdio>
#include <cstring>

#include "core/Config.h"
#include "game/Game.h"
#include "render/RaylibBackend.h"

int main(int argc, char** argv) {
    using namespace apex;

    // `apex-rush --shot <path>`: run 90 frames, save a screenshot, exit.
    // Used for automated visual verification.
    const char* shotPath = nullptr;
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::strcmp(argv[i], "--shot") == 0) shotPath = argv[i + 1];
    }

    Config config;
    config.Load("config.ini");

    RaylibBackend backend;
    // Default to a size that fits small laptop work areas (e.g. 1366x768).
    const int width = config.GetInt("video", "width", 1280);
    const int height = config.GetInt("video", "height", 720);
    if (!backend.Init(width, height, "Apex Rush Online")) {
        std::fprintf(stderr, "Failed to create window\n");
        return 1;
    }

    Game game(backend, backend, config);
    if (!game.Init()) {
        backend.Shutdown();
        return 1;
    }

    int frame = 0;
    while (!backend.ShouldClose() && !game.WantsQuit()) {
        backend.BeginFrame();
        game.Update();
        backend.BeginScene();
        game.Render3D();
        backend.EndScene();
        game.RenderHUD();
        backend.EndFrame();
        if (shotPath && ++frame >= 90) {
            backend.CaptureScreenshot(shotPath);
            break;
        }
    }

    config.Save("config.ini");
    backend.Shutdown();
    return 0;
}
