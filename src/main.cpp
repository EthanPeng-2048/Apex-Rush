#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "core/Config.h"
#include "game/Game.h"
#include "render/NullBackend.h"
#include "render/RaylibBackend.h"

namespace {

using namespace apex;

// `apex-rush --drive-test`: headless physics/controls verification. Drives the
// player car with scripted input and asserts acceleration, braking, steering
// and reverse. Exits 0 on success, 1 on failure. No window is created, so it
// runs on locked sessions and in CI.
int RunDriveTest(Config& config) {
    NullBackend backend;
    if (!backend.Init(1280, 720, "drive-test")) {
        std::printf("[FAIL] NullBackend init\n");
        return 1;
    }

    // Solo run: AI cars would ram the stationary test car and skew the numbers.
    config.SetString("race", "cars", "1");

    Game game(backend, backend, config);
    if (!game.Init()) {
        std::printf("[FAIL] Game init\n");
        return 1;
    }

    int failures = 0;
    auto check = [&failures](bool ok, const char* what, float got, float want) {
        std::printf("[%s] %s (got %.2f, target %.2f)\n", ok ? "PASS" : "FAIL", what, got, want);
        if (!ok) ++failures;
    };

    // The update loop mirrors main: Poll() returns our scripted state.
    auto run = [&](float seconds, InputState input) {
        backend.scripted = input;
        const int frames = static_cast<int>(seconds / backend.frameDt + 0.5f);
        for (int i = 0; i < frames; ++i) {
            backend.BeginFrame();
            game.Update();
            backend.EndFrame();
        }
    };

    // 0) Transform matrix transcription. A transposed matrix inverts the
    // rotation and buries the translation in the w row (stretched "spider
    // web" geometry in-game), so the 16 floats must match TransformPoint.
    {
        Transform tr;
        tr.position = Vec3{1.f, 2.f, 3.f};
        tr.rotation = Quat::FromAxisAngle(Vec3{0.f, 1.f, 0.f}, 90.f * kDegToRad);
        float m[16];
        TransformToMatrix(tr, m);
        const Vec3 basis[3] = {{1.f, 0.f, 0.f}, {0.f, 1.f, 0.f}, {0.f, 0.f, 1.f}};
        float worst = 0.f;
        for (const Vec3& b : basis) {
            const Vec3 viaMatrix{m[0] * b.x + m[1] * b.y + m[2] * b.z + m[3],
                                 m[4] * b.x + m[5] * b.y + m[6] * b.z + m[7],
                                 m[8] * b.x + m[9] * b.y + m[10] * b.z + m[11]};
            const float err = Length(viaMatrix - TransformPoint(tr, b));
            if (err > worst) worst = err;
        }
        check(worst < 1e-4f, "transform matrix == TransformPoint", worst, 0.f);
    }

    // Wait out the 3.5 s countdown; the car settles onto its suspension.
    run(4.f, InputState{});
    if (game.Phase() != RacePhase::Racing) {
        std::printf("[FAIL] countdown never finished (phase=%d)\n",
                    static_cast<int>(game.Phase()));
        return 1;
    }
    std::printf("[PASS] countdown -> racing\n");

    // Wheel telemetry: verifies raycast suspension finds the ground and the
    // static corner load matches mass/4 (~1840 N for a 750 kg car).
    {
        const VehicleState& s = game.PlayerState();
        std::printf("       car y=%.3f  speed=%.2f\n", s.transform.position.y, s.speed);
        for (int i = 0; i < 4; ++i) {
            const WheelState& w = s.wheels[i];
            std::printf("       wheel[%d] grounded=%d suspLen=%.3f load=%.0f\n", i,
                        w.grounded ? 1 : 0, w.suspensionLength, w.load);
        }
    }

    const Vec3 startPos = game.PlayerState().transform.position;

    // 1) Acceleration: full throttle. Keep it to 2.5 s — the circuit curves
    // away and a longer straight run drives the car into the barrier, which
    // caps the speed and ruins the measurement.
    InputState throttle;
    throttle.throttle = 1.f;
    run(2.5f, throttle);
    const float accelSpeed = game.PlayerState().speed;
    const float forwardGain =
        Dot(game.PlayerState().transform.position - startPos,
            TransformDirection(game.PlayerState().transform, Vec3{0.f, 0.f, 1.f}));
    check(accelSpeed > 12.f, "throttle accelerates car", accelSpeed, 12.f);
    check(forwardGain > 10.f, "car moves forward along heading", forwardGain, 10.f);
    std::printf("       speed %.1f km/h, gear %d, rpm %.0f\n", game.PlayerState().speedKmh,
                game.PlayerState().gear, game.PlayerState().rpm);

    // 2) Steering at speed. Keep it gentle — at 60 km/h full lock asks for
    // ~3.4g and plows the car into the barriers, which garbles the heading.
    const Vec3 headingBefore =
        TransformDirection(game.PlayerState().transform, Vec3{0.f, 0.f, 1.f});
    InputState turn;
    turn.throttle = 0.35f;
    turn.steer = 0.4f;
    run(0.8f, turn);
    // Deterministic mapping check: steer = +1 must point the front wheels to
    // the right of the nose. In the vehicle frame (+Z forward, +Y up, right
    // handed) a right turn is a negative yaw, so the steer angle is negative.
    check(game.PlayerState().steerAngleDeg < -2.f, "steer +1 points wheels right",
          game.PlayerState().steerAngleDeg, 0.f);
    const Vec3 headingAfter =
        TransformDirection(game.PlayerState().transform, Vec3{0.f, 0.f, 1.f});
    const float turnDeg = std::acos(Clampf(Dot(headingBefore, headingAfter), -1.f, 1.f)) * kRadToDeg;
    check(turnDeg > 15.f, "steering changes heading", turnDeg, 15.f);
    // steer = +1 (D / right arrow) must swing the nose toward the car's
    // geometric right side = forward x up (=-X when the car faces +Z).
    const Vec3 rightDir = Cross(headingBefore, Vec3{0.f, 1.f, 0.f});
    const float rightSwing = Dot(headingAfter, rightDir);
    check(rightSwing > 0.05f, "steer +1 turns right", rightSwing, 0.f);

    // 3) Braking: full brake for 2 s from whatever speed we reached.
    InputState brake;
    brake.brake = 1.f;
    const float beforeBrake = game.PlayerState().speed;
    run(2.f, brake);
    const float afterBrake = game.PlayerState().speed;
    check(afterBrake < beforeBrake * 0.35f, "brake decelerates car", beforeBrake - afterBrake,
          beforeBrake * 0.65f);

    // 4) Reverse gear: like the real thing it only engages near a standstill.
    // Fresh grid first — the steering/braking run above may have parked the
    // car against a barrier, and reverse must be tested from a clean stop.
    InputState restart;
    restart.restart = true;
    run(0.05f, restart);
    run(4.f, InputState{});
    InputState reverse;
    reverse.throttle = 1.f;
    reverse.reverse = true;
    run(1.5f, reverse);
    {
        const VehicleState& s = game.PlayerState();
        std::printf("       reverse: fwd=%.2f gear=%d throttle=%.2f\n", s.forwardSpeed, s.gear,
                    s.throttle);
    }
    check(game.PlayerState().forwardSpeed < -1.f, "reverse moves car backwards",
          game.PlayerState().forwardSpeed, -1.f);

    std::printf("drive-test: %s (%d failures)\n", failures == 0 ? "OK" : "FAILED", failures);
    backend.Shutdown();
    return failures == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    using namespace apex;

    // `apex-rush --shot <path>`: run 90 frames, save a screenshot, exit.
    // `apex-rush --dump-scene <path.obj>`: headless, export the composed scene
    // (track + cars with draw transforms) for offline inspection.
    const char* shotPath = nullptr;
    const char* dumpPath = nullptr;
    bool driveTest = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--drive-test") == 0) {
            driveTest = true;
        } else if (std::strcmp(argv[i], "--shot") == 0 && i + 1 < argc) {
            shotPath = argv[++i];
        } else if (std::strcmp(argv[i], "--dump-scene") == 0 && i + 1 < argc) {
            dumpPath = argv[++i];
        }
    }

    Config config;
    config.Load("config.ini");

    if (driveTest) {
        return RunDriveTest(config);
    }

    if (dumpPath) {
        NullBackend backend;
        backend.captureDraws = true;
        backend.Init(1280, 720, "dump-scene");
        config.SetString("race", "cars", "2");
        Game game(backend, backend, config);
        if (!game.Init()) return 1;
        for (int i = 0; i < 260; ++i) {  // settle past the countdown
            backend.BeginFrame();
            game.Update();
            backend.EndFrame();
        }
        game.Render3D();
        backend.DumpObj(dumpPath);
        backend.Shutdown();
        std::printf("scene dumped to %s\n", dumpPath);
        return 0;
    }

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
