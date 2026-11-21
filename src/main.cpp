#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

#include "audio/MiniaudioAudio.h"
#include "core/Config.h"
#include "game/Game.h"
#include "physics/NudgeWorld.h"
#include "physics/Qu3eWorld.h"
#include "render/NullBackend.h"
#include "render/RaylibBackend.h"
#include "vehicle/Vehicle.h"

namespace {

using namespace apex;

// Swap the collision backend under test ([physics] engine).
std::unique_ptr<IPhysicsWorld> MakeWorld(const char* engine) {
    if (engine && std::strcmp(engine, "nudge") == 0) return std::make_unique<NudgeWorld>();
    return std::make_unique<Qu3eWorld>();
}

// High-speed steering stability on a bare ground plane. The circuit is a
// ~120 m radius loop, so any straight run drifts into the wall within ~3 s
// and wall contacts garble handling numbers; an infinite plane isolates the
// vehicle dynamics. Regression for the "slight steer at speed = spin" bug:
// a keyboard step to part lock used to ratchet to full lock (steering filter
// jumped straight to its too-large speed limit, and the ESC reference yaw was
// ~10x too large at speed so it kept adding steering) and snap the car.
int RunFlatStabilityTest(IPhysicsWorld& world) {
    world.SetGravity({0.f, -9.81f, 0.f});

    BodyDesc ground;
    ground.transform.position = {0.f, -1.f, 0.f};
    ground.halfExtents = {2000.f, 1.f, 2000.f};
    ground.friction = 0.8f;
    world.CreateBody(ground);

    VehicleParams params;
    Transform pose;
    pose.position = {0.f, 0.55f, 0.f};
    Vehicle car;
    car.Spawn(world, pose, params);

    int failures = 0;
    auto check = [&failures](bool ok, const char* what, float got, float want) {
        std::printf("[%s] %s (got %.2f, target %.2f)\n", ok ? "PASS" : "FAIL", what, got, want);
        if (!ok) ++failures;
    };

    const float dt = 1.f / 240.f;  // matches Game::kFixedDt
    auto step = [&](const VehicleInput& in, float seconds) {
        const int n = static_cast<int>(seconds / dt + 0.5f);
        for (int i = 0; i < n; ++i) {
            car.Update(in, dt);
            world.Step(dt);
        }
    };
    auto heading = [&]() {
        return TransformDirection(car.State().transform, Vec3{0.f, 0.f, 1.f});
    };
    auto sideSlip = [&]() {
        const VehicleState& s = car.State();
        return std::fabs(Dot(s.velocity, Cross(heading(), Vec3{0.f, 1.f, 0.f})));
    };

    step(VehicleInput{}, 1.0f);  // settle on the suspension

    // Phase A: mid speed. Even at ~75 km/h the old ESC reference already
    // ratcheted a 0.25 tap to full lock.
    {
        VehicleInput accel;
        accel.throttle = 1.f;
        step(accel, 2.2f);
        const float v0 = car.State().speed;
        const Vec3 heading0 = heading();

        VehicleInput tap;
        tap.throttle = 0.3f;
        tap.steer = 0.25f;
        step(tap, 1.0f);

        const VehicleState& s = car.State();
        check(v0 > 15.f, "flat: reach mid speed", v0, 15.f);
        const float range = car.SteerEnvelopeDeg(s.speed);
        // 0.25 tap + the steering-assist counter-steer trim (up to 0.85x the
        // envelope once slipping) + slack for the input easing.
        const float bound = 0.25f * range + 0.85f * range + 0.5f;
        check(std::fabs(s.steerAngleDeg) < bound, "flat: steer within speed limit",
              s.steerAngleDeg, bound);
        const float turnDeg =
            std::acos(Clampf(Dot(heading0, heading()), -1.f, 1.f)) * kRadToDeg;
        check(turnDeg < 25.f, "flat: no snap spin at mid speed", turnDeg, 25.f);
        check(sideSlip() < 3.f, "flat: low side slip at mid speed", sideSlip(), 3.f);
        std::printf("       mid-speed tap: v=%.1f km/h\n", s.speedKmh);
    }

    // Phase B: keep the throttle pinned to true racing speed (~200 km/h),
    // then repeat the tap. This is the regime where a slight steer used to
    // spin the car instantly.
    {
        VehicleInput accel;
        accel.throttle = 1.f;
        step(accel, 10.f);
        const float v0 = car.State().speed;
        const Vec3 heading0 = heading();

        VehicleInput tap;
        tap.throttle = 0.2f;
        tap.steer = 0.25f;
        step(tap, 1.2f);

        const VehicleState& s = car.State();
        check(v0 > 40.f, "flat: reach racing speed", v0, 40.f);
        const float range = car.SteerEnvelopeDeg(s.speed);
        const float bound = 0.25f * range + 0.85f * range + 0.5f;
        check(std::fabs(s.steerAngleDeg) < bound, "flat: steer within speed limit",
              s.steerAngleDeg, bound);
        const float turnDeg =
            std::acos(Clampf(Dot(heading0, heading()), -1.f, 1.f)) * kRadToDeg;
        check(turnDeg < 20.f, "flat: no snap spin at racing speed", turnDeg, 20.f);
        check(sideSlip() < 3.5f, "flat: low side slip at racing speed", sideSlip(), 3.5f);
        std::printf("       high-speed tap: v=%.1f km/h\n", s.speedKmh);
    }

    return failures;
}

// Menu/settings flow on the headless backend: boots into the menu (no
// autostart), opens settings, comes back, starts a race — plus a render
// smoke pass over the menu screens. Uses the real config.ini but never
// changes a value, so the settings save round-trips the user's config.
// Deliberately backend-agnostic and free of config overrides: leaving the
// settings screen calls Config::Save, which would persist any override.
int RunMenuTest() {
    NullBackend backend;
    if (!backend.Init(1280, 720, "menu-test")) {
        std::printf("[FAIL] menu-test backend init\n");
        return 1;
    }

    Config config;
    config.Load("config.ini");
    Game game(backend, backend, config);
    if (!game.Init()) {
        std::printf("[FAIL] menu-test game init\n");
        return 1;
    }

    int failures = 0;
    auto checkPhase = [&failures](bool ok, const char* what, RacePhase got, RacePhase want) {
        std::printf("[%s] %s (got %d, want %d)\n", ok ? "PASS" : "FAIL", what,
                    static_cast<int>(got), static_cast<int>(want));
        if (!ok) ++failures;
    };
    auto frame = [&](InputState in) {
        backend.scripted = in;
        backend.BeginFrame();
        game.Update();
        game.RenderHUD();
        backend.EndFrame();
    };

    checkPhase(game.Phase() == RacePhase::Menu, "menu: boots into menu", game.Phase(),
               RacePhase::Menu);

    InputState press;
    press.menuDown = true;  // cursor -> SETTINGS
    frame(press);
    press = InputState{};
    press.menuSelect = true;
    frame(press);
    checkPhase(game.Phase() == RacePhase::Settings, "menu: settings opens", game.Phase(),
               RacePhase::Settings);

    press = InputState{};
    press.menuDown = true;
    // Walk the whole list to the BACK row. Selecting anywhere else would
    // change a value and Save() it into the real config.ini — the row count
    // must stay in sync with Game::kSettingsRowCount.
    for (int i = 0; i < Game::kSettingsRowCount - 1 && game.Phase() == RacePhase::Settings; ++i) {
        frame(press);
    }
    press = InputState{};
    press.menuSelect = true;  // BACK row
    frame(press);
    checkPhase(game.Phase() == RacePhase::Menu, "menu: settings returns", game.Phase(),
               RacePhase::Menu);

    press = InputState{};
    press.menuSelect = true;  // START RACE
    frame(press);
    checkPhase(game.Phase() == RacePhase::Countdown, "menu: start race -> countdown", game.Phase(),
               RacePhase::Countdown);

    return failures;
}

// AI autopilot regression: all four cars AI-driven must stay inside the track
// corridor (no wall planting) and make real progress. The player car runs the
// same AI via [race] autopilot.
int RunAITest(const char* engine) {
    NullBackend backend;
    if (!backend.Init(1280, 720, "ai-test")) {
        std::printf("[FAIL] ai-test backend init\n");
        return 1;
    }

    Config config;
    config.Load("config.ini");
    config.SetString("race", "cars", "4");
    config.SetString("race", "autostart", "true");
    config.SetString("race", "autopilot", "true");
    config.SetString("physics", "engine", engine);
    // Pin the autopilot player car's assists so the AI regression stays
    // deterministic regardless of the player's difficulty settings (AI cars
    // themselves run fixed assists and never read these).
    config.SetString("assists", "steer", "full");
    config.SetString("assists", "esc", "full");
    config.SetString("assists", "tc", "sport");
    config.SetString("assists", "abs", "sport");

    Game game(backend, backend, config);
    if (!game.Init()) {
        std::printf("[FAIL] ai-test game init\n");
        return 1;
    }

    int failures = 0;
    auto check = [&failures](bool ok, const char* what, float got, float want) {
        std::printf("[%s] %s (got %.2f, target %.2f)\n", ok ? "PASS" : "FAIL", what, got, want);
        if (!ok) ++failures;
    };
    auto frame = [&]() {
        backend.scripted = InputState{};
        backend.BeginFrame();
        game.Update();
        backend.EndFrame();
    };

    for (int i = 0; i < 240; ++i) frame();  // countdown + settle

    const Track& track = game.GetTrack();
    const size_t n = game.Racers().size();
    std::vector<float> traveled(n, 0.f), maxOffset(n, 0.f), prevD(n, 0.f);
    for (size_t k = 0; k < n; ++k) {
        prevD[k] = track.ProjectDistance(game.Racers()[k].vehicle.State().transform.position);
    }

    const float total = track.TotalLength();
    for (int i = 0; i < 1500; ++i) {  // 25 s of racing
        frame();
        for (size_t k = 0; k < n; ++k) {
            const VehicleState& s = game.Racers()[k].vehicle.State();
            const float d = track.ProjectDistance(s.transform.position);
            float step = d - prevD[k];
            if (step < -total * 0.5f) step += total;
            else if (step > total * 0.5f) step -= total;
            traveled[k] += step;
            prevD[k] = d;

            const Vec3 rt = Normalize(Cross(Vec3{0.f, 1.f, 0.f}, track.ForwardAt(d)));
            const float off = std::fabs(Dot(s.transform.position - track.PositionAt(d), rt));
            maxOffset[k] = std::max(maxOffset[k], off);
        }
    }

    for (size_t k = 0; k < n; ++k) {
        char name[64];
        std::snprintf(name, sizeof(name), "ai[%zu]: makes progress", k);
        check(traveled[k] > 250.f, name, traveled[k], 250.f);
        const float bound = track.WidthAt(prevD[k]) * 0.5f + 0.5f;
        std::snprintf(name, sizeof(name), "ai[%zu]: stays in corridor", k);
        check(maxOffset[k] < bound, name, maxOffset[k], bound);
    }
    return failures;
}

// AI difficulty isolation: player assist settings must not leak into AI
// cars. Regression for "AI spins at assists-off difficulty" — with every
// player assist off the AI must still run its fixed aids and keep racing.
int RunAIDifficultyIsolationTest(const char* engine) {
    NullBackend backend;
    if (!backend.Init(1280, 720, "ai-diff-test")) {
        std::printf("[FAIL] ai-diff-test backend init\n");
        return 1;
    }

    Config config;
    config.Load("config.ini");
    config.SetString("race", "cars", "2");
    config.SetString("race", "autostart", "true");
    config.SetString("physics", "engine", engine);
    config.SetString("assists", "steer", "off");
    config.SetString("assists", "esc", "off");
    config.SetString("assists", "tc", "off");
    config.SetString("assists", "abs", "off");

    Game game(backend, backend, config);
    if (!game.Init()) {
        std::printf("[FAIL] ai-diff-test game init\n");
        return 1;
    }

    int failures = 0;
    auto check = [&failures](bool ok, const char* what, float got, float want) {
        std::printf("[%s] %s (got %.2f, target %.2f)\n", ok ? "PASS" : "FAIL", what, got, want);
        if (!ok) ++failures;
    };

    const VehicleParams& aiParams = game.Racers()[1].vehicle.Params();
    const VehicleParams& playerParams = game.Racers()[0].vehicle.Params();
    check(playerParams.steerLevel == AssistLevel::Off, "diff: player keeps off assists",
          static_cast<float>(playerParams.steerLevel), 0.f);
    check(aiParams.steerLevel == AssistLevel::Full, "diff: ai steer assist fixed",
          static_cast<float>(aiParams.steerLevel), 2.f);
    check(aiParams.escLevel == AssistLevel::Full, "diff: ai esc assist fixed",
          static_cast<float>(aiParams.escLevel), 2.f);

    const Track& track = game.GetTrack();
    const float d0 = track.ProjectDistance(game.Racers()[1].vehicle.State().transform.position);
    // The player starts one grid slot ahead, so drive it away at full
    // throttle — a parked player car would trip the AI traffic limiter.
    InputState driveAway;
    driveAway.throttle = 1.f;
    for (int i = 0; i < 720; ++i) {
        backend.scripted = driveAway;
        backend.BeginFrame();
        game.Update();
        backend.EndFrame();
    }
    const VehicleState& s = game.Racers()[1].vehicle.State();
    float traveled = track.ProjectDistance(s.transform.position) - d0;
    if (traveled < -track.TotalLength() * 0.5f) traveled += track.TotalLength();
    check(traveled > 150.f, "diff: ai makes progress at off difficulty", traveled, 150.f);
    const Vec3 fwd = TransformDirection(s.transform, Vec3{0.f, 0.f, 1.f});
    check(Dot(fwd, track.ForwardAt(track.ProjectDistance(s.transform.position))) > 0.3f,
          "diff: ai does not spin at off difficulty",
          Dot(fwd, track.ForwardAt(track.ProjectDistance(s.transform.position))), 0.3f);
    return failures;
}

// `apex-rush --drive-test`: headless physics/controls verification. Drives the
// player car with scripted input and asserts acceleration, braking, steering
// and reverse. Exits 0 on success, 1 on failure. No window is created, so it
// runs on locked sessions and in CI. The full suite runs once per physics
// backend (qu3e and nudge must both pass it).
int RunDriveTestForEngine(Config& config, const char* engine) {
    config.SetString("physics", "engine", engine);
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
    run(1.0f, turn);
    // Deterministic mapping check: steer = +1 must point the front wheels to
    // the right of the nose. In the vehicle frame (+Z forward, +Y up, right
    // handed) a right turn is a negative yaw, so the steer angle is negative.
    // Sign only: the circuit curves away, so wall contact near the end of the
    // run can legitimately counter-steer the wheel back toward centre.
    check(game.PlayerState().steerAngleDeg < -0.3f, "steer +1 points wheels right",
          game.PlayerState().steerAngleDeg, 0.f);
    const Vec3 headingAfter =
        TransformDirection(game.PlayerState().transform, Vec3{0.f, 0.f, 1.f});
    const float turnDeg = std::acos(Clampf(Dot(headingBefore, headingAfter), -1.f, 1.f)) * kRadToDeg;
    // Speed-sensitive steering trims the max angle, so the same 0.4 input
    // bends the car less than before; 13 deg in 1 s is still a clear turn.
    check(turnDeg > 13.f, "steering changes heading", turnDeg, 13.f);
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

    // 5) High-speed steering stability lives on a bare ground plane — see
    //    RunFlatStabilityTest. (The circuit loop is too tight to reach racing
    //    speed without hitting the barriers.)
    {
        std::unique_ptr<IPhysicsWorld> flatWorld = MakeWorld(engine);
        failures += RunFlatStabilityTest(*flatWorld);
    }

    // 6) Menu / settings flow (headless, backend-agnostic).
    failures += RunMenuTest();

    // 7) AI autopilot: no wall planting, real progress.
    failures += RunAITest(engine);

    // 8) AI cars ignore player difficulty assists (never spin at assists-off).
    failures += RunAIDifficultyIsolationTest(engine);

    std::printf("drive-test[%s]: %s (%d failures)\n", engine,
                failures == 0 ? "OK" : "FAILED", failures);
    backend.Shutdown();
    return failures;
}

int RunDriveTest(Config& config) {
    int failures = 0;
    for (const char* engine : {"qu3e", "nudge"}) {
        std::printf("=== physics backend: %s ===\n", engine);
        const auto start = std::chrono::steady_clock::now();
        failures += RunDriveTestForEngine(config, engine);
        const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start;
        std::printf("drive-test[%s] wall time: %.2f s\n", engine, elapsed.count());
    }
    std::printf("drive-test: %s (%d failures)\n", failures == 0 ? "OK" : "FAILED", failures);
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
    bool menuShot = false;
    bool settingsShot = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--drive-test") == 0) {
            driveTest = true;
        } else if (std::strcmp(argv[i], "--shot") == 0 && i + 1 < argc) {
            shotPath = argv[++i];
        } else if (std::strcmp(argv[i], "--menu") == 0) {
            menuShot = true;  // keep the main menu up for --shot (QA visual)
        } else if (std::strcmp(argv[i], "--settings") == 0) {
            settingsShot = true;  // open the settings screen for --shot (QA visual)
        } else if (std::strcmp(argv[i], "--dump-scene") == 0 && i + 1 < argc) {
            dumpPath = argv[++i];
        }
    }

    Config config;
    config.Load("config.ini");
    // All CLI automation paths (tests, screenshots, scene dumps) skip the menu.
    if ((driveTest || dumpPath || shotPath) && !menuShot && !settingsShot) {
        config.SetString("race", "autostart", "true");
    }
    if (settingsShot) config.SetString("race", "settings_qa", "true");

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

    // Interactive runs get sound (miniaudio: WASAPI/DirectSound on Windows).
    // Tests and CLI capture paths stay silent and headless.
    MiniaudioAudio audio;
    if (!audio.Init(config.GetString("audio", "backend", "auto").c_str())) {
        std::fprintf(stderr, "audio: device init failed, running silent\n");
    }

    Game game(backend, backend, config, &audio);
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

    // A --shot/--settings run injects automation overrides; saving the config
    // then would leak autostart=true / settings_qa into the user's config.ini
    // and the main menu would silently disappear on the next launch.
    if (!shotPath && !settingsShot) config.Save("config.ini");
    backend.Shutdown();
    return 0;
}
