#include <algorithm>
#include <vector>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include <rush/rush.h>
#include "physics/raycast_probe.h"  // TEMP probe — remove with the perf work

import rush.engine.core;
import rush.engine.job;

#include "audio/MiniaudioAudio.h"
#include "game/Game.h"
import rush.engine.physics;
import rush.engine.render;
import rush.engine.input;

namespace {

using namespace apex;

// Swap the collision backend under test ([physics] engine).
std::unique_ptr<IPhysicsWorld> MakeWorld(const char* engine) {
    if (engine && std::strcmp(engine, "nudge") == 0) return std::make_unique<NudgeWorld>();
    if (engine && std::strcmp(engine, "rpc") == 0) return std::make_unique<RushPhysicsCore>();
    return std::make_unique<Qu3eWorld>();
}

// Scripted test input: NullInputSource + InputSystem wired for headless
// tests. set() translates a semantic InputState (legacy field set) into the
// raw keys/axes the recognizers consume, so test code keeps its old shape
// while the game consumes pure InputBus events.
struct ScriptedInput {
    NullInputSource source;
    InputSystem system{source};

    // Discrete flags publish their Click events directly. The legacy tests
    // treat "press menuDown = true for N frames" as N cursor moves, but a
    // Click recognizer fires only once per key-down edge. Publishing the
    // edge event per frame reproduces the old scripted semantics exactly;
    // Hold actions below still run through the gesture recognizers.
    void press(const char* name) {
        InputEvent ev;
        ev.nameId = InternInputName(std::string(name) + ".click");
        ev.gesture = Gesture::Click;
        ev.value = 1.f;
        ev.tSec = system.Now();
        InputBus().Publish(ev);
    }

    void set(const InputState& s) {
        RawFrame f;
        f.axes[0] = s.throttle;   // 0..1 analog (keyboard gives 0/1)
        f.axes[1] = s.brake;
        f.axes[2] = s.steer;      // -1..1 analog
        f.keys[static_cast<size_t>(KeyCode::W)] = s.throttle > 0.5f;
        f.keys[static_cast<size_t>(KeyCode::S)] = s.brake > 0.5f;
        f.keys[static_cast<size_t>(KeyCode::Space)] = s.handbrake;
        f.keys[static_cast<size_t>(KeyCode::R)] = s.reverse;
        f.keys[static_cast<size_t>(KeyCode::LShift)] = s.overtake;
        f.keys[static_cast<size_t>(KeyCode::B)] = s.lookBack;
        source.scripted = f;

        // Edge-triggered actions: one event per set() call (old IsKeyPressed
        // semantics — a press held across frames keeps moving the cursor).
        if (s.drsToggle) press("drs");
        if (s.wingModeToggle) press("wing");
        if (s.cameraCycle) press("camera");
        if (s.pause) press("pause");
        if (s.restart) press("restart");
        if (s.menuUp) press("menu.up");
        if (s.menuDown) press("menu.down");
        if (s.menuLeft) press("menu.left");
        if (s.menuRight) press("menu.right");
        if (s.menuSelect) press("menu.select");
        if (s.menuBack) press("menu.back");
    }
};

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
    (void)world.CreateBody(ground);

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
    auto step = [&](const CarInput& in, float seconds) {
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

    step(CarInput{}, 1.0f);  // settle on the suspension

    // Phase A: mid speed. Even at ~75 km/h the old ESC reference already
    // ratcheted a 0.25 tap to full lock.
    {
        CarInput accel;
        accel.throttle = 1.f;
        step(accel, 2.2f);
        const float v0 = car.State().speed;
        const Vec3 heading0 = heading();

        CarInput tap;
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
        CarInput accel;
        accel.throttle = 1.f;
        step(accel, 10.f);
        const float v0 = car.State().speed;
        const Vec3 heading0 = heading();

        CarInput tap;
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

        // Regression "cornering at speed blows half the car off": contact
        // damage used to count the full driving speed as impact speed, so a
        // floor/kerb scrape while cornering at 200+ km/h dealt wall-crash
        // damage to the outer side. A clean high-speed run on open tarmac
        // must leave all bodywork untouched.
        for (int i = 0; i < 4; ++i) {
            const float health = car.PartHealth(static_cast<CarPart>(i));
            check(health > 0.95f, "flat: no phantom damage at racing speed", health, 1.f);
        }
    }

    return failures;
}

// Component-level differential regression (backend-agnostic — the Differential
// never touches IPhysicsWorld, so it runs once per drive-test, not per engine):
//  1. spool-equivalence baseline: preload=ramp=0 with lockFactor=1 splits
//     exactly 50:50 (bias = 0) for any split state — the old rigid-driveline
//     behaviour, used as the difference reference;
//  2. cornering: a growing split (outer wheel faster, delta>0) pushes bias
//     toward the faster wheel — driveL > driveR;
//  3. open differential: lockFactor=0 caps the bias at 0 → 50:50 even under a
//     growing split;
//  4. decel parameter set: engine braking (negative axle drive) uses the decel
//     lockFactor, capping the bias far below the accel cap;
//  5. slew limit: the bias torque cannot step more than biasRateLimit*dt per
//     substep, and converges to the preload over subsequent substeps.
int RunDifferentialTest() {
    int failures = 0;
    auto check = [&failures](bool ok, const char* what, float got, float want) {
        std::printf("[%s] diff: %s (got %.2f, target %.2f)\n", ok ? "PASS" : "FAIL", what, got,
                    want);
        if (!ok) ++failures;
    };

    const float r = 0.33f;
    Differential::Input in;
    in.wheelRadius = r;
    in.wheelInertia = 1.5f;  // rear tire inertia, as the orchestrator passes it
    in.fxTorqueDiff = 0.f;   // no tire reaction in the component harness

    {
        DifferentialParams p;
        p.preloadAccel = 0.f;
        p.rampAccel = 0.f;
        p.lockFactorAccel = 1.f;
        Differential d;
        d.Init(p);
        in.dt = 1.f;
        in.axleDrive = 9000.f;
        for (int i = 0; i < 4; ++i) {
            const Differential::Output out = d.Step(in);
            check(out.driveL == 4500.f, "spool baseline: exact 50:50 split",
                  out.driveL, 4500.f);
            check(out.driveR == 4500.f, "spool baseline: exact 50:50 split",
                  out.driveR, 4500.f);
        }
    }

    {
        Differential d;  // defaults: preloadAccel 40, rampAccel 120, lock 0.65
        d.Init(DifferentialParams{});
        in.dt = 1.f;  // one-shot steps: slew (4000 Nm/s) is never binding
        in.axleDrive = 9000.f;
        // Preload spins the split up (no tire reaction here), then the ramp
        // term takes over: the fast (left) wheel must get more drive.
        Differential::Output out;
        for (int i = 0; i < 2; ++i) out = d.Step(in);
        check(out.driveL > out.driveR, "corner: bias toward the faster wheel",
              out.driveL - out.driveR, 1.f);
    }

    {
        DifferentialParams p;
        p.lockFactorAccel = 0.f;  // open diff
        Differential d;
        d.Init(p);
        in.dt = 1.f;
        in.axleDrive = 9000.f;
        Differential::Output out;
        for (int i = 0; i < 4; ++i) out = d.Step(in);
        check(std::fabs(out.driveL - out.driveR) < 1e-3f,
              "open diff: bias capped at zero -> 50:50", out.driveL - out.driveR, 0.f);
    }

    {
        // Decel vs accel parameter sets at the same split: engine braking
        // must bias far below power (decel lockFactor/ramp). Steady state
        // with no tire reaction: the split grows to the clutch slip limit,
        // so bias = preload + ramp * 2*deltaMax per set.
        const float dm = DifferentialParams{}.deltaMax;
        Differential d;  // defaults
        d.Init(DifferentialParams{});
        in.dt = 0.1f;
        in.axleDrive = -9000.f;  // engine braking
        Differential::Output out;
        for (int i = 0; i < 60; ++i) out = d.Step(in);
        const float biasDecel = (out.driveL - out.driveR) * r;
        const float wantDecel = 8.f + 30.f * 2.f * dm;
        check(std::fabs(biasDecel - wantDecel) < 1e-1f, "decel: bias uses the decel ramp",
              biasDecel, wantDecel);
        Differential da;
        da.Init(DifferentialParams{});
        in.axleDrive = 9000.f;  // power
        for (int i = 0; i < 60; ++i) out = da.Step(in);
        const float biasAccel = (out.driveL - out.driveR) * r;
        check(biasAccel > 2.f * biasDecel, "decel: bias far below power bias", biasAccel,
              2.f * biasDecel);
    }

    {
        DifferentialParams p;
        p.rampAccel = 0.f;  // isolate the preload slew from the ramp term
        Differential d;
        d.Init(p);
        in.dt = 1.f / 240.f;
        in.axleDrive = 9000.f;
        const float slewPerStep = 4000.f / 240.f;  // 16.667 Nm
        // driveL - driveR = bias / r, so bias = (driveL - driveR) * r.
        const auto o1 = d.Step(in);
        const float b1 = (o1.driveL - o1.driveR) * r;
        check(std::fabs(b1 - slewPerStep) < 1e-2f, "slew: bias rate limited per substep", b1,
              slewPerStep);
        const auto o2 = d.Step(in);
        const float b2 = (o2.driveL - o2.driveR) * r;
        check(std::fabs(b2 - 2.f * slewPerStep) < 1e-2f, "slew: bias slews toward preload", b2,
              2.f * slewPerStep);
        const auto o3 = d.Step(in);
        const float b3 = (o3.driveL - o3.driveR) * r;
        check(std::fabs(b3 - 40.f) < 1e-2f, "slew: bias converges to preload", b3, 40.f);
    }

    return failures;
}

// TireModel self-aligning moment regression: the caster-trail yaw moment must
// oppose the body slip (restore a turn), vanish without trail, and scale
// linearly with trail. Component-level, backend-agnostic.
int RunTireAlignTest() {
    int failures = 0;
    auto check = [&failures](bool ok, const char* what, float got, float want) {
        std::printf("[%s] align: %s (got %.2f, target %.2f)\n", ok ? "PASS" : "FAIL", what, got,
                    want);
        if (!ok) ++failures;
    };

    const float load = 4000.f;
    const Vec3 fwd{0.f, 0.f, 1.f};
    const Vec3 right{1.f, 0.f, 0.f};
    const float dt = 1.f / 240.f;
    const float trail = 0.035f;

    // Slip to the geometric left (+right axis): the tire pushes back along
    // -right (fyLat < 0), so a positive (restoring, up-yaw) align moment.
    {
        TireModel tire(TireModel::Params::Front());
        const auto out = tire.Compute(load, Vec3{0.8f, 0.f, 12.f}, fwd, right, 0.f, 0.f, dt, 0.f,
                                      false, 0.f, trail);
        check(out.alignMoment > 0.f, "left slip -> restoring positive yaw", out.alignMoment, 0.f);
    }
    {
        TireModel tire(TireModel::Params::Front());
        const auto out = tire.Compute(load, Vec3{-0.8f, 0.f, 12.f}, fwd, right, 0.f, 0.f, dt, 0.f,
                                      false, 0.f, trail);
        check(out.alignMoment < 0.f, "right slip -> negative yaw", out.alignMoment, 0.f);
    }
    {
        TireModel tire(TireModel::Params::Front());
        const auto out = tire.Compute(load, Vec3{0.8f, 0.f, 12.f}, fwd, right, 0.f, 0.f, dt, 0.f,
                                      false, 0.f, 0.f);
        check(out.alignMoment == 0.f, "no trail -> no moment", out.alignMoment, 0.f);
    }
    {
        TireModel a(TireModel::Params::Front());
        const auto o1 = a.Compute(load, Vec3{0.8f, 0.f, 12.f}, fwd, right, 0.f, 0.f, dt, 0.f,
                                  false, 0.f, trail);
        TireModel b(TireModel::Params::Front());
        const auto o2 = b.Compute(load, Vec3{0.8f, 0.f, 12.f}, fwd, right, 0.f, 0.f, dt, 0.f,
                                  false, 0.f, 2.f * trail);
        check(std::fabs(o2.alignMoment - 2.f * o1.alignMoment) < 1e-3f,
              "linear in trail", o2.alignMoment, 2.f * o1.alignMoment);
    }

    return failures;
}

// AeroKit directional-stability regression (component-level, no world): body
// slip must yield a restoring side force and yaw moment, copHeight raises the
// application point, and the terms vanish at standstill (q ~ v^2).
int RunAeroTest() {
    int failures = 0;
    auto check = [&failures](bool ok, const char* what, float got, float want) {
        std::printf("[%s] aero: %s (got %.2f, target %.2f)\n", ok ? "PASS" : "FAIL", what, got,
                    want);
        if (!ok) ++failures;
    };

    CarFrame frame;
    frame.chassis.position = {0.f, 0.5f, 0.f};
    frame.chassis.rotation = Quat{0.f, 0.f, 0.f, 1.f};
    frame.forward = {0.f, 0.f, 1.f};
    frame.right = {1.f, 0.f, 0.f};
    frame.up = {0.f, 1.f, 0.f};

    AeroKit aero;
    aero.Init(AeroKitParams{});

    // Left slip: velocity has a +right (+X) component.
    frame.velocity = {2.f, 0.f, 40.f};
    frame.forwardSpeed = 40.f;
    frame.speed = Length(frame.velocity);
    frame.angularVelocity = {};
    const AeroKit::Output out = aero.Compute(frame, WingMode::ZMode, false);
    check(out.sideForce.x < 0.f, "left slip -> restoring rightward force", out.sideForce.x, 0.f);
    check(out.yawMoment.y < 0.f, "left slip -> restoring (negative) yaw moment",
          out.yawMoment.y, 0.f);
    // COP raised by copHeight above the chassis origin.
    const float copHeight = AeroKitParams{}.copHeight;
    check(std::fabs(out.cop.y - 0.5f - copHeight) < 1e-3f, "cop raised by copHeight",
          out.cop.y, 0.5f + copHeight);

    // Right slip mirrors the signs.
    frame.velocity = {-2.f, 0.f, 40.f};
    const AeroKit::Output out2 = aero.Compute(frame, WingMode::ZMode, false);
    check(out2.sideForce.x > 0.f, "right slip -> restoring leftward force", out2.sideForce.x,
          0.f);
    check(out2.yawMoment.y > 0.f, "right slip -> restoring (positive) yaw moment",
          out2.yawMoment.y, 0.f);

    // Standstill: q = 0, the stability terms vanish exactly.
    frame.velocity = {};
    frame.forwardSpeed = 0.f;
    frame.speed = 0.f;
    const AeroKit::Output out3 = aero.Compute(frame, WingMode::ZMode, false);
    check(out3.sideForce.x == 0.f && out3.sideForce.y == 0.f && out3.sideForce.z == 0.f &&
              out3.yawMoment.x == 0.f && out3.yawMoment.y == 0.f && out3.yawMoment.z == 0.f,
          "standstill -> no aero terms", Length(out3.sideForce), 0.f);

    return failures;
}

// Menu/settings flow on the headless backend: boots into the menu (no
// autostart), opens settings, comes back, starts a race — plus a render
// smoke pass over the menu screens. Test mode is active (Config::SetTestMode),
// so no config.rushconf is read or written: the settings/race-setup Save
// calls inside Game become no-ops and can never leak overrides into the
// user's file. Values not pinned here fall back to code defaults.
int RunMenuTest() {
    NullBackend backend;
    ScriptedInput testInput;
    if (!backend.Init(1280, 720, "menu-test")) {
        std::printf("[FAIL] menu-test backend init\n");
        return 1;
    }

    Config config;
    config.Load(DataPath("config.rushconf"));
    Game game(backend, testInput.system, config);
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
        testInput.set(in);
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
    // change a value and Save() it into the real config.rushconf — the row count
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
    press.menuSelect = true;  // START RACE -> pre-race setup
    frame(press);
    checkPhase(game.Phase() == RacePhase::RaceSetup, "menu: start race opens race setup",
               game.Phase(), RacePhase::RaceSetup);

    press = InputState{};
    press.menuDown = true;
    for (int i = 0; i < 4; ++i) frame(press);  // cursor -> START RACE row
    press = InputState{};
    press.menuSelect = true;
    frame(press);
    checkPhase(game.Phase() == RacePhase::Countdown, "setup: start -> countdown", game.Phase(),
               RacePhase::Countdown);

    // ESC from a race phase must return to the menu without quitting — the
    // same keypress must not also act as "back" in the menu (regression:
    // "ESC exits the app instead of going back to the menu").
    InputState esc;
    esc.pause = true;
    esc.menuBack = true;
    frame(esc);
    checkPhase(game.Phase() == RacePhase::Menu, "esc: race back to menu", game.Phase(),
               RacePhase::Menu);
    if (game.WantsQuit()) {
        std::printf("[FAIL] esc: race-to-menu must not quit\n");
        ++failures;
    }

    // ESC is inert in the menu; only the QUIT item may exit.
    frame(esc);
    checkPhase(game.Phase() == RacePhase::Menu, "esc: menu stays put", game.Phase(),
               RacePhase::Menu);
    if (game.WantsQuit()) {
        std::printf("[FAIL] esc: menu must not quit on back\n");
        ++failures;
    }

    return failures;
}

// Walk the menu through the REAL recognizer path (NullInputSource feeds raw
// key states, InputSystem::PumpOnce runs the gesture machines, Game::Consume
// folds events). This is what a real keyboard does — unlike RunMenuTest which
// injects Click events directly. Verifies direction mapping isn't broken by
// the recognizer/context layer.
int RunMenuRealKeysTest() {
    NullBackend backend;
    ScriptedInput testInput;
    if (!backend.Init(1280, 720, "menu-realkeys")) {
        std::printf("[FAIL] menu-realkeys backend init\n");
        return 1;
    }
    Config config;
    config.Load(DataPath("config.rushconf"));
    Game game(backend, testInput.system, config);
    if (!game.Init()) {
        std::printf("[FAIL] menu-realkeys game init\n");
        return 1;
    }
    int failures = 0;
    auto checkPhase = [&failures](bool ok, const char* what, RacePhase got, RacePhase want) {
        std::printf("[%s] realkey: %s (got %d, want %d)\n", ok ? "PASS" : "FAIL", what,
                    static_cast<int>(got), static_cast<int>(want));
        if (!ok) ++failures;
    };
    // Tap a raw key: down for one frame, up next — a real quick keypress.
    auto tap = [&](KeyCode k) {
        testInput.source.scripted = RawFrame{};
        testInput.source.scripted.keys[static_cast<size_t>(k)] = true;
        backend.BeginFrame();
        game.Update();
        backend.EndFrame();
        testInput.source.scripted = RawFrame{};
        backend.BeginFrame();
        game.Update();
        backend.EndFrame();
    };

    // Boot into menu.
    checkPhase(game.Phase() == RacePhase::Menu, "boot into menu", game.Phase(), RacePhase::Menu);

    // cursor0=START RACE. Enter → RaceSetup.
    tap(KeyCode::Enter);
    checkPhase(game.Phase() == RacePhase::RaceSetup, "Enter at START → RaceSetup",
               game.Phase(), RacePhase::RaceSetup);
    tap(KeyCode::Escape);  // back to menu
    checkPhase(game.Phase() == RacePhase::Menu, "Escape back to menu", game.Phase(),
               RacePhase::Menu);

    // Down once → cursor1=SETTINGS. Enter → Settings.
    tap(KeyCode::Down);
    tap(KeyCode::Enter);
    checkPhase(game.Phase() == RacePhase::Settings, "Down then Enter → Settings",
               game.Phase(), RacePhase::Settings);
    tap(KeyCode::Escape);
    checkPhase(game.Phase() == RacePhase::Menu, "Escape from settings", game.Phase(),
               RacePhase::Menu);

    // Up from menu cursor0 wraps to cursor2=QUIT. Up once from START → QUIT.
    tap(KeyCode::Up);
    tap(KeyCode::Enter);
    checkPhase(game.Phase() == RacePhase::Menu, "Up then Enter → QUIT (phase stays menu)",
               game.Phase(), RacePhase::Menu);
    if (!game.WantsQuit()) {
        std::printf("[FAIL] realkey: Up+Enter at QUIT should want quit\n");
        ++failures;
    }

    backend.Shutdown();
    return failures;
}

// DRS zone semantics: F only arms inside a map-marked zone (zones.csv),
// DRS stays open through speed and steering changes while in the zone, and
// leaving the zone closes it and resets the latch so re-entry starts
// closed. Regression for "DRS opens anywhere" and "re-entry auto-opens".
int RunDrsZoneTest(const char* engine) {
    NullBackend backend;
    ScriptedInput testInput;
    if (!backend.Init(1280, 720, "drs-zone-test")) {
        std::printf("[FAIL] drs-zone-test backend init\n");
        return 1;
    }

    Config config;
    config.Load(DataPath("config.rushconf"));
    config.SetString("race", "cars", "1");  // player only — no AI traffic
    config.SetString("race", "autostart", "true");
    config.SetString("physics", "engine", engine);
    config.SetString("assists", "steer", "full");
    config.SetString("assists", "esc", "full");
    config.SetString("assists", "tc", "sport");
    config.SetString("assists", "abs", "sport");
    // Pin the classic season so a user's [season] config can't disable DRS
    // and fail these zone assertions (local config, never saved).
    config.SetString("season", "drs", "1");
    config.SetString("season", "active_aero", "0");
    config.SetString("season", "overtake", "0");

    Game game(backend, testInput.system, config);
    if (!game.Init()) {
        std::printf("[FAIL] drs-zone-test game init\n");
        return 1;
    }

    int failures = 0;
    auto check = [&failures](bool ok, const char* what, float got, float want) {
        std::printf("[%s] %s (got %.2f, target %.2f)\n", ok ? "PASS" : "FAIL", what, got, want);
        if (!ok) ++failures;
    };

    // Drive with a lookahead steer — the grid sits on the final corner, so
    // a straight run would leave the track before reaching the DRS zone.
    // Same formula as the AI (proven by RunAITest).
    auto frame = [&](const InputState& base) {
        InputState in = base;
        const VehicleState& s = game.PlayerState();
        const Track& track = game.GetTrack();
        const float d = track.ProjectDistance(s.transform.position);
        const Vec3 fwd = TransformDirection(s.transform, Vec3{0.f, 0.f, 1.f});
        const Vec3 right = TransformDirection(s.transform, Vec3{-1.f, 0.f, 0.f});
        const float lookahead = std::min(10.f + s.speed * 0.32f, 30.f);
        const Vec3 target = track.PositionAt(d + lookahead);
        const Vec3 toTarget = target - s.transform.position;
        float yawErr = std::atan2(toTarget.x, toTarget.z) - std::atan2(fwd.x, fwd.z);
        while (yawErr > kPi) yawErr -= 2.f * kPi;
        while (yawErr < -kPi) yawErr += 2.f * kPi;
        const float lateralVel = Dot(s.velocity, right);
        in.steer = Clampf(-yawErr * 1.5f - lateralVel * 0.06f, -1.f, 1.f);
        testInput.set(in);
        backend.BeginFrame();
        game.Update();
        game.RenderHUD();
        backend.EndFrame();
    };

    // Wait out the countdown.
    InputState idle;
    for (int i = 0; i < 240; ++i) frame(idle);

    // 1) Off the grid (outside the zone): F must not arm DRS.
    InputState press;
    press.drsToggle = true;
    frame(press);
    {
        const VehicleState& s = game.PlayerState();
        check(!s.drsAvailable && !s.drsActive, "drs: closed outside zone", s.drsActive, 0.f);
        check(!game.PlayerDrsWanted(), "drs: F ignored outside zone",
              game.PlayerDrsWanted() ? 1.f : 0.f, 0.f);
    }

    // 2) Drive into the zone (main straight, just after the start line).
    InputState gas;
    gas.throttle = 1.f;
    int guard = 0;
    while (!game.PlayerState().drsAvailable && guard++ < 1200) frame(gas);
    check(game.PlayerState().drsAvailable, "drs: zone entered on main straight",
          game.PlayerState().drsAvailable ? 1.f : 0.f, 1.f);
    {
        const VehicleState& s = game.PlayerState();
        check(!s.drsActive && !game.PlayerDrsWanted(), "drs: closed on entry (no auto-open)",
              s.drsActive ? 1.f : 0.f, 0.f);
    }

    // 3) F inside the zone opens DRS — no speed or steering gate.
    press = InputState{};
    press.drsToggle = true;
    frame(press);
    check(game.PlayerState().drsActive, "drs: F opens in zone",
          game.PlayerState().drsActive ? 1.f : 0.f, 1.f);

    // Still in the zone: DRS must stay open (the old implementation closed
    // it below 55 m/s or off-straight).
    for (int i = 0; i < 60; ++i) frame(gas);
    check(game.PlayerState().drsActive, "drs: stays open in zone",
          game.PlayerState().drsActive ? 1.f : 0.f, 1.f);

    // 4) Drive out of the zone: DRS closes and the latch resets, so the
    //    next entry starts closed (regression: re-entry auto-opened).
    guard = 0;
    while (game.PlayerState().drsAvailable && guard++ < 1200) frame(gas);
    check(!game.PlayerState().drsAvailable, "drs: zone exited",
          game.PlayerState().drsAvailable ? 1.f : 0.f, 0.f);
    {
        const VehicleState& s = game.PlayerState();
        check(!s.drsActive, "drs: closes on zone exit", s.drsActive ? 1.f : 0.f, 0.f);
        check(!game.PlayerDrsWanted(), "drs: latch resets on zone exit",
              game.PlayerDrsWanted() ? 1.f : 0.f, 0.f);
    }

    // 5) F outside the zone still does nothing.
    press = InputState{};
    press.drsToggle = true;
    frame(press);
    check(!game.PlayerDrsWanted(), "drs: cannot re-arm outside zone",
          game.PlayerDrsWanted() ? 1.f : 0.f, 0.f);

    backend.Shutdown();
    return failures;
}

// AI autopilot regression: all four cars AI-driven must stay inside the track
// corridor (no wall planting) and make real progress. The player car runs the
// same AI via [race] autopilot.
int RunAITest(const char* engine) {
    NullBackend backend;
    ScriptedInput testInput;
    if (!backend.Init(1280, 720, "ai-test")) {
        std::printf("[FAIL] ai-test backend init\n");
        return 1;
    }

    Config config;
    config.Load(DataPath("config.rushconf"));
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

    Game game(backend, testInput.system, config);
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
        testInput.set(InputState{});
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
    ScriptedInput testInput;
    if (!backend.Init(1280, 720, "ai-diff-test")) {
        std::printf("[FAIL] ai-diff-test backend init\n");
        return 1;
    }

    Config config;
    config.Load(DataPath("config.rushconf"));
    config.SetString("race", "cars", "2");
    config.SetString("race", "autostart", "true");
    config.SetString("physics", "engine", engine);
    config.SetString("assists", "steer", "off");
    config.SetString("assists", "esc", "off");
    config.SetString("assists", "tc", "off");
    config.SetString("assists", "abs", "off");

    Game game(backend, testInput.system, config);
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
    check(playerParams.controls.steerLevel == AssistLevel::Off, "diff: player keeps off assists",
          static_cast<float>(playerParams.controls.steerLevel), 0.f);
    check(aiParams.controls.steerLevel == AssistLevel::Full, "diff: ai steer assist fixed",
          static_cast<float>(aiParams.controls.steerLevel), 2.f);
    check(aiParams.assists.escLevel == AssistLevel::Full, "diff: ai esc assist fixed",
          static_cast<float>(aiParams.assists.escLevel), 2.f);

    const Track& track = game.GetTrack();
    const float d0 = track.ProjectDistance(game.Racers()[1].vehicle.State().transform.position);
    // The player starts one grid slot ahead, so drive it away at full
    // throttle — a parked player car would trip the AI traffic limiter.
    InputState driveAway;
    driveAway.throttle = 1.f;
    for (int i = 0; i < 720; ++i) {
        testInput.set(driveAway);
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

// Contact friction / raycast surface mu: every backend must report the
// create-time Coulomb mu on GetFriction, fill RayHit.friction, and mix a
// pair via GetContactFriction (native or MixFriction fallback).
int RunFrictionRaycastTest(IPhysicsWorld& world) {
    world.SetGravity({0.f, -9.81f, 0.f});

    BodyDesc ground;
    ground.transform.position = {0.f, -1.f, 0.f};
    ground.halfExtents = {2000.f, 1.f, 2000.f};
    ground.friction = 0.75f;
    ground.userData = SurfaceTag(SurfaceType::Road);
    const BodyHandle groundId = world.CreateBody(ground).value_or(kInvalidBody);

    BodyDesc other;
    other.transform.position = {5.f, 0.5f, 0.f};
    other.halfExtents = {1.f, 1.f, 1.f};
    other.friction = 0.3f;
    other.mass = 0.f;
    const BodyHandle otherId = world.CreateBody(other).value_or(kInvalidBody);

    int failures = 0;
    auto check = [&failures](bool ok, const char* what, float got, float want) {
        std::printf("[%s] %s (got %.3f, target %.3f)\n", ok ? "PASS" : "FAIL", what, got, want);
        if (!ok) ++failures;
    };

    const float groundMu = world.GetFriction(groundId).value_or(-1.f);
    const float otherMu = world.GetFriction(otherId).value_or(-1.f);
    check(std::fabs(groundMu - 0.75f) < 1e-4f, "friction: ground mu", groundMu, 0.75f);
    check(std::fabs(otherMu - 0.3f) < 1e-4f, "friction: body mu", otherMu, 0.3f);

    const float mixed = world.GetContactFriction(groundId, otherId).value_or(-1.f);
    const float wantMixed = MixFriction(0.75f, 0.3f);
    check(std::fabs(mixed - wantMixed) < 1e-4f, "friction: contact mix", mixed, wantMixed);

    // Runtime set: native backends write through; nudge updates the query
    // cache only (no NativeSetFriction). Both must change GetFriction.
    (void)world.SetFriction(otherId, 0.55f);
    const float afterSet = world.GetFriction(otherId).value_or(-1.f);
    check(std::fabs(afterSet - 0.55f) < 1e-4f, "friction: set/get", afterSet, 0.55f);

    const RayHit hit = world.Raycast({0.f, 5.f, 0.f}, {0.f, -1.f, 0.f}, 20.f, kInvalidBody);
    check(hit.hit, "raycast: hits ground", hit.hit ? 1.f : 0.f, 1.f);
    check(std::fabs(hit.friction - 0.75f) < 1e-4f, "raycast: surface friction", hit.friction,
          0.75f);
    const float mixAfter = MixFriction(world.GetFriction(groundId).value_or(-1.f),
                                       world.GetFriction(otherId).value_or(-1.f));
    const float mixWant = MixFriction(0.75f, world.GetFriction(otherId).value_or(-1.f));
    check(std::fabs(mixAfter - mixWant) < 1e-4f, "friction: mix tracks set",
          world.GetFriction(otherId).value_or(-1.f), 0.55f);

    return failures;
}

// Wall slam: a high-speed head-on impact must destroy bodywork. Regression
// for two bugs at once — "cornering blows half the car off" (impact speed
// must be the normal closing speed, not the driving speed) and "wall rams
// deal no damage" (damage must scale with crash energy and the normal sign
// must be right).
int RunWallCrashTest(IPhysicsWorld& world) {
    world.SetGravity({0.f, -9.81f, 0.f});

    BodyDesc ground;
    ground.transform.position = {0.f, -1.f, 0.f};
    ground.halfExtents = {2000.f, 1.f, 2000.f};
    ground.friction = 0.8f;
    (void)world.CreateBody(ground);

    BodyDesc wall;
    wall.transform.position = {0.f, 1.f, 30.f};
    wall.halfExtents = {40.f, 2.f, 1.f};
    wall.friction = 0.4f;
    wall.restitution = 0.1f;
    wall.userData = SurfaceTag(SurfaceType::Wall);
    (void)world.CreateBody(wall);

    VehicleParams params;
    Transform pose;
    pose.position = {0.f, 0.55f, 0.f};
    Vehicle car;
    car.Spawn(world, pose, params);
    (void)world.SetLinearVelocity(car.Chassis(), {0.f, 0.f, 50.f});  // 180 km/h

    const float dt = 1.f / 240.f;
    int contactEvents = 0;
    float maxImpact = 0.f;
    for (int i = 0; i < 480; ++i) {  // 2 s: reach the wall, crash, settle
        CarInput in;
        car.Update(in, dt);
        world.Step(dt);
        const std::vector<ContactEvent> contacts = world.DrainContacts();
        for (const ContactEvent& e : contacts) {
            ++contactEvents;
            maxImpact = std::max(maxImpact, e.impactSpeed);
        }
        car.ProcessContacts(contacts, dt);
    }

    float worst = 1.f;
    for (int i = 0; i < 4; ++i) {
        worst = std::min(worst, car.PartHealth(static_cast<CarPart>(i)));
    }
    std::printf("       wall slam: worst part health %.2f, %d contact events, max impact %.1f m/s\n",
                worst, contactEvents, maxImpact);
    const bool ok = worst < 0.5f;
    std::printf("[%s] wall slam at 180 km/h destroys bodywork (got %.2f, target < 0.50)\n",
                ok ? "PASS" : "FAIL", worst);
    return ok ? 0 : 1;
}

// T4: per-part collision boxes. The backend must tag the contacting shape
// with its CarPart value (qu3e/rpc composite bodies); nudge runs its legacy
// single box and reports -1, which the vehicle maps via the coordinate
// regions. Also verifies the damage mapping prefers the tag, and that the
// sidepod collateral rule survives the tag path.
int RunPartCollisionTest(IPhysicsWorld& world, const char* engine) {
    int failures = 0;
    auto check = [&failures](bool ok, const char* what, float got, float want) {
        std::printf("[%s] parts: %s (got %.2f, target %.2f)\n", ok ? "PASS" : "FAIL", what, got,
                    want);
        if (!ok) ++failures;
    };

    world.SetGravity({0.f, -9.81f, 0.f});
    BodyDesc ground;
    ground.transform.position = {0.f, -1.f, 0.f};
    ground.halfExtents = {2000.f, 1.f, 2000.f};
    ground.friction = 0.8f;
    const BodyHandle groundB = world.CreateBody(ground).value_or(kInvalidBody);
    const float dt = 1.f / 240.f;

    // --- L1: tag propagation ------------------------------------------------
    {
        BodyDesc carDesc;
        carDesc.transform.position = {0.f, 1.f, 0.f};
        carDesc.mass = 750.f;
        ShapeDesc fw;
        fw.localTransform.position = {0.f, 0.06f, 1.68f};
        fw.halfExtents = {0.72f, 0.05f, 0.18f};
        fw.partTag = static_cast<int>(CarPart::FrontWing);
        carDesc.shapes.push_back(fw);
        ShapeDesc main;
        main.localTransform.position = {0.f, 0.15f, 0.f};
        main.halfExtents = {0.58f, 0.18f, 1.35f};
        carDesc.shapes.push_back(main);
        const BodyHandle carB = world.CreateBody(carDesc).value_or(kInvalidBody);

        // Wall A overlaps the front-wing box (z 1.50..1.86 for qu3e/rpc).
        BodyDesc wallA;
        wallA.transform.position = {0.f, 1.06f, 1.87f};
        wallA.halfExtents = {4.f, 2.f, 0.15f};
        wallA.userData = SurfaceTag(SurfaceType::Wall);
        (void)world.CreateBody(wallA);
        // Wall B overlaps the untagged main box / nudge's single box.
        BodyDesc wallB;
        wallB.transform.position = {0.f, 1.f, 0.6f};
        wallB.halfExtents = {4.f, 2.f, 0.15f};
        wallB.userData = SurfaceTag(SurfaceType::Wall);
        (void)world.CreateBody(wallB);

        world.Step(dt);
        const std::vector<ContactEvent> contacts = world.DrainContacts();
        int wingTags = 0, untagged = 0;
        for (const ContactEvent& e : contacts) {
            if (e.bodyA != carB && e.bodyB != carB) continue;
            const int part = (e.bodyA == carB) ? e.partA : e.partB;
            if (part == static_cast<int>(CarPart::FrontWing)) ++wingTags;
            if (part == -1) ++untagged;
        }
        if (std::strcmp(engine, "nudge") == 0) {
            check(wingTags == 0, "nudge reports no part tags", static_cast<float>(wingTags), 0.f);
            check(untagged > 0, "nudge single-box contact is untagged (-1)",
                  static_cast<float>(untagged), 1.f);
        } else {
            check(wingTags > 0, "front-wing box tagged in contact",
                  static_cast<float>(wingTags), 1.f);
            check(untagged > 0, "untagged main box reports -1", static_cast<float>(untagged),
                  1.f);
        }
        (void)world.DestroyBody(carB);
    }

    // --- L2: damage mapping prefers the tag --------------------------------
    const auto spawnCar = [&]() {
        Vehicle car;
        Transform pose;
        pose.position = {0.f, 0.55f, 0.f};
        car.Spawn(world, pose, VehicleParams{});
        return car;
    };
    {
        Vehicle car = spawnCar();
        ContactEvent ev;
        ev.bodyA = car.Chassis();
        ev.bodyB = groundB;
        ev.worldPoint = car.State().transform.position + Vec3{0.f, 0.06f, 1.68f};
        ev.worldNormal = {0.f, 0.f, -1.f};
        ev.impactSpeed = 25.f;
        ev.partA = static_cast<int>(CarPart::FrontWing);
        ev.partB = -1;
        car.ProcessContacts({ev}, dt);
        const float fw = car.PartHealth(CarPart::FrontWing);
        check(fw < 1.f, "tagged front-wing contact damages the wing", fw, 1.f);
        check(car.PartHealth(CarPart::RearWing) == 1.f, "other parts untouched",
              car.PartHealth(CarPart::RearWing), 1.f);
        car.Despawn();
    }
    {
        Vehicle car = spawnCar();
        ContactEvent ev;
        ev.bodyA = car.Chassis();
        ev.bodyB = groundB;
        ev.worldPoint = car.State().transform.position + Vec3{0.f, 0.06f, 1.68f};
        ev.worldNormal = {0.f, 0.f, -1.f};
        ev.impactSpeed = 25.f;
        ev.partA = -1;
        ev.partB = -1;
        car.ProcessContacts({ev}, dt);
        const float fw = car.PartHealth(CarPart::FrontWing);
        check(fw < 1.f, "untagged contact falls back to coordinate region", fw, 1.f);
        car.Despawn();
    }
    {
        Vehicle car = spawnCar();
        ContactEvent ev;
        ev.bodyA = car.Chassis();
        ev.bodyB = groundB;
        ev.worldPoint = car.State().transform.position;
        ev.worldNormal = {0.f, 0.f, -1.f};
        ev.impactSpeed = 25.f;
        ev.partA = static_cast<int>(CarPart::Sidepods);
        ev.partB = -1;
        car.ProcessContacts({ev}, dt);
        check(car.PartHealth(CarPart::Floor) < 1.f,
              "sidepod hit clips the floor edge (collateral rule)",
              car.PartHealth(CarPart::Floor), 1.f);
        car.Despawn();
    }

    return failures;
}

// Season feature gates: classic (DRS) and 2026 (active aero + override)
// must not leak into each other — a disabled system ignores its input even
// if the game layer forwards it. Vehicle-level check of the VehicleParams
// flags the game sets from the [season] config at spawn.
int RunSeasonGateTest(IPhysicsWorld& world) {
    world.SetGravity({0.f, -9.81f, 0.f});

    BodyDesc ground;
    ground.transform.position = {0.f, -1.f, 0.f};
    ground.halfExtents = {2000.f, 1.f, 2000.f};
    ground.friction = 0.8f;
    (void)world.CreateBody(ground);

    int failures = 0;
    auto check = [&failures](bool ok, const char* what, float got, float want) {
        std::printf("[%s] %s (got %.2f, target %.2f)\n", ok ? "PASS" : "FAIL", what, got, want);
        if (!ok) ++failures;
    };

    const float dt = 1.f / 240.f;
    auto spawn = [&](const VehicleParams& params) {
        Vehicle car;
        Transform pose;
        pose.position = {0.f, 0.55f, 0.f};
        car.Spawn(world, pose, params);
        for (int i = 0; i < 60; ++i) {  // settle on the suspension
            car.Update(CarInput{}, dt);
            world.Step(dt);
        }
        return car;
    };
    auto step = [&](Vehicle& car, const CarInput& in, float seconds) {
        const int n = static_cast<int>(seconds / dt + 0.5f);
        for (int i = 0; i < n; ++i) {
            car.Update(in, dt);
            world.Step(dt);
        }
    };

    // Classic season: DRS works, the 2026 systems stay dead.
    {
        VehicleParams params;  // defaults: drs on, aero/override off
        Vehicle car = spawn(params);

        CarInput drs;
        drs.drsZone = true;
        drs.drsRequest = true;
        step(car, drs, 0.05f);
        check(car.State().drsActive, "season: classic DRS works in zone",
              car.State().drsActive ? 1.f : 0.f, 1.f);

        CarInput wing;
        wing.wingToggle = true;
        step(car, wing, 0.05f);
        check(car.State().wingMode == WingMode::ZMode, "season: classic ignores wing toggle",
              static_cast<float>(car.State().wingMode), static_cast<float>(WingMode::ZMode));

        CarInput ovt;
        ovt.overtake = true;
        ovt.throttle = 1.f;
        step(car, ovt, 0.3f);
        check(!car.State().ersDeploying, "season: classic ignores override",
              car.State().ersDeploying ? 1.f : 0.f, 0.f);
    }

    // 2026 season: active aero + override work, DRS stays dead even inside
    // a DRS zone with a stale request.
    {
        VehicleParams params;
        params.drsEnabled = false;
        params.activeAeroEnabled = true;
        params.overtakeEnabled = true;
        Vehicle car = spawn(params);

        CarInput drs;
        drs.drsZone = true;
        drs.drsRequest = true;
        step(car, drs, 0.05f);
        check(!car.State().drsActive, "season: 2026 ignores DRS",
              car.State().drsActive ? 1.f : 0.f, 0.f);

        CarInput wing;
        wing.wingToggle = true;
        step(car, wing, 0.05f);
        check(car.State().wingMode == WingMode::XMode, "season: 2026 wing toggle works",
              static_cast<float>(car.State().wingMode), static_cast<float>(WingMode::XMode));

        CarInput ovt;
        ovt.overtake = true;
        ovt.throttle = 1.f;
        step(car, ovt, 0.3f);
        check(car.State().ersDeploying, "season: 2026 override deploys",
              car.State().ersDeploying ? 1.f : 0.f, 1.f);
    }

    return failures;
}

// `apex-rush --drive-test`: headless physics/controls verification. Drives the
// player car with scripted input and asserts acceleration, braking, steering
// and reverse. Exits 0 on success, 1 on failure. No window is created, so it
// runs on locked sessions and in CI. The full suite runs once per physics
// backend (qu3e, nudge and rush-physics-core must all pass it).
int RunDriveTestForEngine(Config& config, const char* engine) {
    config.SetString("physics", "engine", engine);
    // Pin the simulation environment: user config (assists / hz / season)
    // must not change scripted-input assertions between machines or runs.
    config.SetString("physics", "hz", "240");
    config.SetString("assists", "steer", "full");
    config.SetString("assists", "esc", "full");
    config.SetString("assists", "tc", "sport");
    config.SetString("assists", "abs", "sport");
    config.SetString("season", "drs", "1");
    config.SetString("season", "active_aero", "0");
    config.SetString("season", "overtake", "0");
    NullBackend backend;
    ScriptedInput testInput;
    if (!backend.Init(1280, 720, "drive-test")) {
        std::printf("[FAIL] NullBackend init\n");
        return 1;
    }

    // Solo run: AI cars would ram the stationary test car and skew the numbers.
    config.SetString("race", "cars", "1");

    Game game(backend, testInput.system, config);
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
        testInput.set(input);
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

    // 5b) Wall slam must hurt (damage calibration / normal sign).
    {
        std::unique_ptr<IPhysicsWorld> crashWorld = MakeWorld(engine);
        failures += RunWallCrashTest(*crashWorld);
    }

    // 5b2) Per-part collision boxes: tags, fallback and damage mapping.
    {
        std::unique_ptr<IPhysicsWorld> partsWorld = MakeWorld(engine);
        failures += RunPartCollisionTest(*partsWorld, engine);
    }

    // 5c) Season gates: classic DRS and 2026 aero/override never leak.
    {
        std::unique_ptr<IPhysicsWorld> seasonWorld = MakeWorld(engine);
        failures += RunSeasonGateTest(*seasonWorld);
    }

    // 5d) Contact friction + raycast surface mu (native / compat paths).
    {
        std::unique_ptr<IPhysicsWorld> frictionWorld = MakeWorld(engine);
        failures += RunFrictionRaycastTest(*frictionWorld);
    }

    // 6) Menu / settings flow (headless, backend-agnostic).
    failures += RunMenuTest();
    // 6a) Same flow through the real recognizer path (raw key feed).
    failures += RunMenuRealKeysTest();

    // 6b) DRS zone semantics: map-marked zones, latch reset on exit.
    failures += RunDrsZoneTest(engine);

    // 7) AI autopilot: no wall planting, real progress.
    failures += RunAITest(engine);

    // 8) AI cars ignore player difficulty assists (never spin at assists-off).
    failures += RunAIDifficultyIsolationTest(engine);

    std::printf("drive-test[%s]: %s (%d failures)\n", engine,
                failures == 0 ? "OK" : "FAILED", failures);
    backend.Shutdown();
    return failures;
}

// Threaded-sim smoke test: the same headless harness, but with the race
// stages on the sim worker — exercises the intent mailbox, the triple-buffer
// snapshot handoff and the snapshot HUD path that the inline drive-test can
// never reach. Wall-clock paced (the worker ticks at real 60 Hz), so guards
// are timeouts rather than frame counts.
int RunThreadedSmokeTest(Config& config) {
    config.SetString("physics", "engine", "qu3e");
    config.SetString("physics", "hz", "240");
    config.SetString("assists", "steer", "full");
    config.SetString("assists", "esc", "full");
    config.SetString("assists", "tc", "sport");
    config.SetString("assists", "abs", "sport");
    config.SetString("season", "drs", "1");
    config.SetString("season", "active_aero", "0");
    config.SetString("season", "overtake", "0");
    config.SetString("race", "cars", "2");
    config.SetString("race", "autopilot", "false");

    NullBackend backend;
    ScriptedInput testInput;
    if (!backend.Init(1280, 720, "threaded-smoke")) {
        std::printf("[FAIL] NullBackend init\n");
        return 1;
    }

    int failures = 0;
    auto check = [&failures](bool ok, const char* what, float got, float want) {
        std::printf("[%s] %s (got %.2f, target %.2f)\n", ok ? "PASS" : "FAIL", what, got, want);
        if (!ok) ++failures;
    };

    {
        Game game(backend, testInput.system, config);
        game.SetThreadedSim(true);
        if (!game.Init()) {
            std::printf("[FAIL] Game init\n");
            backend.Shutdown();
            return 1;
        }

        using clock = std::chrono::steady_clock;
        // Pump frames for `seconds` of wall time; RenderHUD every frame so
        // the snapshot-backed HUD path is exercised throughout.
        auto runWall = [&](double seconds, InputState input) {
            testInput.set(input);
            const auto t0 = clock::now();
            while (std::chrono::duration<double>(clock::now() - t0).count() < seconds) {
                backend.BeginFrame();
                game.Update();
                game.RenderHUD();
                backend.EndFrame();
            }
        };
        // Pump until pred() or the wall timeout — the worker owns sim time.
        auto waitWall = [&](double timeoutSec, InputState input, auto pred) {
            testInput.set(input);
            const auto t0 = clock::now();
            while (std::chrono::duration<double>(clock::now() - t0).count() < timeoutSec) {
                backend.BeginFrame();
                game.Update();
                game.RenderHUD();
                backend.EndFrame();
                if (pred()) return true;
            }
            return pred();
        };

        // 1) Countdown → Racing on the worker (3.5 s sim + slack).
        const bool reachedRacing =
            waitWall(10.0, InputState{}, [&] { return game.Phase() == RacePhase::Racing; });
        check(reachedRacing, "threaded: countdown -> racing", game.Phase() == RacePhase::Racing ? 1.f : 0.f, 1.f);

        // 2) Snapshot handshake: valid, both cars, player is index 0.
        {
            const auto snap = game.FrameSnapshot();
            const bool ok = snap && snap->valid && snap->cars.size() == 2 && snap->cars[0].isPlayer;
            check(ok, "threaded: snapshot valid with 2 cars", snap && snap->valid ? 1.f : 0.f, 1.f);
        }

        // 3) Throttle through the mailbox accelerates the car (observed via
        //    the snapshot — live racers_ belongs to the worker here).
        const auto snap0 = game.FrameSnapshot();
        const float v0 = snap0 ? snap0->cars[0].state.speed : 0.f;
        InputState gas;
        gas.throttle = 1.f;
        runWall(2.0, gas);
        const auto snap1 = game.FrameSnapshot();
        const float v1 = snap1 ? snap1->cars[0].state.speed : 0.f;
        check(v1 > v0 + 5.f, "threaded: throttle accelerates via mailbox", v1 - v0, 5.f);

        // 4) One-frame ENTER edge survives the mailbox: restart lands on the
        //    sim thread and phase drops back to Countdown, then re-arms.
        InputState enter;
        enter.restart = true;
        runWall(0.05, enter);  // one-ish frames with the edge set
        const bool gotRestart =
            waitWall(5.0, InputState{}, [&] { return game.Phase() == RacePhase::Countdown; });
        check(gotRestart, "threaded: restart edge reaches sim", gotRestart ? 1.f : 0.f, 1.f);
        const bool reRaced =
            waitWall(10.0, InputState{}, [&] { return game.Phase() == RacePhase::Racing; });
        check(reRaced, "threaded: countdown -> racing after restart", reRaced ? 1.f : 0.f, 1.f);

        // 5) ESC: pause is written on the main thread; the worker parks and
        //    the menu stays up (no quit, no crash while joining).
        InputState esc;
        esc.pause = true;
        runWall(0.05, esc);
        check(game.Phase() == RacePhase::Menu, "threaded: ESC returns to menu",
              game.Phase() == RacePhase::Menu ? 1.f : 0.f, 1.f);
        runWall(0.5, InputState{});
        check(game.Phase() == RacePhase::Menu && !game.WantsQuit(),
              "threaded: menu stays after ESC", game.Phase() == RacePhase::Menu ? 1.f : 0.f, 1.f);

        // 6) Menu → Race Setup → START on the async build path: the menu
        //    thread must swap in the background-built world and arm the
        //    countdown without ever blocking a frame on map IO.
        auto tap = [&](InputState in) {
            testInput.set(in);
            backend.BeginFrame();
            game.Update();
            game.RenderHUD();
            backend.EndFrame();
            testInput.set(InputState{});
        };
        InputState select;
        select.menuSelect = true;
        tap(select);  // main menu: START RACE → Race Setup
        check(game.Phase() == RacePhase::RaceSetup, "threaded: menu opens race setup",
              game.Phase() == RacePhase::RaceSetup ? 1.f : 0.f, 1.f);
        InputState down;
        down.menuDown = true;
        for (int i = 0; i < 4; ++i) tap(down);  // cursor → START RACE row
        tap(select);                            // BeginRaceLoad
        check(game.Phase() == RacePhase::RaceSetup, "threaded: build starts async",
              game.Phase() == RacePhase::RaceSetup ? 1.f : 0.f, 1.f);
        const bool loadedToCountdown =
            waitWall(15.0, InputState{}, [&] { return game.Phase() == RacePhase::Countdown; });
        check(loadedToCountdown, "threaded: async load arms countdown",
              loadedToCountdown ? 1.f : 0.f, 1.f);
        const bool rereachedRacing =
            waitWall(10.0, InputState{}, [&] { return game.Phase() == RacePhase::Racing; });
        check(rereachedRacing, "threaded: racing after async load", rereachedRacing ? 1.f : 0.f, 1.f);
        {
            const auto snap = game.FrameSnapshot();
            const bool ok = snap && snap->valid && snap->cars.size() == 2;
            check(ok, "threaded: snapshot valid after async load",
                  snap && snap->valid ? 1.f : 0.f, 1.f);
        }

        // Game destructor joins the worker before backend.Shutdown().
    }
    backend.Shutdown();
    std::printf("threaded-smoke: %s (%d failures)\n", failures == 0 ? "OK" : "FAILED", failures);
    return failures;
}

// InputSystem gesture recognizer unit tests: exercises Hold begin/level/end,
// PressRelease down/up, Click, DoubleClick, LongPress, multi-key OR, and the
// menu/race context gate — all through the pure InputBus (no callbacks).
int RunInputTest() {
    NullInputSource source;
    InputSystem input(source);
    InputRegistry& reg = input.Registry();
    int failures = 0;
    auto check = [&failures](bool ok, const char* what, float got, float want) {
        std::printf("[%s] input: %s (got %.2f, target %.2f)\n", ok ? "PASS" : "FAIL", what, got, want);
        if (!ok) ++failures;
    };
    auto key = [](KeyCode k) {
        DeviceKey d;
        d.kind = DeviceKind::Keyboard;
        d.code = static_cast<uint16_t>(k);
        return d;
    };
    std::vector<InputEvent> events;
    auto drain = [&]() {
        events.clear();
        InputBus().Drain(events);
    };
    auto names = [&]() {
        std::vector<std::string> out;
        for (const InputEvent& e : events) out.push_back(std::string(InputNameOf(e.nameId)));
        return out;
    };
    auto has = [&](const char* n) {
        for (const InputEvent& e : events)
            if (InputNameOf(e.nameId) == n) return true;
        return false;
    };

    // --- Hold: throttle (W + Up OR) ------------------------------------------
    reg.Register({.name = "throttle", .gesture = Gesture::Hold,
                  .keys = {{.key = key(KeyCode::W)}, {.key = key(KeyCode::Up)}},
                  .context = "race"});
    reg.Register({.name = "handbrake", .gesture = Gesture::Hold,
                  .keys = {{.key = key(KeyCode::Space)}}, .context = "race"});
    reg.Register({.name = "restart", .gesture = Gesture::PressRelease,
                  .keys = {{.key = key(KeyCode::Enter)}}});
    reg.Register({.name = "drs", .gesture = Gesture::Click,
                  .keys = {{.key = key(KeyCode::F)}}, .context = "race"});
    reg.Register({.name = "select", .gesture = Gesture::DoubleClick,
                  .keys = {{.key = key(KeyCode::G)}}});
    reg.Register({.name = "hold", .gesture = Gesture::LongPress,
                  .keys = {{.key = key(KeyCode::H)}}});

    input.SetActiveContext("race");

    // Hold begin + level on first down.
    source.scripted = RawFrame{};
    source.scripted.keys[static_cast<size_t>(KeyCode::W)] = true;
    input.PumpOnce();
    drain();
    check(has("throttle.begin"), "hold: begin fires", has("throttle.begin") ? 1.f : 0.f, 1.f);
    check(has("throttle.level"), "hold: level fires on down", has("throttle.level") ? 1.f : 0.f, 1.f);
    check(input.ActionValue("throttle") > 0.9f, "hold: action value while held",
          input.ActionValue("throttle"), 1.f);

    // Up key also holds (multi-key OR).
    source.scripted = RawFrame{};
    source.scripted.keys[static_cast<size_t>(KeyCode::Up)] = true;
    input.PumpOnce();
    drain();
    check(input.ActionValue("throttle") > 0.9f, "hold: OR key holds", input.ActionValue("throttle"), 1.f);

    // Release ends the hold.
    source.scripted = RawFrame{};
    input.PumpOnce();
    drain();
    check(has("throttle.end"), "hold: end fires on release", has("throttle.end") ? 1.f : 0.f, 1.f);
    check(input.ActionValue("throttle") < 0.1f, "hold: value clears on release",
          input.ActionValue("throttle"), 0.f);

    // --- PressRelease: down/up two events ------------------------------------
    source.scripted = RawFrame{};
    source.scripted.keys[static_cast<size_t>(KeyCode::Enter)] = true;
    input.PumpOnce();
    drain();
    check(has("restart.down"), "pressrelease: down fires", has("restart.down") ? 1.f : 0.f, 1.f);
    check(!has("restart.up"), "pressrelease: no up while held", has("restart.up") ? 0.f : 1.f, 1.f);
    source.scripted = RawFrame{};
    input.PumpOnce();
    drain();
    check(has("restart.up"), "pressrelease: up fires on release", has("restart.up") ? 1.f : 0.f, 1.f);

    // --- Click on press edge + context gate ----------------------------------
    source.scripted = RawFrame{};
    source.scripted.keys[static_cast<size_t>(KeyCode::F)] = true;
    input.PumpOnce();
    drain();
    check(has("drs.click"), "click: fires on press edge", has("drs.click") ? 1.f : 0.f, 1.f);
    // Same key, repeated press, emits again.
    source.scripted = RawFrame{};
    input.PumpOnce();
    drain();
    source.scripted.keys[static_cast<size_t>(KeyCode::F)] = true;
    input.PumpOnce();
    drain();
    check(has("drs.click"), "click: re-press fires again", has("drs.click") ? 1.f : 0.f, 1.f);

    // Context gate: switch to menu, F (race-gated) must not fire.
    input.SetActiveContext("menu");
    source.scripted = RawFrame{};
    source.scripted.keys[static_cast<size_t>(KeyCode::F)] = true;
    input.PumpOnce();
    drain();
    check(!has("drs.click"), "context: race-gated click frozen in menu",
          has("drs.click") ? 0.f : 1.f, 1.f);

    // --- DoubleClick: two presses inside the window --------------------------
    input.SetActiveContext("race");
    // Press 1: a click, sets lastRelease.
    source.scripted = RawFrame{};
    source.scripted.keys[static_cast<size_t>(KeyCode::G)] = true;
    input.PumpOnce();
    drain();
    source.scripted = RawFrame{};
    input.PumpOnce();
    drain();
    // Press 2 within the window: dblclick, no extra click.
    source.scripted.keys[static_cast<size_t>(KeyCode::G)] = true;
    input.PumpOnce();
    drain();
    check(has("select.dblclick"), "dblclick: second press in window fires",
          has("select.dblclick") ? 1.f : 0.f, 1.f);
    check(!has("select.click"), "dblclick: second press suppresses extra click",
          has("select.click") ? 0.f : 1.f, 1.f);
    source.scripted = RawFrame{};
    input.PumpOnce();
    drain();

    // --- LongPress: held past threshold fires once ---------------------------
    input.SetActiveContext("race");
    source.scripted = RawFrame{};
    source.scripted.keys[static_cast<size_t>(KeyCode::H)] = true;
    input.PumpOnce();  // pressStart
    drain();
    // Advance the clock beyond longPressMs (500ms) and hold.
    std::this_thread::sleep_for(std::chrono::milliseconds(550));
    source.scripted.keys[static_cast<size_t>(KeyCode::H)] = true;
    input.PumpOnce();
    drain();
    check(has("hold.longpress"), "longpress: fires past threshold",
          has("hold.longpress") ? 1.f : 0.f, 1.f);
    source.scripted = RawFrame{};
    input.PumpOnce();
    drain();

    // --- Axis binding (analog) ----------------------------------------------
    reg.Register({.name = "steer", .gesture = Gesture::Hold,
                  .keys = {{.axisIndex = 2, .axisScale = 1.f}}, .context = "race"});
    source.scripted = RawFrame{};
    source.scripted.axes[2] = 0.35f;
    input.PumpOnce();
    drain();
    check(std::fabs(input.ActionValue("steer") - 0.35f) < 1e-3f,
          "axis: analog value flows to action", input.ActionValue("steer"), 0.35f);
    source.scripted = RawFrame{};
    input.PumpOnce();
    drain();
    check(input.ActionValue("steer") < 1e-3f, "axis: clears on release",
          input.ActionValue("steer"), 0.f);

    // --- Config rebind (ApplyOverrides) -------------------------------------
    Config cfg;
    cfg.SetString("input", "throttle.keys", "lshift,space");
    auto ov = reg.ApplyOverrides(cfg, "input");
    check(ov.has_value(), "overrides: valid rebind ok", ov.has_value() ? 1.f : 0.f, 1.f);
    source.scripted = RawFrame{};
    source.scripted.keys[static_cast<size_t>(KeyCode::LShift)] = true;
    input.PumpOnce();
    drain();
    check(input.ActionValue("throttle") > 0.9f, "overrides: rebound key drives action",
          input.ActionValue("throttle"), 1.f);
    source.scripted = RawFrame{};
    input.PumpOnce();
    drain();
    cfg.SetString("input", "throttle.keys", "???");
    ov = reg.ApplyOverrides(cfg, "input");
    check(!ov.has_value(), "overrides: unrecognized key returns Err", ov.has_value() ? 0.f : 1.f, 1.f);

    // --- Prefix-collision regression (menu.back vs menu.down) ----------------
    // "menu.back.click" must fold only into menuBack, never menuDown — the
    // shared "menu." head used to let the suffix-only matcher hit menu.down.
    reg.Register({.name = "menu.down", .gesture = Gesture::Click,
                  .keys = {{.key = key(KeyCode::Down)}}, .context = "menu"});
    reg.Register({.name = "menu.back", .gesture = Gesture::Click,
                  .keys = {{.key = key(KeyCode::Escape)}}, .context = "menu"});
    InputEvent backEv;
    backEv.nameId = InternInputName("menu.back.click");
    backEv.gesture = Gesture::Click;
    backEv.value = 1.f;
    backEv.tSec = input.Now();
    InputBus().Publish(backEv);
    InputState folded;
    input.Consume(folded);
    check(folded.menuBack, "prefix: menu.back folds into menuBack",
          folded.menuBack ? 1.f : 0.f, 1.f);
    check(!folded.menuDown, "prefix: menu.back does NOT leak into menuDown",
          folded.menuDown ? 0.f : 1.f, 1.f);

    return failures;
}

int RunDriveTest(Config& config) {
    int failures = 0;
    std::printf("=== input recognizers ===\n");
    failures += RunInputTest();
    std::printf("=== differential component ===\n");
    failures += RunDifferentialTest();
    std::printf("=== tire align moment ===\n");
    failures += RunTireAlignTest();
    std::printf("=== aero stability ===\n");
    failures += RunAeroTest();
    for (const char* engine : {"qu3e", "nudge", "rpc"}) {
        std::printf("=== physics backend: %s ===\n", engine);
        const auto start = std::chrono::steady_clock::now();
        failures += RunDriveTestForEngine(config, engine);
        const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - start;
        std::printf("drive-test[%s] wall time: %.2f s\n", engine, elapsed.count());
    }
    std::printf("=== threaded sim smoke ===\n");
    failures += RunThreadedSmokeTest(config);
    std::printf("drive-test: %s (%d failures)\n", failures == 0 ? "OK" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}

// `apex-rush --bench [cars]`: headless performance snapshot of one physics
// backend. Autopilot grid, fixed assist pins, N seconds of racing after the
// countdown. Prints per-frame section means so the async-engine refactor can
// be compared against this baseline byte-for-byte on the same workload.
int RunBench(Config& config, int cars, const char* engine, double raceSeconds, int hz) {
    config.SetString("physics", "engine", engine);
    config.SetString("race", "cars", std::to_string(cars));
    if (hz > 0) config.SetString("physics", "hz", std::to_string(hz));
    config.SetString("race", "autostart", "true");
    config.SetString("race", "autopilot", "true");
    config.SetString("assists", "steer", "full");
    config.SetString("assists", "esc", "full");
    config.SetString("assists", "tc", "sport");
    config.SetString("assists", "abs", "sport");

    NullBackend backend;
    ScriptedInput testInput;
    if (!backend.Init(1280, 720, "bench")) {
        std::printf("[FAIL] bench backend init\n");
        return 1;
    }

    Game game(backend, testInput.system, config);
    if (!game.Init()) {
        std::printf("[FAIL] bench game init\n");
        return 1;
    }
    game.SetBench(true);

    auto frame = [&]() {
        testInput.set(InputState{});
        backend.BeginFrame();
        game.Update();
        backend.EndFrame();
    };

    // Countdown + settle.
    for (int i = 0; i < 240; ++i) frame();
    game.ResetBench();

    const int frames = static_cast<int>(raceSeconds / backend.frameDt + 0.5);
    const auto wall0 = std::chrono::steady_clock::now();
    for (int i = 0; i < frames; ++i) frame();
    const std::chrono::duration<double> wall = std::chrono::steady_clock::now() - wall0;

    const Game::BenchSample& s = game.Bench();
    const double n = s.frames > 0 ? static_cast<double>(s.frames) : 1.0;
    const double steps = s.physSteps > 0 ? static_cast<double>(s.physSteps) : 1.0;

    // Worst-percentile ("1% low"): the mean of the slowest 1% of samples —
    // the perceived frame rate / physics hitchiness, not just the average.
    const auto worstPct = [](std::vector<double>& v, double pct) -> double {
        if (v.size() < 2) return v.empty() ? 0.0 : v.front();
        std::sort(v.begin(), v.end());
        const size_t k = std::max<size_t>(1, v.size() * pct / 100.0);
        double sum = 0.0;
        for (size_t i = v.size() - k; i < v.size(); ++i) sum += v[i];
        return sum / static_cast<double>(k);
    };
    auto frameTimes = s.frameTimesMs;
    auto stepTimes = s.stepTimesMs;
    const double frame1Low = worstPct(frameTimes, 1.0);
    const double step1Low = worstPct(stepTimes, 1.0);

    std::printf("bench racers=%zu\n", game.Racers().size());
    std::printf(
        "bench[%s] cars=%d frames=%ld physSteps=%ld wall=%.3fs\n"
        "  per-frame ms:  update=%.3f  ai=%.3f  race=%.3f\n"
        "  per-step  ms:  vehicle=%.4f  step=%.4f  contact=%.4f\n"
        "  1%% low:        frame=%.3f ms (%.1f fps)  step=%.4f ms  vehicle=%.4f ms\n"
        "  sim-share:     ai=%.1f%%  vehicle=%.1f%%  step=%.1f%%  contact=%.1f%%  race=%.1f%%\n"
        "  throughput:    %.1f steps/s wall, %.1f frames/s wall\n",
        engine, cars, s.frames, s.physSteps, wall.count(),
        1e3 * s.updateSec / n, 1e3 * s.aiSec / n, 1e3 * s.raceSec / n,
        1e3 * s.vehicleSec / steps, 1e3 * s.stepSec / steps, 1e3 * s.contactSec / steps,
        frame1Low, frame1Low > 0.0 ? 1000.0 / frame1Low : 0.0, step1Low,
        1e3 * s.vehicleSec / steps,
        100.0 * s.aiSec / std::max(s.updateSec, 1e-12),
        100.0 * s.vehicleSec / std::max(s.updateSec, 1e-12),
        100.0 * s.stepSec / std::max(s.updateSec, 1e-12),
        100.0 * s.contactSec / std::max(s.updateSec, 1e-12),
        100.0 * s.raceSec / std::max(s.updateSec, 1e-12),
        steps / wall.count(), n / wall.count());

    backend.Shutdown();

    // Per-stage physics breakdown of the final substep (backend-reported; an
    // all-zero report means the backend does not instrument itself). This is
    // the number that tells you whether to attack broadphase, narrowphase or
    // the solver next.
    IPhysicsWorld::StageStats stage;
    if (game.World()) game.World()->ReadStageStats(stage);
    if (stage.steps > 0 || stage.gridCells > 0) {
        std::printf("  stage ms:      proxy=%.4f  pairs=%.4f  narrow=%.4f  build=%.4f  solve=%.4f  sync=%.4f"
                    "  (peak contacts=%d points=%d islands=%d cells=%d, steps=%ld)\n",
                    stage.proxyMs, stage.pairMs, stage.narrowphaseMs, stage.islandBuildMs,
                    stage.solveMs, stage.syncMs, stage.contacts, stage.contactPoints,
                    stage.islands, stage.gridCells, stage.steps);
        // Closure: the instrumented segments are a partition of Step(), so their
        // excess over stepMs is what the timers miss (clock reads, call
        // overhead); wrapMs is the adapter work outside Step() (force push +
        // contact events) — the part a per-stage report can never show.
        const double segs = stage.broadphaseMs + stage.proxyMs + stage.pairMs + stage.forceMs +
                            stage.narrowphaseMs + stage.islandBuildMs + stage.solveMs;
        std::printf("  closure ms:    step=%.4f  segs=%.4f  unaccounted=%+.4f  wrap=%.4f"
                    "  force=%.4f\n",
                    stage.stepMs, segs, segs - stage.stepMs, stage.wrapMs, stage.forceMs);
        std::printf("  pairs ms:      query=%.4f  sort+queue=%.4f   (peak per step: moved=%d"
                    "  produced=%d  unique=%d)\n",
                    stage.pairQueryMs, stage.sortMs, stage.movedProxies,
                    stage.pairsProduced, stage.pairsUnique);
    }

    // TEMP probe: per-backend raycast cost (remove with the probe).
    const auto rp = apex::RpcRaycastProbe();
    if (rp.calls > 0) {
        std::printf("  raycast[rpc]:  calls=%llu  avg=%.3fus  leaf/ray=%.2f  nodes/ray=%.1f\n",
                    static_cast<unsigned long long>(rp.calls), rp.micros / rp.calls,
                    static_cast<double>(rp.leafTests) / rp.calls,
                    static_cast<double>(rp.nodeVisits) / rp.calls);
    }
    const auto qp = apex::Qu3eRaycastProbe();
    if (qp.calls > 0) {
        std::printf("  raycast[qu3e]: calls=%llu  avg=%.3fus  leaf/ray=%.2f\n",
                    static_cast<unsigned long long>(qp.calls), qp.micros / qp.calls,
                    static_cast<double>(qp.leafTests) / qp.calls);
    }
    return 0;
}

}  // namespace

// `apex-rush --stress [cars] [hz] [seconds]`: collision stress run. Collapses
// the whole grid into one spot at speed (what a mid-pack pile-up does) and
// reports frame cost plus any body that went non-finite or ran away, so
// "several cars collide" stays measurable and regression-testable.
int RunStress(Config& config, int cars, int hz, double seconds, const char* engine) {
    config.SetString("physics", "engine", engine);
    config.SetString("physics", "hz", std::to_string(hz));
    config.SetString("race", "cars", std::to_string(cars));
    config.SetString("race", "autostart", "true");
    config.SetString("race", "autopilot", "true");
    config.SetString("race", "damage", "true");
    config.SetString("assists", "steer", "full");
    config.SetString("assists", "esc", "full");
    config.SetString("assists", "tc", "sport");
    config.SetString("assists", "abs", "sport");

    NullBackend backend;
    ScriptedInput testInput;
    if (!backend.Init(1280, 720, "stress")) {
        std::printf("[FAIL] stress backend init\n");
        return 1;
    }
    Game game(backend, testInput.system, config);
    if (!game.Init()) {
        std::printf("[FAIL] stress game init\n");
        return 1;
    }
    game.SetBench(true);

    auto frame = [&] {
        testInput.set(InputState{});
        backend.BeginFrame();
        game.Update();
        backend.EndFrame();
    };

    // Countdown + settle so every car is on its wheels before the pile forms.
    for (int i = 0; i < 240; ++i) frame();

    game.ResetBench();
    game.StressPileUp(7.f, 40.f);

    int badStates = 0;
    float worstSpeed = 0.f;
    float worstY = 0.f;
    const int frames = static_cast<int>(seconds / backend.frameDt + 0.5);
    const auto wall0 = std::chrono::steady_clock::now();
    for (int i = 0; i < frames; ++i) {
        frame();
        for (const Racer& racer : game.Racers()) {
            const VehicleState& s = racer.vehicle.State();
            const Vec3 p = s.transform.position;
            const bool finite = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
                                std::isfinite(s.velocity.x) && std::isfinite(s.velocity.y) &&
                                std::isfinite(s.velocity.z) && std::isfinite(s.speed);
            if (!finite || s.speed > 400.f) ++badStates;
            if (s.speed > worstSpeed) worstSpeed = s.speed;
            if (std::fabs(p.y) > std::fabs(worstY)) worstY = p.y;
        }
    }
    const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
    const Game::BenchSample& s = game.Bench();
    const double n = s.frames > 0 ? static_cast<double>(s.frames) : 1.0;
    const double steps = s.physSteps > 0 ? static_cast<double>(s.physSteps) : 1.0;
    auto frameTimes = s.frameTimesMs;
    std::sort(frameTimes.begin(), frameTimes.end());
    const auto pct = [&frameTimes](double p) {
        if (frameTimes.empty()) return 0.0;
        const size_t k = std::min(frameTimes.size() - 1,
                                  static_cast<size_t>(frameTimes.size() * p));
        return frameTimes[k];
    };

    std::printf("stress[%s] cars=%d hz=%d frames=%ld physSteps=%ld wall=%.3fs\n", engine, cars, hz,
                s.frames, s.physSteps, wall);
    std::printf("  frame ms: mean=%.2f p50=%.2f p99=%.2f max=%.2f   (%.1f fps wall)\n",
                1e3 * s.updateSec / n, pct(0.50), pct(0.99), pct(1.0), n / wall);
    std::printf("  step  ms: vehicle=%.4f world=%.4f   badStates=%d  maxSpeed=%.1f m/s  max|y|=%.1f\n",
                1e3 * s.vehicleSec / steps, 1e3 * s.stepSec / steps, badStates, worstSpeed, worstY);
    backend.Shutdown();
    return badStates == 0 ? 0 : 1;
}

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
    bool bench = false;
    int benchCars = 16;
    int benchHz = 0;  // 0 = use the configured rate
    const char* benchEngine = nullptr;  // null = all backends
    bool stress = false;
    int stressCars = 120;
    int stressHz = 480;
    double stressSeconds = 20.0;
    const char* stressEngine = "rpc";
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--drive-test") == 0) {
            driveTest = true;
        } else if (std::strcmp(argv[i], "--stress") == 0) {
            stress = true;
            if (i + 2 < argc && argv[i + 1][0] != '-' && argv[i + 2][0] != '-') {
                stressCars = std::clamp(std::atoi(argv[i + 1]), 1, 120);
                stressHz = std::max(60, std::atoi(argv[i + 2]));
                i += 2;
            }
            if (i + 1 < argc && argv[i + 1][0] != '-' && std::atof(argv[i + 1]) > 0.0) {
                stressSeconds = std::atof(argv[++i]);
            }
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                stressEngine = argv[++i];
            }
        } else if (std::strcmp(argv[i], "--shot") == 0 && i + 1 < argc) {
            shotPath = argv[++i];
        } else if (std::strcmp(argv[i], "--menu") == 0) {
            menuShot = true;  // keep the main menu up for --shot (QA visual)
        } else if (std::strcmp(argv[i], "--settings") == 0) {
            settingsShot = true;  // open the settings screen for --shot (QA visual)
        } else if (std::strcmp(argv[i], "--dump-scene") == 0 && i + 1 < argc) {
            dumpPath = argv[++i];
        } else if (std::strcmp(argv[i], "--bench") == 0) {
            bench = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                benchCars = std::atoi(argv[++i]);
                if (benchCars < 1) benchCars = 1;
                if (benchCars > 120) benchCars = 120;
            }
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                benchHz = std::max(60, std::atoi(argv[++i]));
            }
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                benchEngine = argv[++i];
            }
        }
    }

    // Test/automation mode: never read or write config.rushconf. Tests pin
    // their environment in memory; the user's settings must survive untouched
    // (a settings/race-setup Save from a test would leak automation overrides
    // like race.autostart=true into the user's config).
    if (driveTest || bench || dumpPath || shotPath || menuShot || settingsShot || stress) {
        Config::SetTestMode(true);
    }

    Config config;
    config.Load(DataPath("config.rushconf"));
    // Prove the rush-engine link is live (static libs drop unreferenced TUs).
    std::printf("rush-engine %s\n", rush::VersionString());
    // All CLI automation paths (tests, screenshots, scene dumps) skip the menu.
    if ((driveTest || dumpPath || shotPath || bench || stress) && !menuShot && !settingsShot) {
        config.SetString("race", "autostart", "true");
    }
    if (settingsShot) config.SetString("race", "settings_qa", "true");

    // JobSystem width: deterministic serial for drive-test; native parallel
    // pool for bench (async performance) and interactive play. Emscripten
    // has no pthreads in our build (no COOP/COEP) — always serial there.
    if (driveTest) {
        rush::JobSystem::Get().Configure(0);
        return RunDriveTest(config);
    }
#if defined(__EMSCRIPTEN__)
    rush::JobSystem::Get().Configure(0);
#else
    if (bench || (!menuShot && !settingsShot && !dumpPath && !shotPath)) {
        rush::JobSystem::Get().Configure(std::thread::hardware_concurrency());
    } else {
        rush::JobSystem::Get().Configure(0);
    }
#endif

    if (bench) {
        // Local config only — never Save, same rule as the test paths.
        int rc = 0;
        if (benchEngine) {
            rc |= RunBench(config, benchCars, benchEngine, 10.0, benchHz);
        } else {
            for (const char* engine : {"qu3e", "nudge", "rpc"}) {
                rc |= RunBench(config, benchCars, engine, 10.0, benchHz);
            }
        }
        return rc;
    }

    if (stress) {
        return RunStress(config, stressCars, stressHz, stressSeconds, stressEngine);
    }

    if (dumpPath) {
        NullBackend backend;
        ScriptedInput testInput;
        backend.captureDraws = true;
        backend.Init(1280, 720, "dump-scene");
        config.SetString("race", "cars", "2");
        Game game(backend, testInput.system, config);
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

    // Input runs on its own thread: RaylibInputSource samples the device,
    // InputSystem recognizes gestures and publishes to InputBus; the game
    // drains on the main thread. Tests/WASM keep PumpOnce inline instead.
    RaylibInputSource inputSource;
    InputSystem inputSystem(inputSource);
    inputSystem.Start();

    // Interactive runs get sound (miniaudio: WASAPI/DirectSound on Windows).
    // Tests and CLI capture paths stay silent and headless.
    MiniaudioAudio audio;
    if (!audio.Init(config.GetString("audio", "backend", "auto").c_str())) {
        std::fprintf(stderr, "audio: device init failed, running silent\n");
    }

    Game game(backend, inputSystem, config, &audio);
    // Native play: race stages run on the sim worker; menus stay on main.
    // Headless tests / --bench keep inline (deterministic pump).
    #if !defined(__EMSCRIPTEN__)
    game.SetThreadedSim(true);
#endif
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
    // then would leak autostart=true / settings_qa into the user's config.rushconf
    // and the main menu would silently disappear on the next launch.
    if (!shotPath && !settingsShot) config.Save(DataPath("config.rushconf"));
    inputSystem.Stop();
    backend.Shutdown();
    return 0;
}