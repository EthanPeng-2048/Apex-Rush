#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "audio/IAudio.h"
import rush.engine.core;
#include "game/Track.h"
import rush.engine.physics;
import rush.engine.render;
import rush.engine.input;
import rush.engine.runtime;

namespace apex {

// Declares the input events the game consumes (event name + gesture + keys).
// Called by Game::Init; tests may also call it directly before PumpOnce.
void RegisterGameInput(InputSystem& input);

// Race phases drive UI and input gating.
enum class RacePhase {
    Menu,
    Settings,
    RaceSetup,   // pre-race config (circuit / laps / cars / damage)
    Countdown,
    Racing,
    Finished,
};

struct Racer {
    Vehicle vehicle;
    CarInput input;
    std::string name;
    bool isPlayer = false;
    int lap = 0;
    float lapDistance = 0.f;
    float lastLapTime = 0.f;
    float bestLapTime = 0.f;
    float currentLapStart = 0.f;
    float raceTime = 0.f;
    int position = 1;
    float progress = 0.f;  // lap + distance fraction, for ranking
    float stuckTimer = 0.f;
    bool finished = false;
};

// Owns the world, track, racers and UI. One instance per running game.
class Game {
public:
    // Race-setup rows: circuit, laps, cars, damage, start, back.
    static constexpr int kRaceSetupRowCount = 6;
    // Global settings rows: season + feature gates, assists, video/physics,
    // audio + BACK.
    static constexpr int kSettingsRowCount = 17;

    Game(IRenderer& renderer, InputSystem& input, Config& config, IAudio* audio = nullptr);
    ~Game();

    bool Init();
    // Update (input, physics, race), then Render3D (inside BeginScene/EndScene),
    // then RenderHUD (2D overlay, after EndScene).
    void Update();
    void Render3D();
    void RenderHUD();
    bool WantsQuit() const { return wantsQuit_; }

    // Introspection for tests and (later) the network layer.
    const VehicleState& PlayerState() const { return racers_.front().vehicle.State(); }
    const std::vector<Racer>& Racers() const { return racers_; }
    RacePhase Phase() const { return phase_; }
    float RaceClock() const { return raceClock_; }
    const Track& GetTrack() const { return track_; }
    // True while the player's DRS toggle is latched on (test hook: the
    // latch resets when leaving a DRS zone, which drives re-entry behavior).
    bool PlayerDrsWanted() const { return drsWanted_; }

    // Optional section timers for --bench (off by default; zero overhead).
    struct BenchSample {
        double aiSec = 0.0;
        double vehicleSec = 0.0;
        double stepSec = 0.0;
        double contactSec = 0.0;
        double raceSec = 0.0;
        double updateSec = 0.0;
        long frames = 0;
        long physSteps = 0;
    };
    void SetBench(bool on) { benchOn_ = on; }
    void ResetBench() { bench_ = BenchSample{}; }
    const BenchSample& Bench() const { return bench_; }

    // Async runtime: when enabled, race phases run on a worker thread and
    // Render3D reads a published snapshot (double buffer). Menu phases stay
    // on the caller thread. drive-test / WASM keep this off (inline pump).
    struct CarPose {
        Transform chassis;
        VehicleState state;
        VehicleParams params;
        bool isPlayer = false;
        Color tint{1.f, 1.f, 1.f, 1.f};
        // Race statistics — HUD reads these from the snapshot while the sim
        // worker owns racers_ (position/lap/… are written by the sim).
        int lap = 0;
        int position = 1;
        float lastLapTime = 0.f;
        float bestLapTime = 0.f;
        float raceTime = 0.f;
        // Aero part health (owned by AeroKit; mirrored for the snapshot/HUD).
        std::array<float, 4> partHealth{1.f, 1.f, 1.f, 1.f};
        float damageTotal = 0.f;
    };
    struct RaceSnapshot {
        std::vector<CarPose> cars;
        RacePhase phase = RacePhase::Menu;
        float raceClock = 0.f;
        bool valid = false;
        // steady_clock seconds at Publish — render extrapolates from this so
        // cars move continuously between snapshot ticks (camera is smoothed;
        // discrete car steps showed up as synchronized "all cars flash").
        double publishTime = 0.0;
        // Countdown overlay + transient status line ("AI 3 reset") — both
        // sim-owned; the HUD must not read the live members mid-step.
        float phaseTimer = 0.f;
        std::string statusText;
        float statusTimer = 0.f;
    };
    void SetThreadedSim(bool on);
    bool ThreadedSim() const { return threadedSim_; }
    // Last snapshot taken by Update() on the main thread. Null before the
    // first race frame. Tests must read this instead of Racers()/PlayerState()
    // while the sim worker is running — those are live racers_ views.
    std::shared_ptr<const RaceSnapshot> FrameSnapshot() const { return frameSnap_; }

private:
    // Async race build: everything the background thread reads, snapshotted
    // on the main thread so the worker never touches config_ or renderer_.
    struct RaceBuildConfig {
        std::string mapName;
        float fixedDt = 1.f / 240.f;
        // [physics] engine: "qu3e" | "nudge" | "rpc" (Rush Physics Core).
        std::string engine = "qu3e";
        int carCount = 4;
        bool damage = true;
        AssistLevel absLevel = AssistLevel::Sport;
        AssistLevel tcLevel = AssistLevel::Sport;
        AssistLevel escLevel = AssistLevel::Full;
        AssistLevel steerLevel = AssistLevel::Full;
        bool seasonDrs = true;
        bool seasonAero = false;
        bool seasonOverride = false;
    };
    // Staging result: built off-thread, applied on the main thread in one
    // swap (meshes are the only half that needs the renderer).
    struct RaceBuildResult {
        std::unique_ptr<IPhysicsWorld> world;
        Track track;  // offline half only; BuildMeshes runs on main
        MapData map;
        std::vector<Racer> racers;
    };

    void BuildWorld();
    void LoadSettings();
    void SpawnRacers();
    static void SpawnRacersInto(std::vector<Racer>& out, const Track& track,
                                IPhysicsWorld& world, const RaceBuildConfig& cfg);
    RaceBuildConfig MakeRaceBuildConfig();
    // Async path (interactive): snapshot config, spawn the build thread.
    void BeginRaceLoad();
    // Join a finished build and swap it in (called once per Update).
    void PollRaceLoad();
    void FinishRaceLoad();
    void CancelRaceLoad();
    void StartRace();
    void UpdatePlayerInput();
    // Sim-thread half of player control: consumes the intent mailbox
    // (input mapping, DRS latching, autopilot, restart). Never called on
    // the main thread while the sim worker is running.
    void ApplyPlayerIntent();
    void UpdateMenu();
    void UpdateSettings();
    void UpdateRaceSetup();
    void RenderMenu();
    void RenderSettings();
    void RenderRaceSetup();
    void UpdateAI(Racer& racer, float dt);
    void StepPhysics(float dt);
    void ProcessCollisionEvents(float dt);
    void UpdateRace(float dt);
    void UpdateCamera(float dt);
    // Sim half of the frame (AI + fixed-step physics + race rules). Runs on
    // the caller thread in inline mode, or on the sim worker when threaded.
    void UpdateSim(float frameDt);
    void PublishSnapshot();
    void StartSimWorker();
    void StopSimWorker();
    void UpdateCameraFrom(const VehicleState& player, float dt);

    IRenderer& renderer_;
    InputSystem& input_;
    Config& config_;
    IAudio* audio_ = nullptr;  // optional; null in headless tests

    std::unique_ptr<IPhysicsWorld> world_;
    Track track_;
    std::vector<Racer> racers_;

    // Atomic: written by the main thread (pause → Menu) and the sim worker
    // (countdown → Racing → Finished) while both may be live.
    std::atomic<RacePhase> phase_{RacePhase::Countdown};
    float phaseTimer_ = 3.5f;
    float raceClock_ = 0.f;
    int totalLaps_ = 3;
    std::string mapName_ = "test_circuit";
    std::vector<std::string> mapList_;

    // Camera
    int cameraMode_ = 0;  // 0 chase, 1 far chase, 2 hood, 3 orbit
    float cameraSmooth_ = 0.f;
    Vec3 cameraPos_{0.f, 8.f, -14.f};
    Vec3 cameraTarget_;

    // Fixed-step accumulator. Step size comes from [physics] hz (120/240/480),
    // default 240 Hz — the tire contact damping and stiff springs are tuned
    // for 240, lower rates trade accuracy for speed.
    float accumulator_ = 0.f;
    float fixedDt_ = 1.f / 240.f;

    VideoConfig video_;

    std::string statusText_;
    float statusTimer_ = 0.f;
    bool wantsQuit_ = false;

    // Latched DRS request: F only arms/disarms inside a map-marked DRS zone;
    // leaving the zone closes DRS and resets the latch, so re-entering a
    // zone later starts closed (the driver must re-arm). Written on the sim
    // thread (zone check needs the car's position); read by test hooks.
    std::atomic<bool> drsWanted_{false};

    // Season feature gates ([season] in config): classic (DRS) and 2026
    // (active aero + override) never mix. The settings screen exposes a
    // season preset plus per-feature toggles; toggling a feature flips the
    // preset display to CUSTOM.
    bool seasonDrs_ = true;
    bool seasonAero_ = false;
    bool seasonOverride_ = false;

    // Menu / settings state.
    InputState rawInput_;
    int menuCursor_ = 0;
    int settingsCursor_ = 0;
    int raceSetupCursor_ = 0;

    // Main → sim control mailbox. Continuous fields (axes) are overwritten
    // every main-thread frame; edge requests are set-only by the producer
    // and cleared by the sim on consume, so a render loop faster than the
    // sim tick can't swallow a one-frame IsKeyPressed edge.
    struct PlayerIntent {
        InputState raw{};
        bool autopilot = false;
        bool restart = false;
        bool drsToggle = false;
        bool wingToggle = false;
    };
    std::mutex intentMutex_;
    PlayerIntent intent_;

    bool benchOn_ = false;
    BenchSample bench_;

    // Async sim worker + snapshot handshake (rush.engine.runtime).
    // frameSnap_ is the main thread's once-per-frame CopyFront() — camera,
    // audio, HUD and Render3D all read this so they never mix two sim ticks.
    // shared_ptr handoff: no per-frame deep copy of the snapshot payload.
    bool threadedSim_ = false;
    rush::SnapshotTripleBuffer<RaceSnapshot> snapshots_;
    std::shared_ptr<const RaceSnapshot> frameSnap_;
    rush::SimThread simWorker_;
    std::atomic<bool> simRun_{false};

    // Async race build handoff (RaceSetup → Countdown). The worker fills
    // pending_ then raises loadDone_; the main thread joins before reading
    // pending_ (join synchronizes the handoff — no payload lock needed).
    std::thread loadThread_;
    std::unique_ptr<RaceBuildResult> pending_;
    std::atomic<bool> loadDone_{false};
    std::atomic<bool> loadCancelled_{false};
};

}  // namespace apex