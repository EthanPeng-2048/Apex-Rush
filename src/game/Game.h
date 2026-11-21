#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "audio/IAudio.h"
import rush.engine.core;
#include "game/Track.h"
import rush.engine.physics;
import rush.engine.render;
import rush.engine.runtime;

namespace apex {

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
    VehicleInput input;
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

    Game(IRenderer& renderer, IInput& input, Config& config, IAudio* audio = nullptr);
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
    };
    struct RaceSnapshot {
        std::vector<CarPose> cars;
        RacePhase phase = RacePhase::Menu;
        float raceClock = 0.f;
        bool valid = false;
    };
    void SetThreadedSim(bool on);
    bool ThreadedSim() const { return threadedSim_; }

private:
    void BuildWorld();
    void LoadSettings();
    void SpawnRacers();
    void StartRace();
    void UpdatePlayerInput();
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
    IInput& input_;
    Config& config_;
    IAudio* audio_ = nullptr;  // optional; null in headless tests

    std::unique_ptr<IPhysicsWorld> world_;
    Track track_;
    std::vector<Racer> racers_;

    RacePhase phase_ = RacePhase::Countdown;
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
    // zone later starts closed (the driver must re-arm).
    bool drsWanted_ = false;

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

    bool benchOn_ = false;
    BenchSample bench_;

    // Async sim worker + snapshot handshake (rush.engine.runtime).
    bool threadedSim_ = false;
    rush::SnapshotDoubleBuffer<RaceSnapshot> snapshots_;
    rush::SimThread simWorker_;
    std::atomic<bool> simRun_{false};
    // Player input written by the main thread, read by the sim worker.
    std::atomic<float> asyncThrottle_{0.f};
    std::atomic<float> asyncBrake_{0.f};
    std::atomic<float> asyncSteer_{0.f};
    std::atomic<uint32_t> asyncBits_{0};  // VehicleInput flag bits
};

}  // namespace apex