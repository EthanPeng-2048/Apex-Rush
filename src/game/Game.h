#pragma once

#include <memory>
#include <string>
#include <vector>

#include "audio/IAudio.h"
#include "core/Config.h"
#include "game/Track.h"
#include "physics/IPhysicsWorld.h"
#include "render/IRenderer.h"
#include "vehicle/Vehicle.h"

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
    // Global settings rows: assists + video/physics + audio + BACK.
    static constexpr int kSettingsRowCount = 13;

    Game(IRenderer& renderer, IInput& input, Config& config, IAudio* audio = nullptr);

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
    void UpdateRace(float dt);
    void UpdateCamera(float dt);

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

    // Menu / settings state.
    InputState rawInput_;
    int menuCursor_ = 0;
    int settingsCursor_ = 0;
    int raceSetupCursor_ = 0;
};

}  // namespace apex
