#pragma once

#include <memory>
#include <vector>

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
    bool finished = false;
};

// Owns the world, track, racers and UI. One instance per running game.
class Game {
public:
    Game(IRenderer& renderer, IInput& input, Config& config);

    bool Init();
    // Update (input, physics, race), then Render3D (inside BeginScene/EndScene),
    // then RenderHUD (2D overlay, after EndScene).
    void Update();
    void Render3D();
    void RenderHUD();
    bool WantsQuit() const { return wantsQuit_; }

    // Introspection for tests and (later) the network layer.
    const VehicleState& PlayerState() const { return racers_.front().vehicle.State(); }
    RacePhase Phase() const { return phase_; }
    float RaceClock() const { return raceClock_; }
    const Track& GetTrack() const { return track_; }

private:
    void BuildWorld();
    void LoadSettings();
    void SpawnRacers();
    void StartRace();
    void UpdatePlayerInput();
    void UpdateMenu();
    void UpdateSettings();
    void RenderMenu();
    void RenderSettings();
    void UpdateAI(Racer& racer, float dt);
    void StepPhysics(float dt);
    void UpdateRace(float dt);
    void UpdateCamera(float dt);

    IRenderer& renderer_;
    IInput& input_;
    Config& config_;

    std::unique_ptr<IPhysicsWorld> world_;
    Track track_;
    std::vector<Racer> racers_;

    RacePhase phase_ = RacePhase::Countdown;
    float phaseTimer_ = 3.5f;
    float raceClock_ = 0.f;
    int totalLaps_ = 3;
    int circuitIndex_ = 0;

    // Camera
    int cameraMode_ = 0;  // 0 chase, 1 far chase, 2 hood, 3 orbit
    float cameraSmooth_ = 0.f;
    Vec3 cameraPos_{0.f, 8.f, -14.f};
    Vec3 cameraTarget_;

    // Fixed-step accumulator
    float accumulator_ = 0.f;
    static constexpr float kFixedDt = 1.f / 240.f;

    std::string statusText_;
    float statusTimer_ = 0.f;
    bool wantsQuit_ = false;

    // Menu / settings state.
    InputState rawInput_;
    int menuCursor_ = 0;
    int settingsCursor_ = 0;
};

}  // namespace apex
