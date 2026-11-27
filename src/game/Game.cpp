#include "game/Game.h"

#include <algorithm>
#include <vector>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>
#include <cstring>

import rush.engine.core;
import rush.engine.job;
import rush.engine.physics;
import rush.engine.render;
import rush.engine.input;

namespace apex {

namespace {

using BenchClock = std::chrono::steady_clock;

double BenchSec(BenchClock::time_point a, BenchClock::time_point b) {
    return std::chrono::duration<double>(b - a).count();
}

// "test_circuit" -> "TEST CIRCUIT" (raylib's font has no lowercase issues,
// but the menu looks better uppercase).
std::string MapDisplayName(const std::string& id) {
    std::string out;
    for (char c : id) {
        if (c == '_' || c == '-') {
            out.push_back(' ');
        } else {
            out.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
        }
    }
    return out;
}

// Assist levels are stored as "off" / "sport" / "full"; legacy "true"/"false"
// values map to sport / off so old config files keep working.
AssistLevel ParseAssistLevel(const Config& config, const char* key, AssistLevel fallback) {
    std::string v = config.GetString("assists", key, "");
    for (char& c : v) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (v == "full" || v == "2") return AssistLevel::Full;
    if (v == "sport" || v == "true" || v == "1") return AssistLevel::Sport;
    if (v == "off" || v == "false" || v == "0") return AssistLevel::Off;
    return fallback;
}

// Aero force rules live in the engine's AeroKit now (part health → area,
// DRS / X-Z wing mode / pitch balance). The game only applies damage via
// Vehicle::ProcessContacts.

const char* AssistLevelName(AssistLevel level) {
    switch (level) {
        case AssistLevel::Full:
            return "FULL";
        case AssistLevel::Sport:
            return "SPORT";
        default:
            return "OFF";
    }
}

// Playability target: steering aid fully on, the rest at Sport.
AssistLevel DefaultAssistLevel(const char* key) {
    return std::strcmp(key, "steer") == 0 ? AssistLevel::Full : AssistLevel::Sport;
}

void SaveAssistLevel(Config& config, const char* key, AssistLevel level) {
    config.SetString("assists", key,
                     level == AssistLevel::Full ? "full"
                     : level == AssistLevel::Sport ? "sport"
                                                   : "off");
}

// Video / physics presets cycled by the settings rows.
struct ResPreset {
    int width, height;
};
const ResPreset kResPresets[] = {{1280, 720}, {1600, 900}, {1920, 1080}, {2560, 1440}};
constexpr int kResPresetCount = 4;
const int kFpsPresets[] = {0, 30, 60, 120, 144};       // 0 = uncapped
constexpr int kFpsPresetCount = 5;
const int kPhysicsPresets[] = {120, 240, 480};
constexpr int kPhysicsPresetCount = 3;

// Cycle a preset index by menu direction (select = next).
int CyclePreset(int index, int count, int dir) {
    return (index + (dir != 0 ? (dir > 0 ? 1 : -1) : 1) + count) % count;
}

std::string FormatTime(float seconds) {
    if (seconds <= 0.f) return "--:--.---";
    const int minutes = static_cast<int>(seconds / 60.f);
    const float rest = seconds - minutes * 60.f;
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%d:%06.3f", minutes, rest);
    return buffer;
}

std::string Pad(const std::string& s, size_t width) {
    return s.size() >= width ? s.substr(0, width) : s + std::string(width - s.size(), ' ');
}

}  // namespace

// Game-side input registration: declare the events the game consumes, their
// gesture kinds and default key bindings. Called from Game::Init before any
// phase runs; the input thread recognizes these and publishes to InputBus.
void RegisterGameInput(InputSystem& input) {
    InputRegistry& reg = input.Registry();
    const DeviceKey kb = {DeviceKind::Keyboard, 0, 0};
    auto key = [&](KeyCode k) {
        DeviceKey d = kb;
        d.code = static_cast<uint16_t>(k);
        return d;
    };

    // Race continuous axes (Hold): keyboard OR-analog. The axis bindings let
    // scripted tests (NullInputSource.scripted.axes) inject analog throttle /
    // brake / steer; the keyboard keys bind the same names for real play.
    reg.Register({.name = "throttle", .gesture = Gesture::Hold,
                  .keys = {{.key = key(KeyCode::W)}, {.key = key(KeyCode::Up)},
                           {.axisIndex = 0, .axisScale = 1.f}},
                  .context = "race"});
    reg.Register({.name = "brake", .gesture = Gesture::Hold,
                  .keys = {{.key = key(KeyCode::S)}, {.key = key(KeyCode::Down)},
                           {.axisIndex = 1, .axisScale = 1.f}},
                  .context = "race"});
    reg.Register({.name = "steer", .gesture = Gesture::Hold,
                  .keys = {{.key = key(KeyCode::A), .axisScale = -1.f},
                           {.key = key(KeyCode::D), .axisScale = 1.f},
                           {.key = key(KeyCode::Left), .axisScale = -1.f},
                           {.key = key(KeyCode::Right), .axisScale = 1.f},
                           {.axisIndex = 2, .axisScale = 1.f}},
                  .context = "race"});
    reg.Register({.name = "handbrake", .gesture = Gesture::Hold,
                  .keys = {{.key = key(KeyCode::Space)}}, .context = "race"});
    reg.Register({.name = "reverse", .gesture = Gesture::Hold,
                  .keys = {{.key = key(KeyCode::R)}}, .context = "race"});
    reg.Register({.name = "overtake", .gesture = Gesture::Hold,
                  .keys = {{.key = key(KeyCode::LShift)}, {.key = key(KeyCode::RShift)}},
                  .context = "race"});
    reg.Register({.name = "lookback", .gesture = Gesture::Hold,
                  .keys = {{.key = key(KeyCode::B)}}, .context = "race"});

    // Race discrete actions (Click): DRS / wing / camera / pause / restart.
    reg.Register({.name = "drs", .gesture = Gesture::Click,
                  .keys = {{.key = key(KeyCode::F)}}, .context = "race"});
    reg.Register({.name = "wing", .gesture = Gesture::Click,
                  .keys = {{.key = key(KeyCode::V)}}, .context = "race"});
    reg.Register({.name = "camera", .gesture = Gesture::Click,
                  .keys = {{.key = key(KeyCode::C)}}, .context = "race"});
    reg.Register({.name = "pause", .gesture = Gesture::Click,
                  .keys = {{.key = key(KeyCode::Escape)}, {.key = key(KeyCode::P)}}});
    reg.Register({.name = "restart", .gesture = Gesture::Click,
                  .keys = {{.key = key(KeyCode::Enter)}}});

    // Menu navigation (Click, context-gated to menu phases).
    reg.Register({.name = "menu.up", .gesture = Gesture::Click,
                  .keys = {{.key = key(KeyCode::Up)}, {.key = key(KeyCode::W)}},
                  .context = "menu"});
    reg.Register({.name = "menu.down", .gesture = Gesture::Click,
                  .keys = {{.key = key(KeyCode::Down)}, {.key = key(KeyCode::S)}},
                  .context = "menu"});
    reg.Register({.name = "menu.left", .gesture = Gesture::Click,
                  .keys = {{.key = key(KeyCode::Left)}, {.key = key(KeyCode::A)}},
                  .context = "menu"});
    reg.Register({.name = "menu.right", .gesture = Gesture::Click,
                  .keys = {{.key = key(KeyCode::Right)}, {.key = key(KeyCode::D)}},
                  .context = "menu"});
    reg.Register({.name = "menu.select", .gesture = Gesture::Click,
                  .keys = {{.key = key(KeyCode::Enter)}, {.key = key(KeyCode::Space)}},
                  .context = "menu"});
    reg.Register({.name = "menu.back", .gesture = Gesture::Click,
                  .keys = {{.key = key(KeyCode::Escape)}}, .context = "menu"});
}

Game::Game(IRenderer& renderer, InputSystem& input, Config& config, IAudio* audio)
    : renderer_(renderer), input_(input), config_(config), audio_(audio) {}

bool Game::Init() {
    StartRace();
    // Register the game's input events: modules that need input declare
    // their (event name, gesture, keys) here — the input thread recognizes
    // them and publishes to InputBus; the game only Drain()s.
    RegisterGameInput(input_);
    // User rebinds from config.rushconf [input] (<name>.keys = "a,d"). A bad
    // value is non-fatal: keep the registered defaults.
    if (auto r = input_.Registry().ApplyOverrides(config_, "input"); !r) {
        std::fprintf(stderr, "[input] rebind skipped: %s\n", r.error().message.c_str());
    }
    // Tests and automation skip straight to the grid; normal launches open
    // the main menu over the parked scene. `race.settings_qa` opens the
    // settings screen directly for --shot QA captures (never saved).
    if (config_.GetBool("race", "settings_qa", false)) {
        phase_ = RacePhase::Settings;
    } else {
        phase_ = config_.GetBool("race", "autostart", false) ? RacePhase::Countdown
                                                             : RacePhase::Menu;
    }
    return true;
}

void Game::LoadSettings() {
    mapList_ = ListMaps(DataPath("assets/maps"));
    if (mapList_.empty()) mapList_.push_back("test_circuit");
    mapName_ = config_.GetString("race", "map", mapList_.front());
    if (std::find(mapList_.begin(), mapList_.end(), mapName_) == mapList_.end()) {
        mapName_ = mapList_.front();
    }
    totalLaps_ = std::clamp(config_.GetInt("race", "laps", 3), 1, 20);

    // Video: window mode / size / fps cap / quality shadows.
    video_.borderlessFullscreen = config_.GetString("video", "mode", "windowed") == "borderless";
    video_.width = std::clamp(config_.GetInt("video", "width", 1280), 640, 7680);
    video_.height = std::clamp(config_.GetInt("video", "height", 720), 360, 4320);
    video_.fpsLimit = std::max(0, config_.GetInt("video", "fps_limit", 0));
    video_.shadows = config_.GetString("video", "quality", "high") != "low";
    renderer_.ApplyVideo(video_);

    // Physics tick rate (fixed step).
    const int hz = std::clamp(config_.GetInt("physics", "hz", 240), 60, 1000);
    fixedDt_ = 1.f / static_cast<float>(hz);

    // Audio volume / backend.
    if (audio_) {
        audio_->SetMasterVolume(std::clamp(config_.GetInt("audio", "volume", 80), 0, 100) / 100.f);
        audio_->SetBackend(config_.GetString("audio", "backend", "auto").c_str());
    }

    // Season feature gates: classic (DRS) vs 2026 (active aero + override).
    // Defaults = classic 2025 preset (DRS on, the 2026 systems off). ERS is a
    // power-unit feature: it follows the overtake gate unless [season] ers
    // says otherwise, so a server can run a hybrid without the shift boost.
    seasonDrs_ = config_.GetBool("season", "drs", true);
    seasonAero_ = config_.GetBool("season", "active_aero", false);
    seasonOverride_ = config_.GetBool("season", "overtake", false);
    seasonErs_ = config_.GetBool("season", "ers", seasonOverride_);
}

// Fill a fresh racer list from a config snapshot. Static so the async build
// thread can run it against staging world/track objects without reading any
// Game members.
void Game::SpawnRacersInto(std::vector<Racer>& out, const Track& track, IPhysicsWorld& world,
                           const RaceBuildConfig& cfg) {
    // Large grids (up to 120) so the async bench can stress the sim; the
    // settings CARS row cycles the same range.
    const int carCount = std::clamp(cfg.carCount, 1, 120);
    out.resize(static_cast<size_t>(carCount));
    for (int i = 0; i < carCount; ++i) {
        Racer& racer = out[static_cast<size_t>(i)];
        racer.name = (i == 0) ? "YOU" : ("AI " + std::to_string(i));
        racer.isPlayer = (i == 0);

        VehicleParams params;
        if (racer.isPlayer) {
            // Difficulty assists only apply to the player's car.
            params.assists.absLevel = cfg.absLevel;
            params.assists.tcLevel = cfg.tcLevel;
            params.assists.escLevel = cfg.escLevel;
            params.controls.steerLevel = cfg.steerLevel;
        } else {
            // AI cars run a fixed assist set so player difficulty settings
            // never change their behavior. Without the counter-steer / yaw
            // aids the AI spins at the assists-off difficulty.
            params.assists.absLevel = AssistLevel::Sport;
            params.assists.tcLevel = AssistLevel::Sport;
            params.assists.escLevel = AssistLevel::Full;
            params.controls.steerLevel = AssistLevel::Full;
        }
        params.damageEnabled = cfg.damage;
        // Fitted features are a build property of the car (see
        // VehicleCapability): the vehicle gates activation itself, so a stale
        // input can never leak a system that is not on the car.
        params.capabilities = 0u;
        if (cfg.seasonDrs) params.capabilities |= VehicleCapability::Drs;
        if (cfg.seasonAero) params.capabilities |= VehicleCapability::ActiveAero;
        if (cfg.seasonOverride) params.capabilities |= VehicleCapability::Overtake;
        if (cfg.seasonErs) params.capabilities |= VehicleCapability::Ers;

        Transform pose;
        pose.position = track.StartPosition(i + cfg.gridOffset);
        pose.rotation = track.StartRotation();
        racer.vehicle.Spawn(world, pose, params, &racer);
    }
}

Game::RaceBuildConfig Game::MakeRaceBuildConfig() {
    RaceBuildConfig cfg;
    cfg.mapName = mapName_;
    cfg.fixedDt = fixedDt_;
    cfg.engine = config_.GetString("physics", "engine", "qu3e");
    cfg.carCount = std::clamp(config_.GetInt("race", "cars", 4), 1, 120);
    // LAN roles override the grid: the host drives every joined car, the client
    // only its own (spawned on its authoritative slot).
    if (netRole_ == NetRole::Host) {
        cfg.carCount = std::clamp<int>(netCarCount_, 1, net::kMaxCars);
    } else if (netRole_ == NetRole::Client) {
        cfg.carCount = 1;
        cfg.gridOffset = netConfig_.rules.carCount > 1 ? netLocalSlot_ : 0;
    }
    cfg.damage = config_.GetBool("race", "damage", true);
    cfg.absLevel = ParseAssistLevel(config_, "abs", DefaultAssistLevel("abs"));
    cfg.tcLevel = ParseAssistLevel(config_, "tc", DefaultAssistLevel("tc"));
    cfg.escLevel = ParseAssistLevel(config_, "esc", DefaultAssistLevel("esc"));
    cfg.steerLevel = ParseAssistLevel(config_, "steer", DefaultAssistLevel("steer"));
    cfg.seasonDrs = seasonDrs_;
    cfg.seasonAero = seasonAero_;
    cfg.seasonOverride = seasonOverride_;
    cfg.seasonErs = seasonErs_;
    return cfg;
}

// Build the selected collision backend: qu3e | nudge | rpc (Rush Physics Core).
static std::unique_ptr<IPhysicsWorld> MakePhysicsWorld(const std::string& engine, float stepDt) {
    if (engine == "nudge") return std::make_unique<NudgeWorld>();
    if (engine == "rpc") return std::make_unique<RushPhysicsCore>(stepDt);
    return std::make_unique<Qu3eWorld>(stepDt);
}

void Game::SpawnRacers() { SpawnRacersInto(racers_, track_, *world_, MakeRaceBuildConfig()); }

// Bench runs the parallel-island path (bit-identical numerics) and needs the
// per-stage timers; both are world settings, and the world may be (re)built
// after SetBench was called, so re-apply them on every build.
void Game::ApplyWorldTuning() {
    if (!world_) return;
    world_->SetParallelIslands(benchOn_);
    world_->SetDiagnostics(benchOn_);
}

void Game::StressPileUp(float radius, float speed) {
    if (!world_ || racers_.empty()) return;
    const Vec3 origin = track_.PositionAt(track_.TotalLength() * 0.35f) + Vec3{0.f, 1.2f, 0.f};
    for (size_t i = 0; i < racers_.size(); ++i) {
        const float a = static_cast<float>(i) * 2.39996323f;  // golden angle
        const float r = radius * std::sqrt(static_cast<float>(i) / static_cast<float>(racers_.size()));
        const Vec3 offset{std::cos(a) * r, 0.f, std::sin(a) * r};
        Transform pose;
        pose.position = origin + offset;
        pose.rotation = track_.StartRotation();
        Racer& racer = racers_[i];
        racer.vehicle.Reset(pose);
        const Vec3 inward = r > 1e-3f ? Normalize(Vec3{} - offset) : Vec3{};
        (void)world_->SetLinearVelocity(racer.vehicle.Chassis(), inward * speed);
    }
}

// (Re)build world, track and grid from the current config — used at boot and
// whenever the player starts a race from the menu (inline / drive-test).
// Interactive starts go through BeginRaceLoad() instead.
void Game::StartRace() {
    LoadSettings();
    racers_.clear();
    drsWanted_ = false;
    hostTick_ = 0;
    localTick_ = 0;
    netCarCount_ = netCarCount_ > 0 ? netCarCount_ : 1;
    // Swap collision backends in-game via [physics] engine (qu3e | nudge | rpc).
    world_ = MakePhysicsWorld(config_.GetString("physics", "engine", "qu3e"), fixedDt_);
    world_->SetGravity({0.f, -9.81f, 0.f});
    BuildWorld();
    SpawnRacers();
    phase_ = RacePhase::Countdown;
    phaseTimer_ = 3.5f;
    raceClock_ = 0.f;
    accumulator_ = 0.f;
}

// Interactive race start: LoadSettings runs here (renderer/audio are main-
// thread only), then the heavy build — map IO, collision, grid spawn — runs
// on a background thread. The menu keeps rendering; PollRaceLoad() swaps the
// result in when it's ready. Inline modes call StartRace() directly.
void Game::BeginRaceLoad() {
    LoadSettings();
    const RaceBuildConfig cfg = MakeRaceBuildConfig();

    loadCancelled_.store(false, std::memory_order_release);
    loadDone_.store(false, std::memory_order_release);
    loadThread_ = std::thread([this, cfg] {
        auto built = std::make_unique<RaceBuildResult>();
        built->world = MakePhysicsWorld(cfg.engine, cfg.fixedDt);
        built->world->SetGravity({0.f, -9.81f, 0.f});
        if (loadCancelled_.load(std::memory_order_acquire)) {
            loadDone_.store(true, std::memory_order_release);
            return;
        }
        built->map = LoadMapData(DataPath("assets/maps"), cfg.mapName);
        built->track.BuildOffline(built->map, *built->world);
        if (loadCancelled_.load(std::memory_order_acquire)) {
            loadDone_.store(true, std::memory_order_release);
            return;
        }
        SpawnRacersInto(built->racers, built->track, *built->world, cfg);
        pending_ = std::move(built);
        loadDone_.store(true, std::memory_order_release);
    });
}

void Game::PollRaceLoad() {
    if (!loadThread_.joinable() || !loadDone_.load(std::memory_order_acquire)) return;
    loadThread_.join();
    if (loadCancelled_.load(std::memory_order_acquire)) {
        pending_.reset();
        return;
    }
    if (pending_) {
        FinishRaceLoad();
    } else {
        // Build bailed without a result (defensive): fall back to the
        // synchronous path so START never deadlocks in RaceSetup.
        StartRace();
    }
    pending_.reset();
}

void Game::FinishRaceLoad() {
    // Main thread, sim worker parked (phase is still RaceSetup).
    racers_.clear();  // vehicles drop their world refs before the world dies
    track_.Dispose(renderer_);
    world_ = std::move(pending_->world);
    track_ = std::move(pending_->track);
    track_.BuildMeshes(pending_->map, renderer_);
    racers_ = std::move(pending_->racers);
    drsWanted_ = false;
    phase_ = RacePhase::Countdown;
    phaseTimer_ = 3.5f;
    raceClock_ = 0.f;
    accumulator_ = 0.f;
}

void Game::CancelRaceLoad() {
    if (!loadThread_.joinable()) return;
    loadCancelled_.store(true, std::memory_order_release);
    loadThread_.join();
    pending_.reset();
}

Game::~Game() {
    StopSimWorker();
    CancelRaceLoad();
}

void Game::BuildWorld() {
    const MapData map = LoadMapData(DataPath("assets/maps"), mapName_);
    track_.Build(map, *world_, renderer_);
}

void Game::Update() {
    const BenchClock::time_point benchT0 = BenchClock::now();
    const float frameDt = std::min(renderer_.FrameTime(), 0.1f);

    UpdatePlayerInput();
    // Apply a finished async race build before dispatch — may flip
    // RaceSetup → Countdown so this frame already starts the sim worker.
    PollRaceLoad();

    // LAN: pump discovery + session every frame (any phase), so lobby traffic
    // keeps flowing while the room browser is open.
    if (netRole_ != NetRole::Offline) LanTick();

    // A phase change opens its screen — the engine owns focus, so "reset the
    // cursor" is a screen open. Done before dispatch so this frame's keys land
    // on the screen we are about to draw, and again after it because a handler
    // may have switched phase for the frame that is about to be drawn.
    SyncUiPhase();

    // Single load: phase_ is written by both threads (pause vs. race rules),
    // so read it once per frame for consistent dispatch.
    const RacePhase ph = phase_;
    const bool menuPhase = ph == RacePhase::Menu || ph == RacePhase::Settings ||
                           ph == RacePhase::RaceSetup || ph == RacePhase::Lan;

    if (menuPhase) {
        // Menus always own the caller thread; park the sim worker.
        StopSimWorker();
        // Decay the status line here too — the sim-owned decrement in
        // UpdateSim doesn't run while parked, and a stale "car reset"
        // message must not survive into the next race's HUD.
        if (statusTimer_ > 0.f) statusTimer_ -= frameDt;
        if (ph == RacePhase::Menu) {
            UpdateMenu();
        } else if (ph == RacePhase::Settings) {
            UpdateSettings();
        } else if (ph == RacePhase::Lan) {
            UpdateLan();
        } else {
            UpdateRaceSetup();
        }
    } else if (threadedSim_) {
        // Race on the worker: main only feeds input + camera/audio from this
        // frame's snapshot copy. UpdateSim runs on simWorker_.
        if (!simWorker_.Running()) StartSimWorker();
        // Take ONE stable handle per frame — Render3D and audio/camera below
        // must not observe a Publish that lands mid-draw (position flicker).
        frameSnap_ = snapshots_.CopyFront();
    } else {
        UpdateSim(frameDt);
    }

    SyncUiPhase();

    if (threadedSim_ && !menuPhase && simWorker_.Running()) {
        // Main-thread half of the race frame: camera + audio from the frame
        // snapshot (never touches racers_ while the worker owns them).
        // statusTimer_ is sim-owned and decremented in UpdateSim.
        if (frameSnap_ && frameSnap_->valid && !frameSnap_->cars.empty()) {
            const VehicleState& s = frameSnap_->cars[0].state;
            UpdateCameraFrom(s, frameDt);
        }
        if (audio_ && frameSnap_ && frameSnap_->valid && !frameSnap_->cars.empty()) {
            const VehicleState& s = frameSnap_->cars[0].state;
            AudioState audioState;
            audioState.engineRpm = s.rpm;
            audioState.engineLoad = s.throttle;
            audioState.speed = s.speed;
            for (const WheelState& wheel : s.wheels) {
                const float slip = std::max(std::fabs(wheel.slipRatio) * 1.8f,
                                            std::fabs(wheel.slipAngleDeg) / 18.f);
                audioState.skid = std::max(audioState.skid, slip);
            }
            audioState.skid = Clampf(audioState.skid, 0.f, 1.f);
            audio_->Update(audioState);
        }
    } else {
        UpdateCamera(frameDt);

        if (audio_ && !racers_.empty()) {
            const VehicleState& s = racers_.front().vehicle.State();
            AudioState audioState;
            audioState.engineRpm = s.rpm;
            audioState.engineLoad = s.throttle;
            audioState.speed = s.speed;
            for (const WheelState& wheel : s.wheels) {
                const float slip = std::max(std::fabs(wheel.slipRatio) * 1.8f,
                                            std::fabs(wheel.slipAngleDeg) / 18.f);
                audioState.skid = std::max(audioState.skid, slip);
            }
            audioState.skid = Clampf(audioState.skid, 0.f, 1.f);
            audio_->Update(audioState);
        }
    }

    if (benchOn_) {
        const double sec = BenchSec(benchT0, BenchClock::now());
        bench_.updateSec += sec;
        bench_.frameTimesMs.push_back(sec * 1e3);
        ++bench_.frames;
    }
}

void Game::UpdateSim(float frameDt) {
    // LAN client: only the local car is simulated (prediction + rollback).
    if (netRole_ == NetRole::Client) {
        LanClientSimTick(frameDt);
        return;
    }

    // Latch the player mailbox before anything reads player.input — mapping,
    // DRS zone latching, autopilot and restart all run on this thread so
    // they never race the sim worker's view of racers_.
    ApplyPlayerIntent();

    if (netRole_ == NetRole::Host) {
        // Remote cars are driven by their owners' inputs, never by the AI.
        LanApplyRemoteInputs();
    } else if (benchOn_) {
        const BenchClock::time_point t0 = BenchClock::now();
        rush::JobSystem::Get().ParallelFor(racers_.size(), [&](std::size_t i) {
            Racer& racer = racers_[i];
            if (!racer.isPlayer && !racer.finished) UpdateAI(racer, frameDt);
        });
        bench_.aiSec += BenchSec(t0, BenchClock::now());
    } else {
        rush::JobSystem::Get().ParallelFor(racers_.size(), [&](std::size_t i) {
            Racer& racer = racers_[i];
            if (!racer.isPlayer && !racer.finished) UpdateAI(racer, frameDt);
        });
    }

    // Fixed-step physics (rate from [physics] hz, default 240).
    accumulator_ += frameDt;
    while (accumulator_ >= fixedDt_) {
        StepPhysics(fixedDt_);
        // Gameplay stage: drain CollisionBus after the physics stage has
        // published (strict async — no producer-side callbacks).
        ProcessCollisionEvents(fixedDt_);
        accumulator_ -= fixedDt_;
    }

    if (benchOn_) {
        const BenchClock::time_point t0 = BenchClock::now();
        UpdateRace(frameDt);
        bench_.raceSec += BenchSec(t0, BenchClock::now());
    } else {
        UpdateRace(frameDt);
    }

    // Status line clock is sim-owned (UpdateRace may arm it for a reset).
    if (statusTimer_ > 0.f) statusTimer_ -= frameDt;

    // Inline mode draws straight from racers_ — only the async path needs
    // a published snapshot.
    if (threadedSim_) PublishSnapshot();

    // Host: hand the authoritative world to the net layer at ~snapshot rate.
    if (netRole_ == NetRole::Host && netSession_) {
        LanBuildHostWorld(hostWorld_);
        std::lock_guard lock(hostWorldMutex_);
        hostWorldFresh_ = true;
    }
}

void Game::PublishSnapshot() {
    RaceSnapshot& snap = snapshots_.Back();
    snap.cars.resize(racers_.size());
    snap.phase = phase_;
    snap.raceClock = raceClock_;
    snap.phaseTimer = phaseTimer_;
    snap.statusText = statusText_;
    snap.statusTimer = statusTimer_;
    for (std::size_t i = 0; i < racers_.size(); ++i) {
        Racer& racer = racers_[i];
        CarPose& pose = snap.cars[i];
        pose.chassis = world_ ? world_->GetTransform(racer.vehicle.Chassis())
                                       .value_or(racer.vehicle.State().transform)
                              : racer.vehicle.State().transform;
        pose.state = racer.vehicle.State();
        pose.state.transform = pose.chassis;
        pose.params = racer.vehicle.Params();
        pose.isPlayer = racer.isPlayer;
        pose.tint = racer.isPlayer ? Color{0.88f, 0.92f, 1.0f, 1.f}
                                   : Color{1.0f, 0.86f, 0.74f, 1.f};
        pose.lap = racer.lap;
        pose.position = racer.position;
        pose.lastLapTime = racer.lastLapTime;
        pose.bestLapTime = racer.bestLapTime;
        pose.raceTime = racer.raceTime;
        pose.partHealth = racer.vehicle.Aero().StateRef().partHealth;
        pose.damageTotal = racer.vehicle.Aero().DamageTotal();
    }
    snap.valid = true;
    snap.publishTime = std::chrono::duration<double>(
                           std::chrono::steady_clock::now().time_since_epoch())
                           .count();
    snapshots_.Publish();
}

void Game::SetThreadedSim(bool on) {
    if (on == threadedSim_) return;
    threadedSim_ = on;
    if (!on) StopSimWorker();
}

void Game::StartSimWorker() {
    if (simWorker_.Running()) return;
    simRun_.store(true, std::memory_order_release);
    // Publish an initial snapshot so render has data before the first tick.
    PublishSnapshot();
    simWorker_.Start([this] {
        using clock = std::chrono::steady_clock;
        auto next = clock::now();
        while (simRun_.load(std::memory_order_acquire)) {
            if (!threadedSim_ || phase_ == RacePhase::Menu ||
                phase_ == RacePhase::Settings || phase_ == RacePhase::RaceSetup) {
                break;
            }
            const float frameDt = 1.f / 60.f;
            UpdateSim(frameDt);
            next += std::chrono::microseconds(16667);
            const auto now = clock::now();
            if (next > now) {
                std::this_thread::sleep_until(next);
            } else {
                next = now;  // fell behind — don't spiral
            }
        }
        simRun_.store(false, std::memory_order_release);
    });
}

void Game::StopSimWorker() {
    if (!simWorker_.Running()) return;
    simRun_.store(false, std::memory_order_release);
    simWorker_.Stop();
}

void Game::Render3D() {
    track_.Render(renderer_);

    // While the sim worker owns racers_, draw only the frame snapshot taken
    // in Update() — never a live Front() that Publish can retarget mid-loop.
    // Menus / inline mode: worker is stopped (or never started) and the main
    // thread owns racers_ again.
    if (threadedSim_ && simWorker_.Running()) {
        const double now = std::chrono::duration<double>(
                               std::chrono::steady_clock::now().time_since_epoch())
                               .count();
        // Hold each published pose for a beat, then extrapolate along world
        // velocity so every car advances smoothly between sim ticks. Camera
        // already smooths; without this, all cars step together on each
        // Publish and the pack "vibrates" against the camera (worst at launch).
        const float ahead =
            frameSnap_ && frameSnap_->valid
                ? Clampf(static_cast<float>(now - frameSnap_->publishTime), 0.f, 0.05f)
                : 0.f;
        for (std::size_t i = 0; frameSnap_ && i < frameSnap_->cars.size(); ++i) {
            const CarPose& pose = frameSnap_->cars[i];
            Transform drawT = pose.chassis;
            drawT.position += pose.state.velocity * ahead;
            Vehicle::RenderPose(renderer_, drawT, pose.state, pose.params, pose.tint);
        }
        // LAN client: other players are not simulated locally, so they come
        // straight from the snapshot playout buffer (rendered slightly in the
        // past, which is what N-05 asks for).
        if (netRole_ == NetRole::Client && netInterp_ && !racers_.empty()) {
            const double now2 = std::chrono::duration<double>(
                                    std::chrono::steady_clock::now().time_since_epoch())
                                    .count() * 1000.0;
            const double renderMs = now2 - netInterp_->DelayMs();
            for (int slot = 0; slot < netCarCount_; ++slot) {
                if (slot == static_cast<int>(netLocalSlot_)) continue;
                Transform chassis;
                VehicleState state;
                if (!netInterp_->Sample(static_cast<std::uint8_t>(slot), renderMs, chassis, state)) {
                    continue;
                }
                Color tint{1.0f, 0.86f, 0.74f, 1.f};
                tint.r = 0.55f + 0.12f * static_cast<float>(slot % 3);
                tint.g = 0.75f;
                tint.b = 0.95f - 0.10f * static_cast<float>(slot % 3);
                Vehicle::RenderPose(renderer_, chassis, state, racers_.front().vehicle.Params(), tint);
            }
        }
        return;
    }

    for (Racer& racer : racers_) {
        Color tint = racer.isPlayer ? Color{0.88f, 0.92f, 1.0f, 1.f} : Color{1.0f, 0.86f, 0.74f, 1.f};
        racer.vehicle.Render(renderer_, tint);
    }
}

void Game::UpdatePlayerInput() {
    // Menus own their input (UpdateMenu / UpdateSettings / UpdateRaceSetup);
    // cars stay parked and the sim worker is (or will be) stopped.
    const RacePhase ph = phase_;
    const bool menuPhase = ph == RacePhase::Menu || ph == RacePhase::Settings ||
                           ph == RacePhase::RaceSetup || ph == RacePhase::Lan;

    // Switch the input context before sampling so race-gated events
    // (throttle, DRS, ...) freeze in menus and menu registrations take over.
    input_.SetActiveContext(menuPhase ? "menu" : "race");
    // Inline mode (drive-test / WASM / headless): sample + recognize now;
    // threaded mode pumps on the input thread and only Consume() runs here.
    if (!input_.Running()) input_.PumpOnce();

    rawInput_ = InputState{};
    input_.Consume(rawInput_);
    const InputState& raw = rawInput_;

    if (menuPhase) {
        std::lock_guard lock(intentMutex_);
        intent_ = PlayerIntent{};
        return;
    }

    if (raw.cameraCycle) cameraMode_ = (cameraMode_ + 1) % 4;

    // ESC/P drops back to the menu from any race phase. phase_ is atomic and
    // this is the only main-thread write during a race — the worker observes
    // it and parks without the main thread ever touching racers_ here.
    if (raw.pause) {
        if (netRole_ != NetRole::Offline) {
            // Online races never pause: leaving the room is the only exit.
            LanStop("LEFT ROOM");
            return;
        }
        phase_ = RacePhase::Menu;
        std::lock_guard lock(intentMutex_);
        intent_ = PlayerIntent{};
        return;
    }

    // Everything that reads or writes racers_ (device→car mapping, DRS zone
    // probe, autopilot, restart) runs on the sim thread via this mailbox —
    // the worker may be stepping the player's car right now.
    std::lock_guard lock(intentMutex_);
    intent_.raw = raw;
    intent_.autopilot = config_.GetBool("race", "autopilot", false);
    // Edge requests are set-only here; ApplyPlayerIntent clears them after
    // consumption so one-frame key edges survive any render/sim rate gap.
    if (raw.restart) intent_.restart = true;
    if (raw.drsToggle) intent_.drsToggle = true;
    if (raw.wingModeToggle) intent_.wingToggle = true;
}

void Game::ApplyPlayerIntent() {
    PlayerIntent it;
    {
        std::lock_guard lock(intentMutex_);
        it = intent_;
        intent_.restart = false;
        intent_.drsToggle = false;
        intent_.wingToggle = false;
    }

    if (racers_.empty()) return;
    Racer& player = racers_.front();

    // --- Input mapping (phase as of this sim tick).
    if (phase_ == RacePhase::Racing && !player.finished) {
        const InputState& raw = it.raw;
        // DRS: F only arms/disarms inside a map-marked zone (authored on
        // straights), and only when the season enables DRS. Leaving the
        // zone closes DRS and resets the latch, so re-entering a zone later
        // starts closed — the driver must re-arm. The zone probe needs the
        // car's position, which only the sim thread may read mid-race.
        const float playerD =
            track_.ProjectDistance(player.vehicle.State().transform.position, player.lapDistance);
        const bool inDrsZone = track_.InDrsZone(playerD);
        if (!inDrsZone) {
            drsWanted_ = false;
        } else if (it.drsToggle && seasonDrs_) {
            drsWanted_ = !drsWanted_;
        }
        player.input.throttle = raw.throttle;
        player.input.brake = raw.brake;
        player.input.steer = raw.steer;
        player.input.handbrake = raw.handbrake;
        player.input.reverse = raw.reverse;
        player.input.drsZone = inDrsZone;
        player.input.drsRequest = drsWanted_;
        // 2026 systems are gated by the season — the keys do nothing in
        // classic rules (and the HUD hides their hints).
        player.input.wingToggle = it.wingToggle && seasonAero_;
        player.input.overtake = raw.overtake && seasonOverride_;
    } else if (phase_ == RacePhase::Countdown) {
        player.input = CarInput{};
    } else {
        player.input.throttle = 0.f;
        player.input.brake = 0.8f;
    }

    // ENTER restarts: reset the whole grid back to the countdown.
    if (it.restart) {
        for (Racer& racer : racers_) {
            const Transform pose{track_.StartPosition(racer.isPlayer ? 0 : 1),
                                 track_.StartRotation()};
            racer.vehicle.Reset(pose);
            racer.lap = 0;
            racer.finished = false;
            racer.lapDistance = 0.f;
            racer.currentLapStart = 0.f;
        }
        phase_ = RacePhase::Countdown;
        phaseTimer_ = 3.5f;
        raceClock_ = 0.f;
    }

    // AI 代驾 (autopilot): the player car drives itself — demo mode and the
    // AI regression harness ([race] autopilot=true).
    if (it.autopilot && phase_ == RacePhase::Racing && !player.finished) {
        player.input = CarInput{};
        UpdateAI(player, 0.f);
    }
}

void Game::UpdateAI(Racer& racer, float dt) {
    (void)dt;
    if (phase_ != RacePhase::Racing) {
        racer.input = CarInput{};
        return;
    }
    const VehicleState& state = racer.vehicle.State();
    const float d = track_.ProjectDistance(state.transform.position, racer.lapDistance);

    const Vec3 forward = TransformDirection(state.transform, Vec3{0.f, 0.f, 1.f});
    // Geometric right is -X in the vehicle frame (+Z forward / +Y up, right
    // handed) — see Vehicle::Update. `steer` is positive = steer right.
    const Vec3 right = TransformDirection(state.transform, Vec3{-1.f, 0.f, 0.f});

    // --- Steering: heading error toward a speed-scaled lookahead point,
    // damped by lateral velocity (anti-sideslip). This is what keeps the car
    // off the walls — lateral-offset steering alone cuts corners too hard.
    // The lookahead is short enough to hold a real hairpin line: pure pursuit
    // with a long aim point sweeps wide through tight corners.
    const float lookahead = std::min(10.f + state.speed * 0.32f, 30.f);
    const Vec3 target = track_.PositionAt(d + lookahead);
    const Vec3 toTarget = target - state.transform.position;
    float yawErr = std::atan2(toTarget.x, toTarget.z) - std::atan2(forward.x, forward.z);
    while (yawErr > kPi) yawErr -= 2.f * kPi;
    while (yawErr < -kPi) yawErr += 2.f * kPi;
    const float lateralVel = Dot(state.velocity, right);
    // Modest gain + strong anti-sideslip damping: high gain + weak damping
    // makes the steering loop oscillate at speed and spin the car.
    float steer = Clampf(-yawErr * 1.5f - lateralVel * 0.06f, -1.f, 1.f);

    // --- Speed target from the sharpest corner inside the braking horizon.
    // The grip budget is derived from the car's own tire / aero calibration:
    // max lateral a(v) = aLatBase + dfPerV2*v^2 must cover the corner demand
    // v^2*curv (curvMax carries a 1.6x margin — slaloms punish optimistic
    // entries through the direction changes). The vehicle's steering
    // envelope is 1.2x the same grip curve (see Vehicle::SteerEnvelopeDeg),
    // so one constraint covers both.
    const float horizon = std::max(60.f, state.speed * state.speed / 24.f);
    float curvMax = 0.f;
    for (float dd = 0.f; dd <= horizon; dd += 10.f) {
        curvMax = std::max(curvMax, track_.CurvatureAt(d + dd) * 1.6f);
    }
    curvMax = std::max(curvMax, 0.0012f);
    const VehicleParams& p = racer.vehicle.Params();
    const float gripRef = 0.5f * (p.frontTire.D0 + p.rearTire.D0);
    const float aLatBase = 9.81f * gripRef * 0.92f;
    const float dfPerV2 = 0.5f * p.aero.airDensity * p.aero.downforceArea / p.mass;
    const float gripDenom = curvMax - dfPerV2;
    float vTarget = gripDenom > 1e-4f ? std::sqrt(aLatBase / gripDenom) : 62.f;
    // 0.90 safety factor: the rigid driveline couples the brakes to the crank
    // inertia, so real decel is ~77% of the old free-wheeling calibration —
    // the entry budget must carry that margin or corner entries run wide.
    vTarget = Clampf(vTarget * 0.90f, 12.f, 62.f);

    // Traffic awareness: full track projection per rival (original). The
    // O(AI^2 * samples) cost stays — rivals are few and the job is parallel.
    const float total = track_.TotalLength();
    for (const Racer& other : racers_) {
        if (&other == &racer) continue;
        const float od = track_.ProjectDistance(other.vehicle.State().transform.position,
                                                other.lapDistance);
        float gap = od - d;
        if (gap < -total * 0.5f) gap += total;
        else if (gap > total * 0.5f) gap -= total;
        if (gap > 0.f && gap < 18.f) {
            // Floor at 8 m/s so close following never freezes into an
            // accordion; light contact is harmless in no-damage mode.
            vTarget = std::max(8.f, std::min(vTarget, other.vehicle.State().speed + 2.5f +
                                                          gap * 0.5f));
        }
    }

    float throttle = 1.f;
    float brake = 0.f;
    if (state.speed < 8.f) {
        throttle = 1.f;
    } else if (state.speed > vTarget + 1.5f) {
        throttle = 0.f;
        brake = Clampf((state.speed - vTarget) * 0.14f, 0.15f, 1.f);
    } else if (state.speed > vTarget - 2.f) {
        throttle = 0.55f;
    }

    // Slide recovery: ease off and counter a bit when the rear steps out.
    const float slide =
        std::fabs(state.wheels[2].slipAngleDeg) + std::fabs(state.wheels[3].slipAngleDeg);
    if (slide > 28.f) {
        throttle *= 0.55f;
        steer = Clampf(steer * 1.25f, -1.f, 1.f);
    }

    // Spin recovery: when facing away from the track, coast and unwind the
    // steering toward the target — flooring it while sideways only pins the
    // car against a wall.
    if (Dot(forward, track_.ForwardAt(d)) < 0.2f) {
        throttle = 0.f;
        brake = state.speed > 8.f ? 0.35f : 0.f;
    }

    racer.input.steer = steer;
    racer.input.throttle = throttle;
    racer.input.brake = brake;

    // Use DRS/overtake features like the player — but only what the season
    // enables (the vehicle gates activation on the same flags).
    racer.input.drsZone = track_.InDrsZone(d);
    racer.input.drsRequest = seasonDrs_;
    racer.input.overtake =
        seasonOverride_ && curvMax < 0.002f && state.speed > 50.f;
    racer.input.wingToggle = false;
}

void Game::StepPhysics(float dt) {
    const bool b = benchOn_;
    if (b) ++bench_.physSteps;
    ++hostTick_;

    BenchClock::time_point t = b ? BenchClock::now() : BenchClock::time_point{};
    // Vehicle::Update raycasts and applies forces on the shared IPhysicsWorld
    // — backends are not thread-safe, so this batch stays serial. Parallel
    // vehicle substeps are an L2 concern only (never inside rpc*/qu3e/nudge).
    // AI above is pure state→input and is safe to split.
    // Aero areas are computed inside Vehicle's AeroKit now (damage / DRS /
    // wing mode / pitch); the engine applies the forces each substep.
    for (Racer& racer : racers_) {
        racer.vehicle.Update(racer.input, dt);
    }
    if (b) {
        bench_.vehicleSec += BenchSec(t, BenchClock::now());
        t = BenchClock::now();
    }

    // Step only publishes CollisionEvents onto the bus; damage/audio drain
    // in ProcessCollisionEvents (separate stage — physics never calls game).
    world_->Step(dt);
    if (b) {
        const double sec = BenchSec(t, BenchClock::now());
        bench_.stepSec += sec;
        bench_.stepTimesMs.push_back(sec * 1e3);
    }
}

void Game::ProcessCollisionEvents(float dt) {
    const bool b = benchOn_;
    BenchClock::time_point t = b ? BenchClock::now() : BenchClock::time_point{};

    const std::vector<ContactEvent> contacts = world_->DrainContacts();
    for (Racer& racer : racers_) {
        racer.vehicle.ProcessContacts(contacts, dt);
    }

    // Collision thump for the player's car.
    if (audio_ && !racers_.empty()) {
        const BodyHandle playerBody = racers_.front().vehicle.Chassis();
        for (const ContactEvent& event : contacts) {
            if (event.bodyA != playerBody && event.bodyB != playerBody) continue;
            if (event.impactSpeed > 1.5f) {
                audio_->PlayImpact(Clampf((event.impactSpeed - 1.5f) / 10.f, 0.f, 1.f));
            }
        }
    }
    if (b) bench_.contactSec += BenchSec(t, BenchClock::now());
}

void Game::UpdateRace(float dt) {
    if (phase_ == RacePhase::Countdown) {
        phaseTimer_ -= dt;
        if (phaseTimer_ <= 0.f) {
            phase_ = RacePhase::Racing;
            raceClock_ = 0.f;
            for (Racer& racer : racers_) racer.currentLapStart = 0.f;
        }
        return;
    }
    if (phase_ == RacePhase::Finished) return;

    raceClock_ += dt;

    for (Racer& racer : racers_) {
        const VehicleState& state = racer.vehicle.State();
        const float d = track_.ProjectDistance(state.transform.position, racer.lapDistance);

        // Lap crossing: distance wrapped past the line.
        const float prev = racer.lapDistance;
        if (prev > track_.TotalLength() * 0.75f && d < track_.TotalLength() * 0.25f) {
            ++racer.lap;
            if (racer.lap > 0) {
                const float lapTime = raceClock_ - racer.currentLapStart;
                racer.lastLapTime = lapTime;
                if (racer.bestLapTime <= 0.f || lapTime < racer.bestLapTime) {
                    racer.bestLapTime = lapTime;
                }
                racer.currentLapStart = raceClock_;
                if (netRole_ == NetRole::Host && netSession_) {
                    net::LapCompletePacket lapEvent;
                    lapEvent.slot = static_cast<std::uint8_t>(&racer - racers_.data());
                    lapEvent.lap = static_cast<std::uint8_t>(std::min(racer.lap, 255));
                    lapEvent.lapTimeMs = static_cast<std::uint32_t>(lapTime * 1000.f);
                    lapEvent.bestLapMs = static_cast<std::uint32_t>(racer.bestLapTime * 1000.f);
                    netSession_->SendLapComplete(lapEvent);
                }
            }
            if (racer.lap >= totalLaps_) {
                racer.finished = true;
                racer.raceTime = raceClock_;
                if (netRole_ == NetRole::Host && netSession_) {
                    net::PlayerFinishedPacket fin;
                    fin.slot = static_cast<std::uint8_t>(&racer - racers_.data());
                    fin.position = static_cast<std::uint8_t>(std::clamp(racer.position, 1, 255));
                    fin.raceTimeMs = static_cast<std::uint32_t>(racer.raceTime * 1000.f);
                    fin.bestLapMs = static_cast<std::uint32_t>(racer.bestLapTime * 1000.f);
                    netSession_->SendPlayerFinished(fin);
                }
            }
        }
        racer.lapDistance = d;
        racer.progress = static_cast<float>(racer.lap) + d / std::max(1.f, track_.TotalLength());

        // Off-track / flipped safety reset (simcade recovery). A car lying on
        // its roof or side has no wheel contact and would otherwise be stuck;
        // a car pinned against a wall with wheels spinning gets the same
        // treatment after a few seconds.
        const Vec3 carUp = TransformDirection(state.transform, Vec3{0.f, 1.f, 0.f});
        const bool fellOff = state.transform.position.y < -8.f;
        const bool stranded = carUp.y < 0.25f && state.speed < 3.f;
        const bool pinned = state.speed < 2.5f && racer.input.throttle > 0.5f;
        racer.stuckTimer = pinned ? racer.stuckTimer + dt : 0.f;
        if (fellOff || stranded || racer.stuckTimer > 4.f) {
            const bool wasPinned = racer.stuckTimer > 4.f;
            racer.stuckTimer = 0.f;
            // A wall pin usually leaves a pile of cars — respawn a little
            // ahead on the centerline so the pack clears itself.
            const float dd = d + (wasPinned ? 12.f : 0.f);
            const Vec3 fwd = track_.ForwardAt(dd);
            Transform pose{track_.PositionAt(dd) + Vec3{0.f, 1.f, 0.f},
                           Quat::FromAxisAngle(Vec3{0.f, 1.f, 0.f}, std::atan2(fwd.x, fwd.z))};
            racer.vehicle.Reset(pose);
            statusText_ = racer.name + " reset";
            statusTimer_ = 2.f;
        }
    }

    // Ranking.
    std::vector<size_t> order(racers_.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return racers_[a].progress > racers_[b].progress;
    });
    for (size_t rank = 0; rank < order.size(); ++rank) {
        racers_[order[rank]].position = static_cast<int>(rank) + 1;
    }

    // Race end: offline ends when the player finishes; a LAN host waits until
    // every car is home (or ends it manually).
    bool allDone = true;
    for (const Racer& r : racers_) allDone = allDone && r.finished;
    if ((netRole_ == NetRole::Host) ? allDone : racers_.front().finished) {
        phase_ = RacePhase::Finished;
        if (netRole_ == NetRole::Host && netSession_ && !netEndSent_) {
            netEndSent_ = true;
            std::vector<net::RaceEndEntry> results;
            for (std::size_t i = 0; i < racers_.size(); ++i) {
                const Racer& r = racers_[i];
                net::RaceEndEntry entry;
                entry.slot = static_cast<std::uint8_t>(i);
                entry.position = static_cast<std::uint8_t>(std::clamp(r.position, 1, 255));
                entry.finished = r.finished ? 1 : 0;
                entry.raceTimeMs = static_cast<std::uint32_t>(std::max(0.f, r.raceTime) * 1000.f);
                entry.bestLapMs = static_cast<std::uint32_t>(std::max(0.f, r.bestLapTime) * 1000.f);
                results.push_back(entry);
            }
            netSession_->EndRace(std::move(results));
        }
    }
}

void Game::UpdateCamera(float dt) {
    if (racers_.empty()) return;
    UpdateCameraFrom(racers_.front().vehicle.State(), dt);
}

void Game::UpdateCameraFrom(const VehicleState& state, float dt) {
    const Vec3 forward = TransformDirection(state.transform, Vec3{0.f, 0.f, 1.f});
    const Vec3 up{0.f, 1.f, 0.f};

    // Menus get a slow orbit around the parked car.
    if (phase_ == RacePhase::Menu || phase_ == RacePhase::Settings ||
        phase_ == RacePhase::RaceSetup) {
        const float angle = static_cast<float>(renderer_.TimeSeconds()) * 0.18f;
        const Vec3 desiredPos = state.transform.position +
                                Vec3{std::cos(angle) * 13.f, 5.f, std::sin(angle) * 13.f};
        const Vec3 desiredTarget = state.transform.position + Vec3{0.f, 0.8f, 0.f};
        const float blend = Clampf(dt * 3.f, 0.f, 1.f);
        cameraPos_ += (desiredPos - cameraPos_) * blend;
        cameraTarget_ += (desiredTarget - cameraTarget_) * blend;
        CameraDesc cam;
        cam.position = cameraPos_;
        cam.target = cameraTarget_;
        cam.up = up;
        cam.fovYDeg = 50.f;
        renderer_.SetCamera(cam);
        return;
    }

    Vec3 desiredPos;
    Vec3 desiredTarget;
    switch (cameraMode_) {
        case 0:  // chase — aim above the car so it sits below screen centre
            desiredPos = state.transform.position - forward * 9.5f + up * 3.4f;
            desiredTarget = state.transform.position + forward * 6.f + up * 1.8f;
            break;
        case 1:  // far chase
            desiredPos = state.transform.position - forward * 15.f + up * 6.5f;
            desiredTarget = state.transform.position + forward * 4.f + up * 2.6f;
            break;
        case 2:  // hood
            desiredPos = state.transform.position + forward * 1.2f + up * 1.1f;
            desiredTarget = desiredPos + forward * 20.f;
            break;
        default:  // orbit
        {
            const float angle = static_cast<float>(renderer_.TimeSeconds()) * 0.35f;
            desiredPos = state.transform.position +
                         Vec3{std::cos(angle) * 12.f, 5.5f, std::sin(angle) * 12.f};
            desiredTarget = state.transform.position;
            break;
        }
    }

    // Tight follow on the look target (a lagged target drops the car toward
    // the top of the frame while accelerating), looser on the camera itself.
    const float blend = (cameraSmooth_ <= 0.f) ? 1.f : Clampf(dt * 6.f, 0.f, 1.f);
    const float targetBlend = (cameraSmooth_ <= 0.f) ? 1.f : Clampf(dt * 12.f, 0.f, 1.f);
    cameraSmooth_ = 1.f;
    cameraPos_ += (desiredPos - cameraPos_) * blend;
    cameraTarget_ += (desiredTarget - cameraTarget_) * targetBlend;

    CameraDesc cam;
    cam.position = cameraPos_;
    cam.target = cameraTarget_;
    cam.up = up;
    // Narrow vertical FOV: the old 62+speed*0.12 fisheyed to 72 deg at racing
    // speed, shrinking the road ahead into the distance. Keep it tight and let
    // speed widen it only slightly so the sense of speed survives.
    cam.fovYDeg = 50.f + Clampf(state.speed * 0.03f, 0.f, 2.f);
    renderer_.SetCamera(cam);
}

void Game::RenderHUD() {
    if (phase_ == RacePhase::Menu) {
        RenderMenu();
        return;
    }
    if (phase_ == RacePhase::Settings) {
        RenderSettings();
        return;
    }
    if (phase_ == RacePhase::RaceSetup) {
        RenderRaceSetup();
        return;
    }
    if (phase_ == RacePhase::Lan) {
        RenderLan();
        return;
    }
    if (racers_.empty()) return;

    // Race HUD fields: while the sim worker owns racers_, every sim-written
    // field (position, lap times, clocks, status line) comes from this
    // frame's snapshot taken in Update(); inline mode reads the live
    // members on this same thread.
    const bool snapHud = threadedSim_ && simWorker_.Running() && frameSnap_ &&
                         frameSnap_->valid && !frameSnap_->cars.empty();
    const VehicleState& state =
        snapHud ? frameSnap_->cars[0].state : racers_.front().vehicle.State();
    const std::array<float, 4>& partHealth =
        snapHud ? frameSnap_->cars[0].partHealth
                : racers_.front().vehicle.Aero().StateRef().partHealth;
    // Fitted features come from the car, not from the season globals: a car
    // without a system shows no readout for it (see VehicleCapability).
    const std::uint8_t capabilities = snapHud ? frameSnap_->cars[0].params.capabilities
                                              : racers_.front().vehicle.Params().capabilities;
    const int position = (netRole_ == NetRole::Client && netReadout_.valid)
                             ? static_cast<int>(netReadout_.position)
                             : (snapHud ? frameSnap_->cars[0].position : racers_.front().position);
    const int lap = (netRole_ == NetRole::Client && netReadout_.valid)
                        ? static_cast<int>(netReadout_.lap)
                        : (snapHud ? frameSnap_->cars[0].lap : racers_.front().lap);
    const float lastLapTime =
        snapHud ? frameSnap_->cars[0].lastLapTime : racers_.front().lastLapTime;
    const float bestLapTime =
        snapHud ? frameSnap_->cars[0].bestLapTime : racers_.front().bestLapTime;
    const float raceTime = snapHud ? frameSnap_->cars[0].raceTime : racers_.front().raceTime;
    const float raceClock = snapHud ? frameSnap_->raceClock : raceClock_;
    const float phaseTimer = snapHud ? frameSnap_->phaseTimer : phaseTimer_;
    const float statusTimer = snapHud ? frameSnap_->statusTimer : statusTimer_;
    const std::string& statusText = snapHud ? frameSnap_->statusText : statusText_;
    const int carCount =
        snapHud ? static_cast<int>(frameSnap_->cars.size()) : static_cast<int>(racers_.size());

    const float w = static_cast<float>(renderer_.ScreenWidth());
    const float h = static_cast<float>(renderer_.ScreenHeight());

    // Top-left: position + lap.
    char buffer[128];
    std::snprintf(buffer, sizeof(buffer), "P%d/%d   LAP %d/%d", position, carCount,
                  std::min(lap + 1, totalLaps_), totalLaps_);
    renderer_.DrawRect2D(10.f, 10.f, 340.f, 92.f, Color{0.f, 0.f, 0.f, 0.45f});
    renderer_.DrawText2D(buffer, 22.f, 18.f, 22, Color::White());
    renderer_.DrawText2D("TIME  " + FormatTime(raceClock), 22.f, 46.f, 18, Color::Gray());
    renderer_.DrawText2D("LAST  " + FormatTime(lastLapTime) + "   BEST " +
                             FormatTime(bestLapTime),
                         22.f, 70.f, 18, Color::Yellow());

    // Bottom-right: speed + gear + rpm bar.
    std::snprintf(buffer, sizeof(buffer), "%3.0f KM/H", state.speedKmh);
    renderer_.DrawText2D(buffer, w - 260.f, h - 120.f, 42, Color::White());
    std::snprintf(buffer, sizeof(buffer), "GEAR %s",
                  state.gear == 0 ? "R" : std::to_string(state.gear).c_str());
    renderer_.DrawText2D(buffer, w - 260.f, h - 72.f, 26, Color::Orange());

    const float rpmFrac = Clampf(state.rpm / 12500.f, 0.f, 1.f);
    renderer_.DrawRect2D(w - 260.f, h - 38.f, 240.f, 18.f, Color{0.15f, 0.15f, 0.18f, 0.9f});
    renderer_.DrawRect2D(w - 258.f, h - 36.f, 236.f * rpmFrac, 14.f,
                         rpmFrac > 0.85f ? Color::Red() : Color::Green());

    // Bottom-left: tire wear + damage + ERS.
    const float baseY = h - 150.f;
    renderer_.DrawRect2D(10.f, baseY, 300.f, 140.f, Color{0.f, 0.f, 0.f, 0.45f});
    renderer_.DrawText2D("TIRE WEAR", 20.f, baseY + 8.f, 16, Color::Gray());
    for (int i = 0; i < 4; ++i) {
        const float wear = state.wheels[i].wear;
        const float x = 20.f + static_cast<float>(i) * 68.f;
        renderer_.DrawRect2D(x, baseY + 32.f, 60.f, 12.f, Color{0.15f, 0.15f, 0.18f, 0.9f});
        renderer_.DrawRect2D(x + 2.f, baseY + 34.f, 56.f * (1.f - wear), 8.f,
                             wear > 0.6f ? Color::Red() : (wear > 0.3f ? Color::Yellow() : Color::Green()));
    }
    renderer_.DrawText2D("DAMAGE", 20.f, baseY + 56.f, 16, Color::Gray());
    static const char* kPartNames[4] = {"FW", "RW", "SP", "FL"};
    for (int i = 0; i < 4; ++i) {
        const float health = partHealth[i];
        const float x = 20.f + static_cast<float>(i) * 68.f;
        renderer_.DrawRect2D(x, baseY + 80.f, 60.f, 12.f, Color{0.15f, 0.15f, 0.18f, 0.9f});
        renderer_.DrawRect2D(x + 2.f, baseY + 82.f, 56.f * health, 8.f,
                             health > 0.7f ? Color::Green() : (health > 0.35f ? Color::Yellow() : Color::Red()));
        renderer_.DrawText2D(kPartNames[i], x + 18.f, baseY + 98.f, 14, Color::Gray());
    }

    // ERS bar + DRS/wing indicators.
    if (HasCapability(capabilities, VehicleCapability::Ers)) {
        renderer_.DrawText2D("ERS", 20.f, baseY + 118.f, 16, Color::Gray());
        renderer_.DrawRect2D(60.f, baseY + 118.f, 140.f, 12.f, Color{0.15f, 0.15f, 0.18f, 0.9f});
        renderer_.DrawRect2D(62.f, baseY + 120.f, 136.f * state.ersCharge, 8.f,
                             state.ersDeploying ? Color::Yellow() : Color::Blue());
    }

    // DRS / wing indicators — only for the features this season enables
    // (classic shows DRS, 2026 shows the X/Z modes; never both).
    if (seasonDrs_) {
        if (state.drsActive) {
            renderer_.DrawText2D("DRS", w - 120.f, 30.f, 30, Color::Green());
        } else if (state.drsAvailable) {
            renderer_.DrawText2D("DRS RDY", w - 170.f, 30.f, 22, Color::Yellow());
        }
    }
    if (seasonAero_) {
        renderer_.DrawText2D(state.wingMode == WingMode::XMode ? "X-MODE" : "Z-MODE",
                             w - 150.f, 64.f, 20,
                             state.wingMode == WingMode::XMode ? Color::Blue() : Color::Orange());
    }

    // Phase overlays.
    if (phase_ == RacePhase::Countdown) {
        const int count = std::min(3, static_cast<int>(std::ceil(phaseTimer - 0.5f)));
        renderer_.DrawRect2D(w * 0.5f - 70.f, h * 0.5f - 70.f, 140.f, 140.f, Color{0.f, 0.f, 0.f, 0.5f});
        if (count > 0) {
            std::snprintf(buffer, sizeof(buffer), "%d", count);
            renderer_.DrawText2D(buffer, w * 0.5f - 30.f, h * 0.5f - 40.f, 64, Color::White());
        } else {
            renderer_.DrawText2D("GO!", w * 0.5f - 55.f, h * 0.5f - 40.f, 64, Color::Green());
        }
    } else if (phase_ == RacePhase::Finished) {
        renderer_.DrawRect2D(w * 0.5f - 220.f, h * 0.5f - 90.f, 440.f, 180.f, Color{0.f, 0.f, 0.f, 0.6f});
        renderer_.DrawText2D("RACE FINISHED", w * 0.5f - 150.f, h * 0.5f - 60.f, 36, Color::White());
        std::snprintf(buffer, sizeof(buffer), "P%d   TIME %s   BEST %s", position,
                      FormatTime(raceTime > 0.f ? raceTime : raceClock).c_str(),
                      FormatTime(bestLapTime).c_str());
        renderer_.DrawText2D(buffer, w * 0.5f - 190.f, h * 0.5f - 8.f, 22, Color::Yellow());
        renderer_.DrawText2D("ENTER = RESTART    ESC = MENU", w * 0.5f - 190.f, h * 0.5f + 40.f, 20,
                             Color::Gray());
    }

    if (statusTimer > 0.f && !statusText.empty()) {
        renderer_.DrawText2D(statusText, w * 0.5f - 80.f, h - 200.f, 22, Color::Orange());
    }

    // Help line lists only the keys the car actually wires up.
    std::string help = "W/S THROTTLE-BRAKE   A/D STEER";
    if (HasCapability(capabilities, VehicleCapability::Drs)) help += "   F DRS";
    if (HasCapability(capabilities, VehicleCapability::ActiveAero)) help += "   V WING";
    if (HasCapability(capabilities, VehicleCapability::Overtake)) help += "   SHIFT OVERTAKE";
    help += "   C CAMERA   ESC MENU";
    renderer_.DrawText2D(help, 12.f, h - 22.f, 15, Color::DarkGray());
}

// ---------------------------------------------------------------- screens ---
//
// Every menu below is data for rush.engine.ui: the game names the rows and
// their ids, the engine owns focus, digit/text editing and drawing, and the
// dispatchers react to the UiAction that comes back (by id, never by row
// index — a dynamic list makes indices move).

namespace {

// The engine keeps cursor in range, but a discovered list can shrink under it.
const UiWidget* FocusedRow(const UiScreen& screen) {
    const int c = screen.cursor;
    if (c < 0 || c >= static_cast<int>(screen.widgets.size())) return nullptr;
    return &screen.widgets[static_cast<std::size_t>(c)];
}

}  // namespace

void Game::SyncUiPhase() {
    const RacePhase ph = phase_;
    if (ph == uiPhase_) return;
    uiPhase_ = ph;
    OpenPhaseScreen(ph);
}

void Game::OpenPhaseScreen(RacePhase phase) {
    switch (phase) {
        case RacePhase::Menu:
            BuildMenuScreen();
            UiController::ResetFocus(screens_.menu);
            break;
        case RacePhase::Settings:
            BuildSettingsScreen();
            UiController::ResetFocus(screens_.settings);
            break;
        case RacePhase::RaceSetup:
            BuildRaceSetupScreen();
            UiController::ResetFocus(screens_.raceSetup);
            break;
        case RacePhase::Lan:
            BuildLanScreens();
            lanScreensStale_ = true;
            break;
        default:
            break;   // race phases draw the HUD, not a control screen
    }
}

// ------------------------------------------------------------ main menu -----

void Game::BuildMenuScreen() {
    if (!screens_.menu.widgets.empty()) return;
    const float w = static_cast<float>(renderer_.ScreenWidth());
    const float h = static_cast<float>(renderer_.ScreenHeight());
    UiScreen& s = screens_.menu;
    s.title = "APEX RUSH";
    s.titleSize = 68;
    s.x = w * 0.5f - 165.f;
    s.y = h * 0.24f;
    s.rowStartY = h * 0.40f;
    s.rowHeight = 54.f;
    s.hint = "W/S SELECT    ENTER CONFIRM";
    // Row order is asserted by --drive-test: 0 race, 1 settings, quit last.
    s.widgets = {
        {.kind = UiKind::Label, .label = "OPEN F1 SIM", .size = 20},
        {.kind = UiKind::Button, .id = "start", .label = "START RACE", .size = 28},
        {.kind = UiKind::Button, .id = "settings", .label = "SETTINGS", .size = 28},
        {.kind = UiKind::Button, .id = "lan", .label = "LAN RACE", .size = 28},
        {.kind = UiKind::Button, .id = "quit", .label = "QUIT", .size = 28},
    };
}

void Game::UpdateMenu() {
    if (audio_ && (rawInput_.menuUp || rawInput_.menuDown || rawInput_.menuSelect)) {
        audio_->PlayUIClick();
    }
    BuildMenuScreen();
    const UiAction action = ui_.Update(screens_.menu, rawInput_);
    if (action.kind != UiActionKind::Activate) return;
    if (action.id == "start") {
        // START RACE opens the pre-race setup, not the race itself.
        phase_ = RacePhase::RaceSetup;
    } else if (action.id == "settings") {
        phase_ = RacePhase::Settings;
    } else if (action.id == "lan") {
        EnterLan();
    } else if (action.id == "quit") {
        wantsQuit_ = true;
    }
    // ESC is inert here on purpose: quitting must go through the QUIT item
    // only. Acting on menuBack also ate the same keypress that just brought
    // the player back from a race ("ESC exits the app instead of the menu").
}

void Game::RenderMenu() {
    BuildMenuScreen();
    ui_.Draw(screens_.menu, renderer_);
}

// -------------------------------------------------------------- settings ----

void Game::BuildSettingsScreen() {
    if (!screens_.settings.widgets.empty()) return;
    const float w = static_cast<float>(renderer_.ScreenWidth());
    const float h = static_cast<float>(renderer_.ScreenHeight());
    UiScreen& s = screens_.settings;
    // The row height shrinks so the list fits 720p.
    const float top = h * 0.17f;
    const float rowH = std::clamp((h - top - 90.f) / kSettingsRowCount, 26.f, 46.f);
    const int fontSize = rowH > 38.f ? 22 : 17;
    s.title = "SETTINGS";
    s.titleSize = 42;
    s.x = w * 0.5f - 280.f;   // one column for the title and the rows
    s.y = h * 0.08f;
    s.rowStartY = top;
    s.rowHeight = rowH;
    s.valueColumnX = 560.f;
    s.hint = "A/D CHANGE    W/S SELECT    ENTER/ESC BACK";

    struct RowDef {
        const char* id;
        const char* label;
    };
    static constexpr RowDef kDefs[kSettingsRowCount] = {
        {"season", "SEASON"},
        {"drs", "DRS"},
        {"aero", "ACTIVE AERO"},
        {"overtake", "OVERTAKE"},
        {"abs", "ABS"},
        {"tc", "TRACTION CTRL"},
        {"esc", "STABILITY CTRL"},
        {"steer", "STEER ASSIST"},
        {"display", "DISPLAY MODE"},
        {"resolution", "RESOLUTION"},
        {"quality", "QUALITY"},
        {"fps", "FPS LIMIT"},
        {"physics_hz", "PHYSICS RATE"},
        {"physics_engine", "PHYSICS ENGINE"},
        {"volume", "SOUND VOLUME"},
        {"audio_backend", "AUDIO BACKEND"},
        {"back", "BACK"},
    };
    s.widgets.reserve(kSettingsRowCount);
    for (int i = 0; i < kSettingsRowCount; ++i) {
        UiWidget& row = s.widgets.emplace_back();
        row.size = fontSize;
        row.id = kDefs[i].id;
        row.label = kDefs[i].label;
        row.kind = (i == kSettingsRowCount - 1) ? UiKind::Button : UiKind::Value;
    }
}

void Game::RefreshSettingsValues() {
    const int physicsHz = static_cast<int>(1.f / fixedDt_ + 0.5f);
    char resBuffer[32];
    std::snprintf(resBuffer, sizeof(resBuffer), "%dx%d", video_.width, video_.height);
    // Season preset display: named presets, or CUSTOM once a feature gate was
    // flipped individually.
    const std::string preset = config_.GetString("season", "preset", "2025");
    const std::string seasonName = preset == "2026"   ? "2026"
                                   : preset == "2025" ? "2025"
                                                      : "CUSTOM";
    const std::string values[kSettingsRowCount] = {
        seasonName,
        seasonDrs_ ? "ON" : "OFF",
        seasonAero_ ? "ON" : "OFF",
        seasonOverride_ ? "ON" : "OFF",
        AssistLevelName(ParseAssistLevel(config_, "abs", DefaultAssistLevel("abs"))),
        AssistLevelName(ParseAssistLevel(config_, "tc", DefaultAssistLevel("tc"))),
        AssistLevelName(ParseAssistLevel(config_, "esc", DefaultAssistLevel("esc"))),
        AssistLevelName(ParseAssistLevel(config_, "steer", DefaultAssistLevel("steer"))),
        video_.borderlessFullscreen ? "BORDERLESS" : "WINDOWED",
        resBuffer,
        video_.shadows ? "HIGH" : "LOW",
        video_.fpsLimit > 0 ? std::to_string(video_.fpsLimit) : "UNCAPPED",
        std::to_string(physicsHz) + " HZ",
        MapDisplayName(config_.GetString("physics", "engine", "qu3e")),
        std::to_string(config_.GetInt("audio", "volume", 80)),
        MapDisplayName(config_.GetString("audio", "backend", "auto")),
        "",
    };
    for (int i = 0; i < kSettingsRowCount; ++i) {
        screens_.settings.widgets[static_cast<std::size_t>(i)].value = values[i];
    }
}

void Game::UpdateSettings() {
    if (audio_ && (rawInput_.menuUp || rawInput_.menuDown || rawInput_.menuLeft ||
                   rawInput_.menuRight || rawInput_.menuSelect)) {
        audio_->PlayUIClick();
    }
    BuildSettingsScreen();
    const UiAction action = ui_.Update(screens_.settings, rawInput_);

    const auto backToMenu = [this]() {
        config_.Save(DataPath("config.rushconf"));
        phase_ = RacePhase::Menu;
    };
    if (action.kind == UiActionKind::Back) {
        backToMenu();
        return;
    }
    if (action.kind == UiActionKind::Activate) {
        if (action.id == "back") backToMenu();
        return;
    }

    std::string id = action.id;
    int step = 0;
    if (action.kind == UiActionKind::AdjustLeft) {
        step = -1;
    } else if (action.kind == UiActionKind::AdjustRight) {
        step = 1;
    } else if (action.kind == UiActionKind::None && rawInput_.menuSelect) {
        // Enter has always flipped the focused row in place. The engine only
        // reports Left/Right on Value rows, so an unused Enter is passed on as
        // a zero step, which the row handlers read as "toggle".
        const UiWidget* row = FocusedRow(screens_.settings);
        if (row == nullptr || row->kind != UiKind::Value) return;
        id = row->id;
    } else {
        return;
    }
    ApplySettingStep(id, step);
    RefreshSettingsValues();
}

void Game::ApplySettingStep(const std::string& id, int step) {
    if (id == "season") {
        // SEASON preset: classic (DRS) <-> 2026 (aero + override).
        // Writing a preset rewrites the three feature toggles with it.
        const std::string preset = config_.GetString("season", "preset", "2025");
        int idx = (preset == "2026") ? 1 : 0;  // custom -> classic
        idx = CyclePreset(idx, 2, step);
        const bool is2026 = (idx == 1);
        config_.SetString("season", "preset", is2026 ? "2026" : "2025");
        config_.SetString("season", "drs", is2026 ? "0" : "1");
        config_.SetString("season", "active_aero", is2026 ? "1" : "0");
        config_.SetString("season", "overtake", is2026 ? "1" : "0");
        seasonDrs_ = !is2026;
        seasonAero_ = is2026;
        seasonOverride_ = is2026;
        return;
    }
    if (id == "drs" || id == "aero" || id == "overtake") {
        // Per-feature toggles. Flipping one marks the preset CUSTOM — the
        // three systems may then mix, but only deliberately.
        static const char* kSeasonKeys[3] = {"drs", "active_aero", "overtake"};
        bool* gates[3] = {&seasonDrs_, &seasonAero_, &seasonOverride_};
        const int i = (id == "drs") ? 0 : (id == "aero" ? 1 : 2);
        const bool value = (step != 0) ? (step > 0) : !*gates[i];
        *gates[i] = value;
        config_.SetString("season", kSeasonKeys[i], value ? "1" : "0");
        config_.SetString("season", "preset", "custom");
        return;
    }
    if (id == "abs" || id == "tc" || id == "esc" || id == "steer") {
        const char* key = id.c_str();
        int level = static_cast<int>(ParseAssistLevel(config_, key, DefaultAssistLevel(key)));
        level = CyclePreset(level, 3, step);
        SaveAssistLevel(config_, key, static_cast<AssistLevel>(level));
        return;
    }
    if (id == "display") {
        video_.borderlessFullscreen = (step != 0) ? (step > 0) : !video_.borderlessFullscreen;
        config_.SetString("video", "mode", video_.borderlessFullscreen ? "borderless" : "windowed");
        renderer_.ApplyVideo(video_);
        return;
    }
    if (id == "resolution") {
        int idx = 0;
        for (int i = 0; i < kResPresetCount; ++i) {
            if (kResPresets[i].width == video_.width && kResPresets[i].height == video_.height) {
                idx = i;
            }
        }
        idx = CyclePreset(idx, kResPresetCount, step);
        video_.width = kResPresets[idx].width;
        video_.height = kResPresets[idx].height;
        config_.SetString("video", "width", std::to_string(video_.width));
        config_.SetString("video", "height", std::to_string(video_.height));
        renderer_.ApplyVideo(video_);
        return;
    }
    if (id == "quality") {
        video_.shadows = (step != 0) ? (step > 0) : !video_.shadows;
        config_.SetString("video", "quality", video_.shadows ? "high" : "low");
        renderer_.ApplyVideo(video_);
        return;
    }
    if (id == "fps") {
        int idx = 0;
        for (int i = 0; i < kFpsPresetCount; ++i) {
            if (kFpsPresets[i] == video_.fpsLimit) idx = i;
        }
        idx = CyclePreset(idx, kFpsPresetCount, step);
        video_.fpsLimit = kFpsPresets[idx];
        config_.SetString("video", "fps_limit", std::to_string(video_.fpsLimit));
        renderer_.ApplyVideo(video_);
        return;
    }
    if (id == "physics_hz") {
        const int hz = static_cast<int>(1.f / fixedDt_ + 0.5f);
        int idx = 1;
        for (int i = 0; i < kPhysicsPresetCount; ++i) {
            if (kPhysicsPresets[i] == hz) idx = i;
        }
        idx = CyclePreset(idx, kPhysicsPresetCount, step);
        fixedDt_ = 1.f / static_cast<float>(kPhysicsPresets[idx]);
        config_.SetString("physics", "hz", std::to_string(kPhysicsPresets[idx]));
        return;
    }
    if (id == "physics_engine") {
        // Collision backend swap (qu3e | nudge | rpc) — rebuilt on next race.
        static const char* kEngines[3] = {"qu3e", "nudge", "rpc"};
        const std::string cur = config_.GetString("physics", "engine", "qu3e");
        int idx = 0;
        for (int i = 0; i < 3; ++i) {
            if (cur == kEngines[i]) idx = i;
        }
        if (step != 0) {
            idx = (idx + (step > 0 ? 1 : 2)) % 3;
        } else {
            idx = (idx + 1) % 3;
        }
        config_.SetString("physics", "engine", kEngines[idx]);
        return;
    }
    if (id == "volume") {
        const int delta = (step != 0) ? (step > 0 ? 20 : -20) : 20;
        const int volume = std::clamp(config_.GetInt("audio", "volume", 80) + delta, 0, 100);
        config_.SetString("audio", "volume", std::to_string(volume));
        if (audio_) audio_->SetMasterVolume(volume / 100.f);
        return;
    }
    if (id == "audio_backend") {
        static const char* kAudioBackends[4] = {"auto", "wasapi", "directsound", "winmm"};
        const std::string current = config_.GetString("audio", "backend", "auto");
        int idx = 0;
        for (int i = 0; i < 4; ++i) {
            if (current == kAudioBackends[i]) idx = i;
        }
        idx = CyclePreset(idx, 4, step);
        config_.SetString("audio", "backend", kAudioBackends[idx]);
        if (audio_) audio_->SetBackend(kAudioBackends[idx]);
    }
}

void Game::RenderSettings() {
    BuildSettingsScreen();
    RefreshSettingsValues();
    ui_.Draw(screens_.settings, renderer_);
}

// ------------------------------------------------------------ race setup ----

void Game::BuildRaceSetupScreen() {
    if (!screens_.raceSetup.widgets.empty()) return;
    const float w = static_cast<float>(renderer_.ScreenWidth());
    const float h = static_cast<float>(renderer_.ScreenHeight());
    UiScreen& s = screens_.raceSetup;
    const float top = h * 0.32f;
    const float rowH = std::clamp((h - top - 110.f) / kRaceSetupRowCount, 34.f, 52.f);
    const int fontSize = rowH > 42.f ? 24 : 20;
    s.title = "RACE SETUP";
    s.titleSize = 42;
    s.x = w * 0.5f - 280.f;
    s.y = h * 0.18f;
    s.rowStartY = top;
    s.rowHeight = rowH;
    s.valueColumnX = 560.f;
    s.hint = "A/D CHANGE    W/S SELECT    ENTER CONFIRM    ESC BACK";

    struct RowDef {
        const char* id;
        const char* label;
        UiKind kind;
    };
    static constexpr RowDef kDefs[kRaceSetupRowCount] = {
        {"circuit", "CIRCUIT", UiKind::Value},
        {"laps", "LAPS", UiKind::Value},
        {"cars", "CARS", UiKind::Value},
        {"damage", "DAMAGE", UiKind::Value},
        {"start", "START RACE", UiKind::Button},
        {"back", "BACK", UiKind::Button},
    };
    s.widgets.reserve(kRaceSetupRowCount);
    for (int i = 0; i < kRaceSetupRowCount; ++i) {
        UiWidget& row = s.widgets.emplace_back();
        row.size = fontSize;
        row.id = kDefs[i].id;
        row.label = kDefs[i].label;
        row.kind = kDefs[i].kind;
    }
}

void Game::RefreshRaceSetupValues() {
    if (screens_.raceSetup.widgets.empty()) return;
    std::vector<UiWidget>& rows = screens_.raceSetup.widgets;
    rows[0].value = MapDisplayName(mapName_);
    rows[1].value = std::to_string(totalLaps_);
    rows[2].value = std::to_string(config_.GetInt("race", "cars", 4));
    rows[3].value = config_.GetBool("race", "damage", true) ? "ON" : "OFF";
}

void Game::UpdateRaceSetup() {
    // Async build in flight: freeze the screen (RenderRaceSetup shows LOADING);
    // only BACK cancels — the join waits out the build stages.
    if (loadThread_.joinable()) {
        if (rawInput_.menuBack) {
            CancelRaceLoad();
            config_.Save(DataPath("config.rushconf"));
            phase_ = RacePhase::Menu;
        }
        return;
    }

    if (audio_ && (rawInput_.menuUp || rawInput_.menuDown || rawInput_.menuLeft ||
                   rawInput_.menuRight || rawInput_.menuSelect)) {
        audio_->PlayUIClick();
    }
    BuildRaceSetupScreen();
    const UiAction action = ui_.Update(screens_.raceSetup, rawInput_);

    const auto backToMenu = [this]() {
        config_.Save(DataPath("config.rushconf"));
        phase_ = RacePhase::Menu;
    };
    if (action.kind == UiActionKind::Back) {
        backToMenu();
        return;
    }
    if (action.kind == UiActionKind::Activate) {
        if (action.id == "start") {
            config_.Save(DataPath("config.rushconf"));
            if (threadedSim_) {
                BeginRaceLoad();  // menu keeps rendering; PollRaceLoad swaps in
            } else {
                StartRace();  // inline / drive-test / WASM: deterministic
            }
        } else if (action.id == "back") {
            backToMenu();
        }
        return;
    }

    std::string id = action.id;
    int step = 0;
    if (action.kind == UiActionKind::AdjustLeft) {
        step = -1;
    } else if (action.kind == UiActionKind::AdjustRight) {
        step = 1;
    } else if (action.kind == UiActionKind::None && rawInput_.menuSelect) {
        // Enter on a value row steps it forward (the rows did this before the
        // engine owned the input); a zero step keeps that meaning.
        const UiWidget* row = FocusedRow(screens_.raceSetup);
        if (row == nullptr || row->kind != UiKind::Value) return;
        id = row->id;
    } else {
        return;
    }
    ApplyRaceSetupStep(id, step);
    RefreshRaceSetupValues();
}

void Game::ApplyRaceSetupStep(const std::string& id, int step) {
    if (id == "circuit") {
        const int count = static_cast<int>(mapList_.size());
        int idx = 0;
        for (int i = 0; i < count; ++i) {
            if (mapList_[static_cast<std::size_t>(i)] == mapName_) idx = i;
        }
        idx = CyclePreset(idx, count, step);
        mapName_ = mapList_[static_cast<std::size_t>(idx)];
        config_.SetString("race", "map", mapName_);
        return;
    }
    if (id == "laps") {
        totalLaps_ = std::clamp(totalLaps_ + (step != 0 ? step : 1), 1, 20);
        config_.SetString("race", "laps", std::to_string(totalLaps_));
        return;
    }
    if (id == "cars") {
        const int cars = std::clamp(config_.GetInt("race", "cars", 4) + (step != 0 ? step : 1), 1, 120);
        config_.SetString("race", "cars", std::to_string(cars));
        return;
    }
    if (id == "damage") {
        const bool value =
            (step != 0) ? (step > 0) : !config_.GetBool("race", "damage", true);
        config_.SetString("race", "damage", value ? "true" : "false");
    }
}

void Game::RenderRaceSetup() {
    const float w = static_cast<float>(renderer_.ScreenWidth());
    const float h = static_cast<float>(renderer_.ScreenHeight());
    renderer_.DrawRect2D(0.f, 0.f, w, h, Color{0.f, 0.f, 0.f, 0.45f});

    // Async build in flight: the rows are gated in UpdateRaceSetup, so show
    // progress instead of a dead-looking menu.
    if (loadThread_.joinable()) {
        renderer_.DrawText2D("RACE SETUP", w * 0.5f - 120.f, h * 0.18f, 42, Color::White());
        const std::string loading = "LOADING...";
        const Vec2f loadSize = renderer_.MeasureText2D(loading, 36);
        renderer_.DrawText2D(loading, w * 0.5f - loadSize.x * 0.5f, h * 0.48f, 36,
                             Color::Orange());
        const std::string cancelHint = "ESC TO CANCEL";
        const Vec2f cancelSize = renderer_.MeasureText2D(cancelHint, 18);
        renderer_.DrawText2D(cancelHint, w * 0.5f - cancelSize.x * 0.5f, h - 52.f, 18,
                             Color::Gray());
        return;
    }

    BuildRaceSetupScreen();
    RefreshRaceSetupValues();
    ui_.Draw(screens_.raceSetup, renderer_);
}
}  // namespace apex
