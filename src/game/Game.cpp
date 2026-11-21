#include "game/Game.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "game/MapData.h"
#include "core/Paths.h"
#include "physics/NudgeWorld.h"
#include "physics/Qu3eWorld.h"

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

Game::Game(IRenderer& renderer, IInput& input, Config& config, IAudio* audio)
    : renderer_(renderer), input_(input), config_(config), audio_(audio) {}

bool Game::Init() {
    StartRace();
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
    // Defaults = classic 2025 preset (DRS on, the 2026 systems off).
    seasonDrs_ = config_.GetBool("season", "drs", true);
    seasonAero_ = config_.GetBool("season", "active_aero", false);
    seasonOverride_ = config_.GetBool("season", "overtake", false);
}

void Game::SpawnRacers() {
    // Large grids (up to 120) so the async bench can stress the sim;
    // the settings CARS row cycles the same range.
    const int carCount = std::clamp(config_.GetInt("race", "cars", 4), 1, 120);
    racers_.resize(static_cast<size_t>(carCount));
    for (int i = 0; i < carCount; ++i) {
        Racer& racer = racers_[static_cast<size_t>(i)];
        racer.name = (i == 0) ? "YOU" : ("AI " + std::to_string(i));
        racer.isPlayer = (i == 0);

        VehicleParams params;
        if (racer.isPlayer) {
            // Difficulty assists only apply to the player's car.
            params.absLevel = ParseAssistLevel(config_, "abs", DefaultAssistLevel("abs"));
            params.tcLevel = ParseAssistLevel(config_, "tc", DefaultAssistLevel("tc"));
            params.escLevel = ParseAssistLevel(config_, "esc", DefaultAssistLevel("esc"));
            params.steerLevel = ParseAssistLevel(config_, "steer", DefaultAssistLevel("steer"));
        } else {
            // AI cars run a fixed assist set so player difficulty settings
            // never change their behavior. Without the counter-steer / yaw
            // aids the AI spins at the assists-off difficulty.
            params.absLevel = AssistLevel::Sport;
            params.tcLevel = AssistLevel::Sport;
            params.escLevel = AssistLevel::Full;
            params.steerLevel = AssistLevel::Full;
        }
        params.damageEnabled = config_.GetBool("race", "damage", true);
        // Season rules apply to every car (player and AI alike).
        params.drsEnabled = seasonDrs_;
        params.activeAeroEnabled = seasonAero_;
        params.overtakeEnabled = seasonOverride_;

        Transform pose;
        pose.position = track_.StartPosition(i);
        pose.rotation = track_.StartRotation();
        racer.vehicle.Spawn(*world_, pose, params, &racer);
    }
}

// (Re)build world, track and grid from the current config — used at boot and
// whenever the player starts a race from the menu.
void Game::StartRace() {
    LoadSettings();
    racers_.clear();
    drsWanted_ = false;
    // Swap collision backends in-game via [physics] engine (qu3e | nudge).
    if (config_.GetString("physics", "engine", "qu3e") == "nudge") {
        world_ = std::make_unique<NudgeWorld>();
    } else {
        world_ = std::make_unique<Qu3eWorld>(fixedDt_);
    }
    world_->SetGravity({0.f, -9.81f, 0.f});
    BuildWorld();
    SpawnRacers();
    phase_ = RacePhase::Countdown;
    phaseTimer_ = 3.5f;
    raceClock_ = 0.f;
    accumulator_ = 0.f;
}

void Game::BuildWorld() {
    const MapData map = LoadMapData(DataPath("assets/maps"), mapName_);
    track_.Build(map, *world_, renderer_);
}

void Game::Update() {
    const BenchClock::time_point benchT0 = BenchClock::now();
    const float frameDt = std::min(renderer_.FrameTime(), 0.1f);

    UpdatePlayerInput();

    if (phase_ == RacePhase::Menu) {
        UpdateMenu();
    } else if (phase_ == RacePhase::Settings) {
        UpdateSettings();
    } else if (phase_ == RacePhase::RaceSetup) {
        UpdateRaceSetup();
    } else {
        if (benchOn_) {
            const BenchClock::time_point t0 = BenchClock::now();
            for (Racer& racer : racers_) {
                if (!racer.isPlayer && !racer.finished) UpdateAI(racer, frameDt);
            }
            bench_.aiSec += BenchSec(t0, BenchClock::now());
        } else {
            for (Racer& racer : racers_) {
                if (!racer.isPlayer && !racer.finished) UpdateAI(racer, frameDt);
            }
        }

        // Fixed-step physics (rate from [physics] hz, default 240).
        accumulator_ += frameDt;
        while (accumulator_ >= fixedDt_) {
            StepPhysics(fixedDt_);
            accumulator_ -= fixedDt_;
        }

        if (benchOn_) {
            const BenchClock::time_point t0 = BenchClock::now();
            UpdateRace(frameDt);
            bench_.raceSec += BenchSec(t0, BenchClock::now());
        } else {
            UpdateRace(frameDt);
        }
    }

    UpdateCamera(frameDt);
    if (statusTimer_ > 0.f) statusTimer_ -= frameDt;

    // Feed the audio engine with player telemetry (engine / wind / skid).
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

    if (benchOn_) {
        bench_.updateSec += BenchSec(benchT0, BenchClock::now());
        ++bench_.frames;
    }
}

void Game::Render3D() {
    track_.Render(renderer_);

    for (Racer& racer : racers_) {
        Color tint = racer.isPlayer ? Color{0.15f, 0.55f, 1.f, 1.f} : Color{0.9f, 0.45f, 0.12f, 1.f};
        racer.vehicle.Render(renderer_, tint);
    }
}

void Game::UpdatePlayerInput() {
    if (racers_.empty()) return;
    Racer& player = racers_.front();

    rawInput_ = input_.Poll();
    const InputState& raw = rawInput_;

    // Menus own their input (UpdateMenu / UpdateSettings / UpdateRaceSetup);
    // cars stay parked.
    if (phase_ == RacePhase::Menu || phase_ == RacePhase::Settings ||
        phase_ == RacePhase::RaceSetup) {
        player.input = VehicleInput{};
        return;
    }

    if (raw.cameraCycle) cameraMode_ = (cameraMode_ + 1) % 4;

    // ESC/P drops back to the menu from any race phase.
    if (raw.pause) {
        phase_ = RacePhase::Menu;
        menuCursor_ = 0;
        player.input = VehicleInput{};
        return;
    }

    if (phase_ == RacePhase::Racing && !player.finished) {
        // DRS: F only arms/disarms inside a map-marked zone (authored on
        // straights), and only when the season enables DRS. Leaving the
        // zone closes DRS and resets the latch, so re-entering a zone later
        // starts closed — the driver must re-arm.
        const float playerD =
            track_.ProjectDistance(player.vehicle.State().transform.position);
        const bool inDrsZone = track_.InDrsZone(playerD);
        if (!inDrsZone) {
            drsWanted_ = false;
        } else if (raw.drsToggle && seasonDrs_) {
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
        player.input.wingToggle = raw.wingModeToggle && seasonAero_;
        player.input.overtake = raw.overtake && seasonOverride_;
    } else if (phase_ == RacePhase::Countdown) {
        player.input = VehicleInput{};
    } else {
        player.input.throttle = 0.f;
        player.input.brake = 0.8f;
    }

    if (raw.restart) {
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
    if (config_.GetBool("race", "autopilot", false) && phase_ == RacePhase::Racing &&
        !player.finished) {
        player.input = VehicleInput{};
        UpdateAI(player, 0.f);
    }
}

void Game::UpdateAI(Racer& racer, float dt) {
    (void)dt;
    if (phase_ != RacePhase::Racing) {
        racer.input = VehicleInput{};
        return;
    }
    const VehicleState& state = racer.vehicle.State();
    const float d = track_.ProjectDistance(state.transform.position);

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
    const float dfPerV2 = 0.5f * p.airDensity * p.downforceArea / p.mass;
    const float gripDenom = curvMax - dfPerV2;
    float vTarget = gripDenom > 1e-4f ? std::sqrt(aLatBase / gripDenom) : 62.f;
    vTarget = Clampf(vTarget * 0.95f, 12.f, 62.f);

    const float total = track_.TotalLength();
    for (const Racer& other : racers_) {
        if (&other == &racer) continue;
        const float od = track_.ProjectDistance(other.vehicle.State().transform.position);
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

    BenchClock::time_point t = b ? BenchClock::now() : BenchClock::time_point{};
    for (Racer& racer : racers_) {
        racer.vehicle.Update(racer.input, dt);
    }
    if (b) {
        bench_.vehicleSec += BenchSec(t, BenchClock::now());
        t = BenchClock::now();
    }

    world_->Step(dt);
    if (b) {
        bench_.stepSec += BenchSec(t, BenchClock::now());
        t = BenchClock::now();
    }

    const std::vector<ContactEvent> contacts = world_->DrainContacts();
    for (Racer& racer : racers_) {
        racer.vehicle.ProcessContacts(contacts, dt);
    }
    if (b) bench_.contactSec += BenchSec(t, BenchClock::now());

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
        const float d = track_.ProjectDistance(state.transform.position);

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
            }
            if (racer.lap >= totalLaps_) {
                racer.finished = true;
                racer.raceTime = raceClock_;
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

    // Race end: when the player finishes (or everyone has).
    if (racers_.front().finished) {
        phase_ = RacePhase::Finished;
    }
}

void Game::UpdateCamera(float dt) {
    if (racers_.empty()) return;
    const VehicleState& state = racers_.front().vehicle.State();
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
    if (racers_.empty()) return;
    const Racer& player = racers_.front();
    const VehicleState& state = player.vehicle.State();

    const float w = static_cast<float>(renderer_.ScreenWidth());
    const float h = static_cast<float>(renderer_.ScreenHeight());

    // Top-left: position + lap.
    char buffer[128];
    std::snprintf(buffer, sizeof(buffer), "P%d/%d   LAP %d/%d", player.position,
                  static_cast<int>(racers_.size()), std::min(player.lap + 1, totalLaps_), totalLaps_);
    renderer_.DrawRect2D(10.f, 10.f, 340.f, 92.f, Color{0.f, 0.f, 0.f, 0.45f});
    renderer_.DrawText2D(buffer, 22.f, 18.f, 22, Color::White());
    renderer_.DrawText2D("TIME  " + FormatTime(raceClock_), 22.f, 46.f, 18, Color::Gray());
    renderer_.DrawText2D("LAST  " + FormatTime(player.lastLapTime) + "   BEST " +
                             FormatTime(player.bestLapTime),
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
        const float health = state.partHealth[i];
        const float x = 20.f + static_cast<float>(i) * 68.f;
        renderer_.DrawRect2D(x, baseY + 80.f, 60.f, 12.f, Color{0.15f, 0.15f, 0.18f, 0.9f});
        renderer_.DrawRect2D(x + 2.f, baseY + 82.f, 56.f * health, 8.f,
                             health > 0.7f ? Color::Green() : (health > 0.35f ? Color::Yellow() : Color::Red()));
        renderer_.DrawText2D(kPartNames[i], x + 18.f, baseY + 98.f, 14, Color::Gray());
    }

    // ERS bar + DRS/wing indicators.
    renderer_.DrawText2D("ERS", 20.f, baseY + 118.f, 16, Color::Gray());
    renderer_.DrawRect2D(60.f, baseY + 118.f, 140.f, 12.f, Color{0.15f, 0.15f, 0.18f, 0.9f});
    renderer_.DrawRect2D(62.f, baseY + 120.f, 136.f * state.ersCharge, 8.f,
                         state.ersDeploying ? Color::Yellow() : Color::Blue());

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
        const int count = std::min(3, static_cast<int>(std::ceil(phaseTimer_ - 0.5f)));
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
        std::snprintf(buffer, sizeof(buffer), "P%d   TIME %s   BEST %s", player.position,
                      FormatTime(player.raceTime > 0.f ? player.raceTime : raceClock_).c_str(),
                      FormatTime(player.bestLapTime).c_str());
        renderer_.DrawText2D(buffer, w * 0.5f - 190.f, h * 0.5f - 8.f, 22, Color::Yellow());
        renderer_.DrawText2D("ENTER = RESTART    ESC = MENU", w * 0.5f - 190.f, h * 0.5f + 40.f, 20,
                             Color::Gray());
    }

    if (statusTimer_ > 0.f && !statusText_.empty()) {
        renderer_.DrawText2D(statusText_, w * 0.5f - 80.f, h - 200.f, 22, Color::Orange());
    }

    // Help line lists only the keys this season actually wires up.
    std::string help = "W/S THROTTLE-BRAKE   A/D STEER";
    if (seasonDrs_) help += "   F DRS";
    if (seasonAero_) help += "   V WING";
    if (seasonOverride_) help += "   SHIFT OVERTAKE";
    help += "   C CAMERA   ESC MENU";
    renderer_.DrawText2D(help, 12.f, h - 22.f, 15, Color::DarkGray());
}

void Game::UpdateMenu() {
    const InputState& in = rawInput_;
    if (audio_ && (in.menuUp || in.menuDown || in.menuSelect)) audio_->PlayUIClick();
    if (in.menuUp) menuCursor_ = (menuCursor_ + 2) % 3;
    if (in.menuDown) menuCursor_ = (menuCursor_ + 1) % 3;
    if (in.menuSelect) {
        switch (menuCursor_) {
            case 0:
                // START RACE opens the pre-race setup, not the race itself.
                phase_ = RacePhase::RaceSetup;
                raceSetupCursor_ = 0;
                break;
            case 1:
                phase_ = RacePhase::Settings;
                settingsCursor_ = 0;
                break;
            default:
                wantsQuit_ = true;
                break;
        }
    }
    // ESC is inert here on purpose: quitting must go through the QUIT item
    // only. Acting on menuBack also ate the same keypress that just brought
    // the player back from a race ("ESC exits the app instead of the menu").
}

void Game::UpdateSettings() {
    constexpr int kRows = kSettingsRowCount;
    const InputState& in = rawInput_;
    if (audio_ && (in.menuUp || in.menuDown || in.menuLeft || in.menuRight || in.menuSelect)) {
        audio_->PlayUIClick();
    }
    if (in.menuUp) settingsCursor_ = (settingsCursor_ + kRows - 1) % kRows;
    if (in.menuDown) settingsCursor_ = (settingsCursor_ + 1) % kRows;

    auto backToMenu = [this]() {
        config_.Save(DataPath("config.ini"));
        phase_ = RacePhase::Menu;
        menuCursor_ = 0;
    };

    if (settingsCursor_ == kRows - 1) {
        if (in.menuSelect || in.menuBack) backToMenu();
        return;
    }
    if (in.menuBack) {
        backToMenu();
        return;
    }

    const int dir = (in.menuRight ? 1 : 0) - (in.menuLeft ? 1 : 0);
    if (dir == 0 && !in.menuSelect) return;

    switch (settingsCursor_) {
        case 0: {
            // SEASON preset: classic (DRS) <-> 2026 (aero + override).
            // Writing a preset rewrites the three feature toggles with it.
            const std::string preset = config_.GetString("season", "preset", "2025");
            int idx = (preset == "2026") ? 1 : 0;  // custom -> classic
            idx = CyclePreset(idx, 2, dir);
            const bool is2026 = (idx == 1);
            config_.SetString("season", "preset", is2026 ? "2026" : "2025");
            config_.SetString("season", "drs", is2026 ? "0" : "1");
            config_.SetString("season", "active_aero", is2026 ? "1" : "0");
            config_.SetString("season", "overtake", is2026 ? "1" : "0");
            seasonDrs_ = !is2026;
            seasonAero_ = is2026;
            seasonOverride_ = is2026;
            break;
        }
        case 1:
        case 2:
        case 3: {
            // Per-feature toggles. Flipping one marks the preset CUSTOM —
            // the three systems may then mix, but only deliberately.
            static const char* kSeasonKeys[3] = {"drs", "active_aero", "overtake"};
            bool* gates[3] = {&seasonDrs_, &seasonAero_, &seasonOverride_};
            const int i = settingsCursor_ - 1;
            bool value = (dir != 0) ? (dir > 0) : !*gates[i];
            *gates[i] = value;
            config_.SetString("season", kSeasonKeys[i], value ? "1" : "0");
            config_.SetString("season", "preset", "custom");
            break;
        }
        case 4:
        case 5:
        case 6:
        case 7: {
            static const char* kAssistKeys[4] = {"abs", "tc", "esc", "steer"};
            const char* key = kAssistKeys[settingsCursor_ - 4];
            int level = static_cast<int>(ParseAssistLevel(config_, key, DefaultAssistLevel(key)));
            level = CyclePreset(level, 3, dir);
            SaveAssistLevel(config_, key, static_cast<AssistLevel>(level));
            break;
        }
        case 8: {
            video_.borderlessFullscreen =
                (dir != 0) ? (dir > 0) : !video_.borderlessFullscreen;
            config_.SetString("video", "mode",
                              video_.borderlessFullscreen ? "borderless" : "windowed");
            renderer_.ApplyVideo(video_);
            break;
        }
        case 9: {
            int idx = 0;
            for (int i = 0; i < kResPresetCount; ++i) {
                if (kResPresets[i].width == video_.width && kResPresets[i].height == video_.height) {
                    idx = i;
                }
            }
            idx = CyclePreset(idx, kResPresetCount, dir);
            video_.width = kResPresets[idx].width;
            video_.height = kResPresets[idx].height;
            config_.SetString("video", "width", std::to_string(video_.width));
            config_.SetString("video", "height", std::to_string(video_.height));
            renderer_.ApplyVideo(video_);
            break;
        }
        case 10: {
            video_.shadows = (dir != 0) ? (dir > 0) : !video_.shadows;
            config_.SetString("video", "quality", video_.shadows ? "high" : "low");
            renderer_.ApplyVideo(video_);
            break;
        }
        case 11: {
            int idx = 0;
            for (int i = 0; i < kFpsPresetCount; ++i) {
                if (kFpsPresets[i] == video_.fpsLimit) idx = i;
            }
            idx = CyclePreset(idx, kFpsPresetCount, dir);
            video_.fpsLimit = kFpsPresets[idx];
            config_.SetString("video", "fps_limit", std::to_string(video_.fpsLimit));
            renderer_.ApplyVideo(video_);
            break;
        }
        case 12: {
            const int hz = static_cast<int>(1.f / fixedDt_ + 0.5f);
            int idx = 1;
            for (int i = 0; i < kPhysicsPresetCount; ++i) {
                if (kPhysicsPresets[i] == hz) idx = i;
            }
            idx = CyclePreset(idx, kPhysicsPresetCount, dir);
            fixedDt_ = 1.f / static_cast<float>(kPhysicsPresets[idx]);
            config_.SetString("physics", "hz", std::to_string(kPhysicsPresets[idx]));
            break;
        }
        case 13: {
            // Collision backend swap (qu3e | nudge) — rebuilt on next race.
            const bool nudge = config_.GetString("physics", "engine", "qu3e") == "nudge";
            config_.SetString("physics", "engine",
                              (dir != 0) ? (dir > 0 ? "nudge" : "qu3e")
                                         : (nudge ? "qu3e" : "nudge"));
            break;
        }
        case 14: {
            const int volume =
                std::clamp(config_.GetInt("audio", "volume", 80) + (dir != 0 ? (dir > 0 ? 20 : -20) : 20),
                           0, 100);
            config_.SetString("audio", "volume", std::to_string(volume));
            if (audio_) audio_->SetMasterVolume(volume / 100.f);
            break;
        }
        default: {
            static const char* kAudioBackends[4] = {"auto", "wasapi", "directsound", "winmm"};
            const std::string current = config_.GetString("audio", "backend", "auto");
            int idx = 0;
            for (int i = 0; i < 4; ++i) {
                if (current == kAudioBackends[i]) idx = i;
            }
            idx = CyclePreset(idx, 4, dir);
            config_.SetString("audio", "backend", kAudioBackends[idx]);
            if (audio_) audio_->SetBackend(kAudioBackends[idx]);
            break;
        }
    }
}

void Game::UpdateRaceSetup() {
    constexpr int kRows = kRaceSetupRowCount;
    const InputState& in = rawInput_;
    if (audio_ && (in.menuUp || in.menuDown || in.menuLeft || in.menuRight || in.menuSelect)) {
        audio_->PlayUIClick();
    }
    if (in.menuUp) raceSetupCursor_ = (raceSetupCursor_ + kRows - 1) % kRows;
    if (in.menuDown) raceSetupCursor_ = (raceSetupCursor_ + 1) % kRows;

    if (in.menuBack) {
        config_.Save(DataPath("config.ini"));
        phase_ = RacePhase::Menu;
        menuCursor_ = 0;
        return;
    }

    const int dir = (in.menuRight ? 1 : 0) - (in.menuLeft ? 1 : 0);
    if (dir == 0 && !in.menuSelect) return;

    switch (raceSetupCursor_) {
        case 0: {
            const int count = static_cast<int>(mapList_.size());
            int idx = 0;
            for (int i = 0; i < count; ++i) {
                if (mapList_[static_cast<size_t>(i)] == mapName_) idx = i;
            }
            idx = CyclePreset(idx, count, dir);
            mapName_ = mapList_[static_cast<size_t>(idx)];
            config_.SetString("race", "map", mapName_);
            break;
        }
        case 1: {
            totalLaps_ = std::clamp(totalLaps_ + (dir != 0 ? dir : 1), 1, 20);
            config_.SetString("race", "laps", std::to_string(totalLaps_));
            break;
        }
        case 2: {
            const int cars = std::clamp(config_.GetInt("race", "cars", 4) + (dir != 0 ? dir : 1),
                                        1, 120);
            config_.SetString("race", "cars", std::to_string(cars));
            break;
        }
        case 3: {
            bool value = config_.GetBool("race", "damage", true);
            value = (dir != 0) ? (dir > 0) : !value;
            config_.SetString("race", "damage", value ? "true" : "false");
            break;
        }
        case 4: {
            // START
            config_.Save(DataPath("config.ini"));
            StartRace();
            break;
        }
        default: {
            // BACK
            config_.Save(DataPath("config.ini"));
            phase_ = RacePhase::Menu;
            menuCursor_ = 0;
            break;
        }
    }
}

void Game::RenderMenu() {
    const float w = static_cast<float>(renderer_.ScreenWidth());
    const float h = static_cast<float>(renderer_.ScreenHeight());

    renderer_.DrawRect2D(0.f, 0.f, w, h, Color{0.f, 0.f, 0.f, 0.35f});
    renderer_.DrawText2D("APEX RUSH", w * 0.5f - 165.f, h * 0.24f, 68, Color::White());
    renderer_.DrawText2D("OPEN F1 SIM", w * 0.5f - 92.f, h * 0.24f + 78.f, 20, Color::Gray());

    const char* items[3] = {"START RACE", "SETTINGS", "QUIT"};
    float y = h * 0.52f;
    for (int i = 0; i < 3; ++i) {
        const bool selected = (i == menuCursor_);
        const Vec2f size = renderer_.MeasureText2D(items[i], 28);
        const float x = w * 0.5f - size.x * 0.5f;
        if (selected) {
            renderer_.DrawRect2D(x - 28.f, y - 8.f, size.x + 56.f, 42.f,
                                 Color{1.f, 0.55f, 0.15f, 0.22f});
        }
        renderer_.DrawText2D(items[i], x, y, 28, selected ? Color::Orange() : Color::White());
        y += 54.f;
    }

    const std::string hint = "W/S SELECT    ENTER CONFIRM";
    const Vec2f hintSize = renderer_.MeasureText2D(hint, 18);
    renderer_.DrawText2D(hint, w * 0.5f - hintSize.x * 0.5f, h - 52.f, 18, Color::Gray());
}

void Game::RenderSettings() {
    const float w = static_cast<float>(renderer_.ScreenWidth());
    const float h = static_cast<float>(renderer_.ScreenHeight());

    renderer_.DrawRect2D(0.f, 0.f, w, h, Color{0.f, 0.f, 0.f, 0.45f});

    const std::string title = "SETTINGS";
    const Vec2f titleSize = renderer_.MeasureText2D(title, 42);
    renderer_.DrawText2D(title, w * 0.5f - titleSize.x * 0.5f, h * 0.08f, 42, Color::White());

    struct Row {
        const char* label;
        std::string value;
    };
    const int physicsHz = static_cast<int>(1.f / fixedDt_ + 0.5f);
    char resBuffer[32];
    std::snprintf(resBuffer, sizeof(resBuffer), "%dx%d", video_.width, video_.height);
    // Season preset display: named presets or CUSTOM once a feature gate
    // was flipped individually.
    const std::string preset = config_.GetString("season", "preset", "2025");
    const std::string seasonName = preset == "2026"   ? "2026"
                                   : preset == "2025" ? "2025"
                                                      : "CUSTOM";
    const Row rows[kSettingsRowCount] = {
        {"SEASON", seasonName},
        {"DRS", seasonDrs_ ? "ON" : "OFF"},
        {"ACTIVE AERO", seasonAero_ ? "ON" : "OFF"},
        {"OVERTAKE", seasonOverride_ ? "ON" : "OFF"},
        {"ABS", AssistLevelName(ParseAssistLevel(config_, "abs", DefaultAssistLevel("abs")))},
        {"TRACTION CTRL", AssistLevelName(ParseAssistLevel(config_, "tc", DefaultAssistLevel("tc")))},
        {"STABILITY CTRL", AssistLevelName(ParseAssistLevel(config_, "esc", DefaultAssistLevel("esc")))},
        {"STEER ASSIST", AssistLevelName(ParseAssistLevel(config_, "steer", DefaultAssistLevel("steer")))},
        {"DISPLAY MODE", video_.borderlessFullscreen ? "BORDERLESS" : "WINDOWED"},
        {"RESOLUTION", resBuffer},
        {"QUALITY", video_.shadows ? "HIGH" : "LOW"},
        {"FPS LIMIT", video_.fpsLimit > 0 ? std::to_string(video_.fpsLimit) : "UNCAPPED"},
        {"PHYSICS RATE", std::to_string(physicsHz) + " HZ"},
        {"PHYSICS ENGINE", MapDisplayName(config_.GetString("physics", "engine", "qu3e"))},
        {"SOUND VOLUME", std::to_string(config_.GetInt("audio", "volume", 80))},
        {"AUDIO BACKEND", MapDisplayName(config_.GetString("audio", "backend", "auto"))},
        {"BACK", ""},
    };

    // The row height shrinks so the list fits 720p.
    const float top = h * 0.17f;
    const float rowH = std::clamp((h - top - 90.f) / kSettingsRowCount, 26.f, 46.f);
    const int fontSize = rowH > 38.f ? 22 : 17;
    float y = top;
    for (int i = 0; i < kSettingsRowCount; ++i) {
        const bool selected = (i == settingsCursor_);
        renderer_.DrawRect2D(w * 0.5f - 300.f, y - 2.f, 600.f, rowH - 4.f,
                             selected ? Color{1.f, 0.55f, 0.15f, 0.22f}
                                      : Color{0.f, 0.f, 0.f, 0.25f});
        renderer_.DrawText2D(rows[i].label, w * 0.5f - 280.f, y + (rowH - 4.f - fontSize) * 0.5f,
                             fontSize, selected ? Color::Orange() : Color::White());
        if (!rows[i].value.empty()) {
            const Vec2f valueSize = renderer_.MeasureText2D(rows[i].value, fontSize);
            renderer_.DrawText2D(rows[i].value, w * 0.5f + 280.f - valueSize.x,
                                 y + (rowH - 4.f - fontSize) * 0.5f, fontSize,
                                 selected ? Color::Yellow() : Color::Gray());
        }
        y += rowH;
    }

    const std::string hint = "A/D CHANGE    W/S SELECT    ENTER/ESC BACK";
    const Vec2f hintSize = renderer_.MeasureText2D(hint, 18);
    renderer_.DrawText2D(hint, w * 0.5f - hintSize.x * 0.5f, h - 52.f, 18, Color::Gray());
}

void Game::RenderRaceSetup() {
    const float w = static_cast<float>(renderer_.ScreenWidth());
    const float h = static_cast<float>(renderer_.ScreenHeight());

    renderer_.DrawRect2D(0.f, 0.f, w, h, Color{0.f, 0.f, 0.f, 0.45f});

    const std::string title = "RACE SETUP";
    const Vec2f titleSize = renderer_.MeasureText2D(title, 42);
    renderer_.DrawText2D(title, w * 0.5f - titleSize.x * 0.5f, h * 0.18f, 42, Color::White());

    struct Row {
        const char* label;
        std::string value;
    };
    const Row rows[kRaceSetupRowCount] = {
        {"CIRCUIT", MapDisplayName(mapName_)},
        {"LAPS", std::to_string(totalLaps_)},
        {"CARS", std::to_string(config_.GetInt("race", "cars", 4))},
        {"DAMAGE", config_.GetBool("race", "damage", true) ? "ON" : "OFF"},
        {"START RACE", ""},
        {"BACK", ""},
    };

    const float top = h * 0.32f;
    const float rowH = std::clamp((h - top - 110.f) / kRaceSetupRowCount, 34.f, 52.f);
    const int fontSize = rowH > 42.f ? 24 : 20;
    float y = top;
    for (int i = 0; i < kRaceSetupRowCount; ++i) {
        const bool selected = (i == raceSetupCursor_);
        const bool action = (i >= 4);  // START / BACK rows
        renderer_.DrawRect2D(w * 0.5f - 300.f, y - 2.f, 600.f, rowH - 4.f,
                             selected ? Color{1.f, 0.55f, 0.15f, 0.22f}
                                      : Color{0.f, 0.f, 0.f, 0.25f});
        renderer_.DrawText2D(rows[i].label,
                             action ? w * 0.5f - renderer_.MeasureText2D(rows[i].label, fontSize).x * 0.5f
                                    : w * 0.5f - 280.f,
                             y + (rowH - 4.f - fontSize) * 0.5f, fontSize,
                             selected ? Color::Orange() : Color::White());
        if (!rows[i].value.empty()) {
            const Vec2f valueSize = renderer_.MeasureText2D(rows[i].value, fontSize);
            renderer_.DrawText2D(rows[i].value, w * 0.5f + 280.f - valueSize.x,
                                 y + (rowH - 4.f - fontSize) * 0.5f, fontSize,
                                 selected ? Color::Yellow() : Color::Gray());
        }
        y += rowH;
    }

    const std::string hint = "A/D CHANGE    W/S SELECT    ENTER CONFIRM    ESC BACK";
    const Vec2f hintSize = renderer_.MeasureText2D(hint, 18);
    renderer_.DrawText2D(hint, w * 0.5f - hintSize.x * 0.5f, h - 52.f, 18, Color::Gray());
}

}  // namespace apex
