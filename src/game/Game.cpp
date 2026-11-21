#include "game/Game.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "game/MapData.h"
#include "physics/Qu3eWorld.h"

namespace apex {

namespace {

constexpr const char* kMapsRoot = "assets/maps";

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

Game::Game(IRenderer& renderer, IInput& input, Config& config)
    : renderer_(renderer), input_(input), config_(config) {}

bool Game::Init() {
    StartRace();
    // Tests and automation skip straight to the grid; normal launches open
    // the main menu over the parked scene.
    phase_ = config_.GetBool("race", "autostart", false) ? RacePhase::Countdown
                                                         : RacePhase::Menu;
    return true;
}

void Game::LoadSettings() {
    mapList_ = ListMaps(kMapsRoot);
    if (mapList_.empty()) mapList_.push_back("test_circuit");
    mapName_ = config_.GetString("race", "map", mapList_.front());
    if (std::find(mapList_.begin(), mapList_.end(), mapName_) == mapList_.end()) {
        mapName_ = mapList_.front();
    }
    totalLaps_ = std::clamp(config_.GetInt("race", "laps", 3), 1, 20);
}

void Game::SpawnRacers() {
    const int carCount = std::clamp(config_.GetInt("race", "cars", 4), 1, 8);
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
    world_ = std::make_unique<Qu3eWorld>();
    world_->SetGravity({0.f, -9.81f, 0.f});
    BuildWorld();
    SpawnRacers();
    phase_ = RacePhase::Countdown;
    phaseTimer_ = 3.5f;
    raceClock_ = 0.f;
    accumulator_ = 0.f;
}

void Game::BuildWorld() {
    const MapData map = LoadMapData(kMapsRoot, mapName_);
    track_.Build(map, *world_, renderer_);
}

void Game::Update() {
    const float frameDt = std::min(renderer_.FrameTime(), 0.1f);

    UpdatePlayerInput();

    if (phase_ == RacePhase::Menu) {
        UpdateMenu();
    } else if (phase_ == RacePhase::Settings) {
        UpdateSettings();
    } else {
        for (Racer& racer : racers_) {
            if (!racer.isPlayer && !racer.finished) UpdateAI(racer, frameDt);
        }

        // Fixed-step physics: 240 Hz.
        accumulator_ += frameDt;
        while (accumulator_ >= kFixedDt) {
            StepPhysics(kFixedDt);
            accumulator_ -= kFixedDt;
        }

        UpdateRace(frameDt);
    }

    UpdateCamera(frameDt);
    if (statusTimer_ > 0.f) statusTimer_ -= frameDt;
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

    // Menus own their input (UpdateMenu / UpdateSettings); cars stay parked.
    if (phase_ == RacePhase::Menu || phase_ == RacePhase::Settings) {
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
        player.input.throttle = raw.throttle;
        player.input.brake = raw.brake;
        player.input.steer = raw.steer;
        player.input.handbrake = raw.handbrake;
        player.input.reverse = raw.reverse;
        player.input.drsRequest = raw.drsToggle;
        player.input.wingToggle = raw.wingModeToggle;
        player.input.overtake = raw.overtake;
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

    // Use DRS/overtake features like the player.
    racer.input.drsRequest = true;
    racer.input.overtake = curvMax < 0.002f && state.speed > 50.f;
    racer.input.wingToggle = false;
}

void Game::StepPhysics(float dt) {
    for (Racer& racer : racers_) {
        racer.vehicle.Update(racer.input, dt);
    }

    world_->Step(dt);

    const std::vector<ContactEvent> contacts = world_->DrainContacts();
    for (Racer& racer : racers_) {
        racer.vehicle.ProcessContacts(contacts, dt);
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
    if (phase_ == RacePhase::Menu || phase_ == RacePhase::Settings) {
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
        case 0:  // chase
            desiredPos = state.transform.position - forward * 9.5f + up * 3.4f;
            desiredTarget = state.transform.position + forward * 6.f;
            break;
        case 1:  // far chase
            desiredPos = state.transform.position - forward * 15.f + up * 6.5f;
            desiredTarget = state.transform.position + forward * 4.f;
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

    const float blend = (cameraSmooth_ <= 0.f) ? 1.f : Clampf(dt * 6.f, 0.f, 1.f);
    cameraSmooth_ = 1.f;
    cameraPos_ += (desiredPos - cameraPos_) * blend;
    cameraTarget_ += (desiredTarget - cameraTarget_) * blend;

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

    if (state.drsActive) {
        renderer_.DrawText2D("DRS", w - 120.f, 30.f, 30, Color::Green());
    } else if (state.drsAvailable) {
        renderer_.DrawText2D("DRS RDY", w - 170.f, 30.f, 22, Color::Yellow());
    }
    renderer_.DrawText2D(state.wingMode == WingMode::XMode ? "X-MODE" : "Z-MODE", w - 150.f, 64.f,
                         20, state.wingMode == WingMode::XMode ? Color::Blue() : Color::Orange());

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

    renderer_.DrawText2D("W/S THROTTLE-BRAKE   A/D STEER   F DRS   V WING   SHIFT OVERTAKE   C CAMERA   ESC MENU",
                         12.f, h - 22.f, 15, Color::DarkGray());
}

void Game::UpdateMenu() {
    const InputState& in = rawInput_;
    if (in.menuUp) menuCursor_ = (menuCursor_ + 2) % 3;
    if (in.menuDown) menuCursor_ = (menuCursor_ + 1) % 3;
    if (in.menuSelect) {
        switch (menuCursor_) {
            case 0:
                StartRace();
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
    if (in.menuBack) wantsQuit_ = true;
}

void Game::UpdateSettings() {
    constexpr int kRows = 9;  // circuit, laps, cars, damage, abs, tc, esc, steer, back
    const InputState& in = rawInput_;
    if (in.menuUp) settingsCursor_ = (settingsCursor_ + kRows - 1) % kRows;
    if (in.menuDown) settingsCursor_ = (settingsCursor_ + 1) % kRows;

    auto backToMenu = [this]() {
        config_.Save("config.ini");
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
            const int count = static_cast<int>(mapList_.size());
            int idx = 0;
            for (int i = 0; i < count; ++i) {
                if (mapList_[static_cast<size_t>(i)] == mapName_) idx = i;
            }
            idx = (idx + (dir != 0 ? (dir > 0 ? 1 : -1) : 1) + count) % count;
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
                                        1, 8);
            config_.SetString("race", "cars", std::to_string(cars));
            break;
        }
        case 3: {
            bool value = config_.GetBool("race", "damage", true);
            value = (dir != 0) ? (dir > 0) : !value;
            config_.SetString("race", "damage", value ? "true" : "false");
            break;
        }
        default: {
            static const char* kAssistKeys[4] = {"abs", "tc", "esc", "steer"};
            const char* key = kAssistKeys[settingsCursor_ - 4];
            int level = static_cast<int>(ParseAssistLevel(config_, key, DefaultAssistLevel(key)));
            level = (level + (dir != 0 ? (dir > 0 ? 1 : -1) : 1) + 3) % 3;
            SaveAssistLevel(config_, key, static_cast<AssistLevel>(level));
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

    const std::string hint = "W/S SELECT    ENTER CONFIRM    ESC QUIT";
    const Vec2f hintSize = renderer_.MeasureText2D(hint, 18);
    renderer_.DrawText2D(hint, w * 0.5f - hintSize.x * 0.5f, h - 52.f, 18, Color::Gray());
}

void Game::RenderSettings() {
    const float w = static_cast<float>(renderer_.ScreenWidth());
    const float h = static_cast<float>(renderer_.ScreenHeight());

    renderer_.DrawRect2D(0.f, 0.f, w, h, Color{0.f, 0.f, 0.f, 0.45f});

    const std::string title = "SETTINGS";
    const Vec2f titleSize = renderer_.MeasureText2D(title, 42);
    renderer_.DrawText2D(title, w * 0.5f - titleSize.x * 0.5f, h * 0.14f, 42, Color::White());

    struct Row {
        const char* label;
        std::string value;
    };
    const int cars = config_.GetInt("race", "cars", 4);
    const Row rows[9] = {
        {"CIRCUIT", MapDisplayName(mapName_)},
        {"LAPS", std::to_string(totalLaps_)},
        {"CARS", std::to_string(cars)},
        {"DAMAGE", config_.GetBool("race", "damage", true) ? "ON" : "OFF"},
        {"ABS", AssistLevelName(ParseAssistLevel(config_, "abs", DefaultAssistLevel("abs")))},
        {"TRACTION CTRL", AssistLevelName(ParseAssistLevel(config_, "tc", DefaultAssistLevel("tc")))},
        {"STABILITY CTRL", AssistLevelName(ParseAssistLevel(config_, "esc", DefaultAssistLevel("esc")))},
        {"STEER ASSIST", AssistLevelName(ParseAssistLevel(config_, "steer", DefaultAssistLevel("steer")))},
        {"BACK", ""},
    };

    float y = h * 0.26f;
    for (int i = 0; i < 9; ++i) {
        const bool selected = (i == settingsCursor_);
        renderer_.DrawRect2D(w * 0.5f - 300.f, y - 6.f, 600.f, 40.f,
                             selected ? Color{1.f, 0.55f, 0.15f, 0.22f}
                                      : Color{0.f, 0.f, 0.f, 0.25f});
        renderer_.DrawText2D(rows[i].label, w * 0.5f - 280.f, y, 22,
                             selected ? Color::Orange() : Color::White());
        if (!rows[i].value.empty()) {
            const Vec2f valueSize = renderer_.MeasureText2D(rows[i].value, 22);
            renderer_.DrawText2D(rows[i].value, w * 0.5f + 280.f - valueSize.x, y, 22,
                                 selected ? Color::Yellow() : Color::Gray());
        }
        y += 46.f;
    }

    const std::string hint = "A/D CHANGE    W/S SELECT    ENTER/ESC BACK";
    const Vec2f hintSize = renderer_.MeasureText2D(hint, 18);
    renderer_.DrawText2D(hint, w * 0.5f - hintSize.x * 0.5f, h - 52.f, 18, Color::Gray());
}

}  // namespace apex
