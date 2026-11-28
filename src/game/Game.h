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
#include "game/net/NetCars.h"
#include "net/discovery.h"
#include "net/identity.h"
#include "net/session.h"
#include "net/transport.h"
import rush.engine.physics;
import rush.engine.render;
import rush.engine.input;
import rush.engine.runtime;
import rush.engine.ui;

namespace apex {

// Declares the input events the game consumes (event name + gesture + keys).
// Called by Game::Init; tests may also call it directly before PumpOnce.
void RegisterGameInput(InputSystem& input);

// Race phases drive UI and input gating.
// NOTE: the numeric order of the first six values is asserted by --drive-test
// and by the shot automation; LAN is appended last so those stay stable.
enum class RacePhase {
    Menu,
    Settings,
    RaceSetup,   // pre-race config (circuit / laps / cars / damage)
    Countdown,
    Racing,
    Finished,
    Lan,         // LAN room browser / lobby (sub-state lives in LanUi)
};

struct Racer {
    Vehicle vehicle;
    CarInput input;
    std::string name;
    // LAN identity. `playerId` is hashed from the player's name (net/identity.h)
    // and `seat` is that id's rank in the frozen roster — also the grid slot.
    // Offline there is no roster, so the seat is simply the spawn index.
    PlayerId playerId = kInvalidPlayerId;
    int seat = 0;
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

// The only starting-pose formula in the game: the grid box for a racer's seat.
// Spawning, the Enter restart, the LAN RaceStarted reset and the host's track
// rebuild all come through here, so a car can never be dropped on a box that
// belongs to someone else.
Transform GridPose(const Track& track, const Racer& racer);

// Owns the world, track, racers and UI. One instance per running game.
class Game {
public:
    // LAN role. Offline = the original single-player flow.
    enum class NetRole { Offline, Host, Client };
    // LAN screen sub-state (all under RacePhase::Lan).
    // RoomCode is the shared room-code editor: HOST and JOIN both enter it and
    // the role only decides what a confirmed code does. NameEntry is the login
    // screen that runs before any of them.
    enum class LanUi { Root, RoomCode, RoomList, Lobby, Message, NameEntry };
    // Which role a confirmed room code launches.
    enum class LanRole { None, Host, Join };

    // Race-setup rows: circuit, laps, cars, damage, start, back.
    static constexpr int kRaceSetupRowCount = 6;
    // LAN root rows: host, join, back.
    static constexpr int kLanRootRows = 3;
    // LAN lobby rows: start (host) / leave.
    static constexpr int kLanLobbyRows = 2;
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

    NetRole Role() const { return netRole_; }
    bool LanActive() const { return netRole_ != NetRole::Offline; }
    // Opens the LAN screen (main menu entry point).
    void EnterLan();
    // Headless automation: start a host / join a room immediately (used by
    // --net-host / --net-join; no menu interaction).
    bool AutoHost(std::uint16_t roomCode);
    bool AutoJoin(std::uint16_t roomCode, const std::string& hostIp);
    // Highest-priority player name (CLI --player). Empty = not given.
    void SetPlayerName(const std::string& name) { playerNameCli_ = name; }

    // This machine's own car. Every HUD / camera / audio / input / prediction
    // path goes through it: with a roster the local player is not necessarily
    // racers_[0] (the host included).
    Racer& LocalRacer();
    const Racer& LocalRacer() const;
    // Seat of this machine's player (offline = 0).
    int LocalSeat() const {
        return netRole_ == NetRole::Offline
                   ? 0
                   : static_cast<int>(netLocalSeat_.load(std::memory_order_relaxed));
    }
    // Single lookup entry for a seat, tolerant of roster holes and of the grid
    // not existing yet (nullptr).
    Racer* RacerBySlot(std::uint8_t seat);
    // Grid box for `racer` under this track (see GridPose).
    Transform GridPose(const Racer& racer) const;
    // Park one car on its own grid box and clear everything the race owns.
    void ResetRacerForStart(Racer& racer);
    void ResetGridForStart();

    // Introspection for tests and (later) the network layer.
    const VehicleState& PlayerState() const { return LocalRacer().vehicle.State(); }
    const std::vector<Racer>& Racers() const { return racers_; }
    RacePhase Phase() const { return phase_; }
    float RaceClock() const { return raceClock_; }
    const Track& GetTrack() const { return track_; }
    // Physics world (diagnostics / QA hooks). Null before the first race build.
    const IPhysicsWorld* World() const { return world_.get(); }
    // QA hook (--stress): drop every car into one small area at speed so the
    // collision path gets hammered the way a mid-race pile-up does.
    void StressPileUp(float radius, float speed);
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
        // Per-sample times [ms] for the worst-percentile (1% low) analysis:
        // one entry per frame (update section) and per physics substep
        // (world Step section). Sorted at report time in RunBench.
        std::vector<double> frameTimesMs;
        std::vector<double> stepTimesMs;
    };
    void SetBench(bool on) {
        benchOn_ = on;
        // Bench runs the parallel-island path (bit-identical numerics, only
        // execution order changes; drive-test never enables it) and needs the
        // physics stage timers. Applied to the current world, and re-applied
        // whenever a race build replaces it.
        ApplyWorldTuning();
    }
    void ResetBench() {
        bench_ = BenchSample{};
        // Stage means live in the physics world and would otherwise keep the
        // countdown/settle warm-up steps, which have almost no contacts and
        // would drag every stage down against the measured window.
        if (world_) world_->ResetStageStats();
    }
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
        // Index into `cars` of this machine's own car (see LocalRacer).
        int localIndex = 0;
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
        // LAN grid: the frozen roster (empty offline). When set, cars are built
        // from seats instead of `carCount` synthetic AI entries, and `lanClient`
        // spawns only the local player's car.
        net::Roster roster{};
        bool lanClient = false;
        int localSeat = 0;
        PlayerId localPlayerId = kInvalidPlayerId;
        bool damage = true;
        AssistLevel absLevel = AssistLevel::Sport;
        AssistLevel tcLevel = AssistLevel::Sport;
        AssistLevel escLevel = AssistLevel::Full;
        AssistLevel steerLevel = AssistLevel::Full;
        bool seasonDrs = true;
        bool seasonAero = false;
        bool seasonOverride = false;
        bool seasonErs = false;
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
    void ApplyWorldTuning();
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

    // --- LAN -----------------------------------------------------------------
    void UpdateLan();
    void RenderLan();
    // Name entry (lanUi_ == LanUi::NameEntry). The screen itself lives in
    // screens_.lanName like every other screen; these only open it and handle
    // the action it reports.
    void EnterLanNameEntry();
    void UpdateLanNameEntry();
    // Name used for this session: --player > [net] player_name > the name typed
    // on the login screen. Empty means "nobody chose one yet".
    std::string ChosenPlayerName() const;
    // ChosenPlayerName(), else a role default (HOST / PLAYER-<n>) so unattended
    // runs never need the login screen.
    std::string EffectivePlayerName(bool host) const;
    void LanStartHost(std::uint16_t roomCode);
    void LanStartClient(std::uint16_t roomCode, const net::Endpoint& host);
    void LanStop(const std::string& message);
    std::unique_ptr<net::IRoomDiscovery> MakeDiscovery();   // config -> discovery options
    void LanTick();                       // discovery + session pump + events
    void LanHandleEvents();
    void LanRequestRaceLoad();            // host: freeze the roster + build the race
    void LanBeginClientRace();            // client: enter the race on RaceStart
    void LanApplyRemoteInputs();          // host sim thread
    void LanClientSimTick(float frameDt); // client sim thread (own car + proxies)
    void LanPublishHostWorld();           // main thread: hand the world to the session
    void LanBuildHostWorld(net::INetSession::HostWorld& out);  // sim thread
    net::InputSample LanSampleLocalInput();    // main thread: InputState -> wire sample
    void LanApplySessionToRacer(std::uint8_t slot);           // HUD mirrors
    double LanRenderTimeMs() const;
    // Client seat/car count from the host's frozen roster; copies the rules that
    // decide the physics (season gates, assists, damage) so both ends simulate
    // the same car.
    void ApplyHostRules(const net::RulesPacket& rules);
    // Host: local config + season gates -> the rules clients must run.
    void FillHostRules(net::RulesPacket& rules) const;

    NetRole netRole_ = NetRole::Offline;
    LanUi lanUi_ = LanUi::Root;
    std::string lanMessage_;
    float lanMessageTimer_ = 0.f;
    std::uint64_t lanJoinStartMs_ = 0;

    std::unique_ptr<net::ITransport> netTransport_;
    std::unique_ptr<net::IRoomDiscovery> netDiscovery_;
    std::unique_ptr<net::INetSession> netSession_;
    net::NetConfig netConfig_{};
    std::unique_ptr<RemoteInterpolator> netInterp_;
    std::unique_ptr<LocalPredictor> netPredict_;
    std::unique_ptr<RemoteProxies> netProxies_;

    std::uint32_t localTick_ = 0;
    std::uint32_t hostTick_ = 0;
    // Seat of this machine's player. Written by the handshake events on the main
    // thread and read by the sim thread (LocalRacer), hence atomic.
    std::atomic<std::uint8_t> netLocalSeat_{0};
    std::uint8_t netPressDrsSeen_ = 0;
    std::uint8_t netPressWingSeen_ = 0;
    std::array<std::uint8_t, net::kMaxCars> lastDrsPress_{};
    std::array<std::uint8_t, net::kMaxCars> lastWingPress_{};
    std::uint32_t lastPushedHostTick_ = 0;
    bool netEndSent_ = false;
    std::uint64_t netLogMs_ = 0;
    std::uint16_t localInputSeq_ = 0;
    net::InputSample netLocalSample_{};
    std::uint8_t netPressDrs_ = 0;
    std::uint8_t netPressWing_ = 0;
    std::uint8_t netPressOvertake_ = 0;
    std::array<net::LobbyPlayer, net::kMaxCars> netLobby_{};
    std::uint8_t netLobbyCount_ = 0;
    std::uint8_t netCarCount_ = 1;
    net::Roster netRoster_{};   // frozen at race start, shared by both roles

    // Player names, in resolution order (see ChosenPlayerName).
    std::string playerNameCli_;     // --player
    std::string lanPlayerName_;     // typed on the login screen
    int lanJoinOrdinal_ = 0;        // PLAYER-<n> default for unattended joins
    // Identity of this machine's player, hashed once when the LAN session opens.
    PlayerId localPlayerId_ = kInvalidPlayerId;

    // Client-side copy of the host's rules. They decide the physics (season
    // gates, assists, damage) and the seat layout, so a client on a different
    // local config still drives the host's car.
    bool netRulesLocked_ = false;
    net::RulesPacket netRules_{};
    // Remote inputs are received on the main thread and consumed by the sim
    // thread (same mailbox pattern as PlayerIntent).
    std::mutex netInputMutex_;
    std::array<net::InputSample, net::kMaxCars> netRemoteInput_{};
    std::array<bool, net::kMaxCars> netRemoteFresh_{};
    std::array<bool, net::kMaxCars> netDrsLatch_{};
    bool clientDrsLatch_ = false;
    float clientWingLatch_ = 0.f;
    std::uint64_t netLastHostSendMs_ = 0;
    net::INetSession::HostWorld hostWorld_{};
    std::mutex hostWorldMutex_;
    bool hostWorldFresh_ = false;
    double netRenderDelayMs_ = 40.0;
    // Authoritative race readout used by the client HUD (lap/position/clock).
    struct NetRaceReadout {
        bool valid = false;
        std::uint8_t lap = 0;
        std::uint8_t position = 0;
        std::uint8_t phase = 0;
        std::uint32_t raceClockMs = 0;
        std::uint8_t carCount = 1;
    } netReadout_;

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
    // ERS is its own gate: the 2026 preset turns it on with overtake, but a
    // user can decouple them via [season] ers without touching the preset.
    bool seasonErs_ = false;

    // Menu / settings state.
    InputState rawInput_;

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

    // --- Declarative menus (rush.engine.ui) ---------------------------------
    // A UiScreen is owned state, not immediate mode: the focus index, the
    // per-row focus flags, the digit sub-cursor and text buffers all live in
    // it. So every screen is a member that is built when it opens and only has
    // its dynamic text refreshed afterwards — rebuilding per frame would reset
    // the focus. The engine resolves input and draws; the game declares rows
    // and reacts to the UiAction that comes back.
    struct UiScreens {
        UiScreen menu;
        UiScreen settings;
        UiScreen raceSetup;
        UiScreen lanRoot;
        UiScreen lanCode;   // the shared room-code wheel (HOST and JOIN)
        UiScreen lanRooms;  // discovered rooms: rows change with the LAN
        UiScreen lanLobby;
        UiScreen lanMessage;
        UiScreen lanName;   // login: one text field, nothing else fits beside it
    };
    UiScreens screens_;
    UiController ui_;
    RacePhase uiPhase_ = RacePhase::Countdown;  // phase the open screens belong to
    LanUi lanScreenState_ = LanUi::Root;        // LAN sub-state last built
    bool lanScreensStale_ = true;               // LAN entered: rebuild + refocus
    NetRole lanLobbyRole_ = NetRole::Offline;   // role the lobby rows were built for
    LanRole lanPendingRole_ = LanRole::None;    // what a confirmed room code does
    std::vector<net::RoomInfo> lanRooms_;       // last filtered discovery result
    std::uint64_t lanQueryNextMs_ = 0;          // next discovery query (room browser)

    // Open the screen a phase change needs (focus reset is a screen open).
    void SyncUiPhase();
    void OpenPhaseScreen(RacePhase phase);
    void BuildMenuScreen();
    void BuildSettingsScreen();
    void BuildRaceSetupScreen();
    void RefreshSettingsValues();
    void RefreshRaceSetupValues();
    // LAN: static skeletons once per visit, dynamic rows refreshed in place.
    void BuildLanScreens();
    void SyncLanScreen();
    UiScreen& ActiveLanScreen();
    std::uint16_t LanRoomCode();
    void LanOpenRoomCode(LanRole role);
    void LanConfirmRoomCode();
    void LanLeaveToMenu();
    void ApplySettingStep(const std::string& id, int step);
    void ApplyRaceSetupStep(const std::string& id, int step);
};

}  // namespace apex
