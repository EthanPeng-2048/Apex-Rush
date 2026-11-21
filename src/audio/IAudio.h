#pragma once

namespace apex {

// Per-frame audio state pushed by the game (player car telemetry).
struct AudioState {
    float engineRpm = 0.f;
    float engineLoad = 0.f;   // 0..1 throttle
    float speed = 0.f;        // m/s
    float skid = 0.f;         // 0..1 tire slip
};

// Lightweight procedural audio: engine / skid / wind loops plus impact and
// UI one-shots. Backed by miniaudio (DirectSound/WASAPI/WinMM on Windows,
// CoreAudio/ALSA/Pulse/WebAudio elsewhere) so it stays cross-platform; the
// game layer only sees this interface (same pluggable pattern as IPhysicsWorld).
class IAudio {
public:
    virtual ~IAudio() = default;

    // backendName: "auto" (default) or a platform backend like "wasapi",
    // "directsound", "winmm", "alsa", "webaudio". Unknown names fall back.
    virtual bool Init(const char* backendName) = 0;
    virtual void Shutdown() = 0;

    // Live setting changes from the settings menu.
    virtual void SetMasterVolume(float volume01) = 0;
    virtual void SetBackend(const char* backendName) = 0;

    virtual void Update(const AudioState& state) = 0;
    virtual void PlayImpact(float strength01) = 0;
    virtual void PlayUIClick() = 0;
};

}  // namespace apex
