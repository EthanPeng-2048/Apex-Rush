#include "audio/MiniaudioAudio.h"

#include <atomic>
#include <cmath>
#include <cstring>
#include <string>

// One private copy of miniaudio, fully static: raylib links its own copy
// internally, so the public API must not leak external symbols here.
#define MA_API static
#define MA_IMPLEMENTATION
#include "miniaudio.h"

namespace apex {

namespace {

constexpr float kSampleRate = 48000.f;

float Clamp01(float v) { return v < 0.f ? 0.f : (v > 1.f ? 1.f : v); }

// "auto" -> use miniaudio's default backend ordering (WASAPI first on
// Windows, so DirectSound is the fallback); explicit names force one.
bool ParseBackend(const char* name, ma_backend& out) {
    if (!name || !name[0] || std::strcmp(name, "auto") == 0) return false;
    if (std::strcmp(name, "wasapi") == 0) { out = ma_backend_wasapi; return true; }
    if (std::strcmp(name, "directsound") == 0 || std::strcmp(name, "dsound") == 0) {
        out = ma_backend_dsound;
        return true;
    }
    if (std::strcmp(name, "winmm") == 0) { out = ma_backend_winmm; return true; }
    if (std::strcmp(name, "coreaudio") == 0) { out = ma_backend_coreaudio; return true; }
    if (std::strcmp(name, "alsa") == 0) { out = ma_backend_alsa; return true; }
    if (std::strcmp(name, "pulseaudio") == 0 || std::strcmp(name, "pulse") == 0) {
        out = ma_backend_pulseaudio;
        return true;
    }
    if (std::strcmp(name, "webaudio") == 0) { out = ma_backend_webaudio; return true; }
    return false;
}

}  // namespace

struct MiniaudioAudioImpl {
    ma_context context{};
    ma_device device{};
    bool hasContext = false;
    bool hasDevice = false;

    // Game thread writes, audio thread reads.
    std::atomic<float> rpm{800.f};
    std::atomic<float> load{0.f};
    std::atomic<float> speed{0.f};
    std::atomic<float> skid{0.f};
    std::atomic<float> master{0.8f};
    std::atomic<float> impact{-1.f};
    std::atomic<float> click{0.f};

    // Audio-thread-only synth state.
    float enginePhase = 0.f;
    float enginePhase2 = 0.f;
    float subPhase = 0.f;
    float skidLp = 0.f;
    float windLp = 0.f;
    float impactEnv = 0.f;
    float impactTime = 0.f;
    float clickEnv = 0.f;
    float clickTime = 0.f;
    unsigned rng = 0x2545F491u;
    std::string backendName = "auto";
};

namespace {

float NextNoise(MiniaudioAudioImpl& im) {
    im.rng = im.rng * 1664525u + 1013904223u;
    return static_cast<float>(im.rng >> 8) / 8388608.f - 1.f;
}

// Procedural mix: engine (2 saws + sub, pitch from rpm), wind (lowpassed
// noise by speed), skid (bright noise by slip) plus impact/click one-shots.
void AudioCallback(ma_device* device, void* output, const void*, ma_uint32 frameCount) {
    auto& im = *static_cast<MiniaudioAudioImpl*>(device->pUserData);
    float* dst = static_cast<float*>(output);

    const float master = im.master.load(std::memory_order_relaxed);
    const float load = im.load.load(std::memory_order_relaxed);
    const float speed = im.speed.load(std::memory_order_relaxed);
    const float skid = im.skid.load(std::memory_order_relaxed);
    const float rpm = im.rpm.load(std::memory_order_relaxed);

    const float impactReq = im.impact.exchange(-1.f, std::memory_order_relaxed);
    if (impactReq >= 0.f) {
        im.impactEnv = 0.55f + 0.45f * impactReq;
        im.impactTime = 0.f;
    }
    if (im.click.exchange(0.f, std::memory_order_relaxed) > 0.f) {
        im.clickEnv = 0.5f;
        im.clickTime = 0.f;
    }

    constexpr float kDt = 1.f / kSampleRate;
    // Firing frequency ~3 power strokes per crank revolution (V6-ish).
    const float freq = Clamp01(rpm / 12500.f) * 480.f + 18.f;
    const float engineAmp = master * (0.16f + 0.34f * load);
    const float speedFrac = Clamp01(speed / 80.f);
    const float windAmp = master * 0.30f * speedFrac * speedFrac;
    const float skidAmp = master * 0.55f * Clamp01(skid);

    for (ma_uint32 i = 0; i < frameCount; ++i) {
        const float n = NextNoise(im);

        im.enginePhase += freq * kDt;
        if (im.enginePhase >= 1.f) im.enginePhase -= 1.f;
        im.enginePhase2 += freq * 2.02f * kDt;
        if (im.enginePhase2 >= 1.f) im.enginePhase2 -= 1.f;
        im.subPhase += freq * 0.5f * kDt;
        if (im.subPhase >= 1.f) im.subPhase -= 1.f;
        const float saw = 2.f * im.enginePhase - 1.f;
        const float saw2 = 2.f * im.enginePhase2 - 1.f;
        const float sub = std::sin(im.subPhase * 6.2831853f);
        const float engine =
            (saw * 0.30f + saw2 * 0.11f + sub * 0.26f + n * 0.05f) * engineAmp;

        im.windLp += (n - im.windLp) * 0.045f;
        const float wind = (n * 0.7f + im.windLp * 0.6f) * windAmp;

        im.skidLp += (n - im.skidLp) * 0.28f;
        const float skidSnd = (n - im.skidLp) * skidAmp;

        float impactSnd = 0.f;
        if (im.impactEnv > 0.001f) {
            im.impactTime += kDt;
            const float env = im.impactEnv * std::exp(-im.impactTime * 8.f);
            impactSnd =
                (n * 0.75f + std::sin(im.impactTime * 130.f) * 0.55f) * env * master * 0.85f;
            im.impactEnv -= kDt * 2.4f;
        }

        float clickSnd = 0.f;
        if (im.clickEnv > 0.001f) {
            im.clickTime += kDt;
            clickSnd = std::sin(im.clickTime * 2200.f) * im.clickEnv * master * 0.30f;
            im.clickEnv -= kDt * 7.f;
        }

        float s = engine + wind + skidSnd + impactSnd + clickSnd;
        if (s > 1.f) s = 1.f;
        else if (s < -1.f) s = -1.f;
        dst[i * 2 + 0] = s;
        dst[i * 2 + 1] = s;
    }
}

}  // namespace

MiniaudioAudio::MiniaudioAudio() : impl_(std::make_unique<MiniaudioAudioImpl>()) {}

MiniaudioAudio::~MiniaudioAudio() { Shutdown(); }

bool MiniaudioAudio::Init(const char* backendName) {
    if (impl_->hasDevice) return true;
    impl_->backendName = backendName ? backendName : "auto";

    ma_context_config ctxCfg = ma_context_config_init();
    ma_backend forced;
    if (ParseBackend(impl_->backendName.c_str(), forced)) {
        if (ma_context_init(&forced, 1, &ctxCfg, &impl_->context) != MA_SUCCESS) {
            // Forced backend unavailable (e.g. wasapi in a session without
            // audio): fall back to the default ordering.
            if (ma_context_init(nullptr, 0, &ctxCfg, &impl_->context) != MA_SUCCESS) return false;
        }
    } else if (ma_context_init(nullptr, 0, &ctxCfg, &impl_->context) != MA_SUCCESS) {
        return false;
    }
    impl_->hasContext = true;

    ma_device_config cfg = ma_device_config_init(ma_device_type_playback);
    cfg.playback.format = ma_format_f32;
    cfg.playback.channels = 2;
    cfg.sampleRate = static_cast<ma_uint32>(kSampleRate);
    cfg.dataCallback = AudioCallback;
    cfg.pUserData = impl_.get();
    if (ma_device_init(&impl_->context, &cfg, &impl_->device) != MA_SUCCESS) {
        ma_context_uninit(&impl_->context);
        impl_->hasContext = false;
        return false;
    }
    impl_->hasDevice = true;
    ma_device_start(&impl_->device);
    return true;
}

void MiniaudioAudio::Shutdown() {
    if (impl_->hasDevice) {
        ma_device_uninit(&impl_->device);
        impl_->hasDevice = false;
    }
    if (impl_->hasContext) {
        ma_context_uninit(&impl_->context);
        impl_->hasContext = false;
    }
}

void MiniaudioAudio::SetMasterVolume(float volume01) {
    impl_->master.store(Clamp01(volume01), std::memory_order_relaxed);
}

void MiniaudioAudio::SetBackend(const char* backendName) {
    const std::string name = backendName ? backendName : "auto";
    if (name == impl_->backendName) return;
    Shutdown();
    Init(name.c_str());
}

void MiniaudioAudio::Update(const AudioState& state) {
    impl_->rpm.store(state.engineRpm, std::memory_order_relaxed);
    impl_->load.store(Clamp01(state.engineLoad), std::memory_order_relaxed);
    impl_->speed.store(state.speed, std::memory_order_relaxed);
    impl_->skid.store(Clamp01(state.skid), std::memory_order_relaxed);
}

void MiniaudioAudio::PlayImpact(float strength01) {
    float current = impl_->impact.load(std::memory_order_relaxed);
    const float strength = Clamp01(strength01);
    // Keep the loudest hit of the frame.
    while (strength > current &&
           !impl_->impact.compare_exchange_weak(current, strength, std::memory_order_relaxed)) {
    }
}

void MiniaudioAudio::PlayUIClick() {
    impl_->click.store(1.f, std::memory_order_relaxed);
}

}  // namespace apex
