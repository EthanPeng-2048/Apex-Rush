#pragma once

#include <memory>

#include "audio/IAudio.h"

namespace apex {

struct MiniaudioAudioImpl;  // defined in the .cpp (holds miniaudio types)

// miniaudio implementation of IAudio. All miniaudio types stay inside the
// .cpp (compiled with MA_API static so the copy inside raylib never clashes).
class MiniaudioAudio final : public IAudio {
public:
    MiniaudioAudio();
    ~MiniaudioAudio() override;

    bool Init(const char* backendName) override;
    void Shutdown() override;
    void SetMasterVolume(float volume01) override;
    void SetBackend(const char* backendName) override;
    void Update(const AudioState& state) override;
    void PlayImpact(float strength01) override;
    void PlayUIClick() override;

private:
    std::unique_ptr<MiniaudioAudioImpl> impl_;
};

}  // namespace apex
