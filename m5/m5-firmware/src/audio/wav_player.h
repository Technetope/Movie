#pragma once

#include <M5Unified.h>

#include <string>

#include "../wav/embedded_wav.h"

class WavPlayer {
 public:
  void Begin(uint8_t volume = 200) {
    volume_ = volume;
    M5.Speaker.stop();
    M5.Speaker.setVolume(volume_);
    initialized_ = true;
  }

  void Loop() {
    if (!initialized_) return;
    if (!M5.Speaker.isPlaying()) {
      current_id_.clear();
    }
  }

  void Stop() {
    if (!initialized_) return;
    M5.Speaker.stop();
    current_id_.clear();
  }

  bool PlayById(const std::string& id) {
    if (!initialized_) return false;
    const auto* wav = Find(id);
    if (!wav) {
      M5.Log.printf("[wav] missing id: %s\n", id.c_str());
      return false;
    }
    // Last-write-wins: stop current, then play the new sound.
    M5.Speaker.stop();
    M5.Speaker.setVolume(volume_);
    if (!M5.Speaker.playWav(wav->data, wav->size, 1, 0, true)) {
      M5.Log.println("[wav] playWav rejected");
      return false;
    }
    current_id_ = id;
    return true;
  }

 private:
  const EmbeddedWav* Find(const std::string& id) const {
    for (size_t i = 0; i < kEmbeddedWavCount; ++i) {
      if (id == kEmbeddedWavs[i].id) {
        return &kEmbeddedWavs[i];
      }
    }
    return nullptr;
  }

  bool initialized_ = false;
  uint8_t volume_ = 200;
  std::string current_id_;
};
