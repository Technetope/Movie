#pragma once

#include <cstddef>
#include <cstdint>

#include "aurora_wav.h"
#include "birds_wav.h"
#include "cicada_wav.h"
#include "crystal_wav.h"
#include "dolphin_wav.h"
#include "frog_wav.h"
#include "geyser_wav.h"
#include "insect_wav.h"
#include "reef_wav.h"

struct EmbeddedWav {
  const char* id;
  const uint8_t* data;
  size_t size;
};

// clang-format off
static const EmbeddedWav kEmbeddedWavs[] = {
    {"aurora",  aurora_wav,  aurora_wav_len},
    {"birds",   birds_wav,   birds_wav_len},
    {"cicada",  cicada_wav,  cicada_wav_len},
    {"crystal", crystal_wav, crystal_wav_len},
    {"dolphin", dolphin_wav, dolphin_wav_len},
    {"frog",    frog_wav,    frog_wav_len},
    {"geyser",  geyser_wav,  geyser_wav_len},
    {"insect",  insect_wav,  insect_wav_len},
    {"reef",    reef_wav,    reef_wav_len},
};
// clang-format on

static constexpr size_t kEmbeddedWavCount =
    sizeof(kEmbeddedWavs) / sizeof(kEmbeddedWavs[0]);
