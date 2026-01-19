// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: 2019-2024 Second State INC

#include "wasinn_piper.h"
#include "wasinnenv.h"

#ifdef WASMEDGE_PLUGIN_WASI_NN_BACKEND_PIPER
#include "simdjson.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <functional>
#include <ios>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>
#endif

namespace WasmEdge::Host::WASINN::Piper {
#ifdef WASMEDGE_PLUGIN_WASI_NN_BACKEND_PIPER

// Helper function to write WAV header
void writeWavHeader(int sampleRate, int16_t numChannels, int32_t numSamples,
                    std::vector<uint8_t> &outputBuffer) {
  int32_t byteRate = sampleRate * numChannels * sizeof(int16_t);
  int32_t dataSize = numSamples * numChannels * sizeof(int16_t);
  int32_t riffSize = 36 + dataSize;

  auto push_u32 = [&](int32_t val) {
    outputBuffer.push_back(val & 0xFF);
    outputBuffer.push_back((val >> 8) & 0xFF);
    outputBuffer.push_back((val >> 16) & 0xFF);
    outputBuffer.push_back((val >> 24) & 0xFF);
  };
  auto push_u16 = [&](int16_t val) {
    outputBuffer.push_back(val & 0xFF);
    outputBuffer.push_back((val >> 8) & 0xFF);
  };
  auto push_str = [&](const char *str) {
    for (int i = 0; i < 4; i++)
      outputBuffer.push_back(str[i]);
  };

  push_str("RIFF");
  push_u32(riffSize);
  push_str("WAVE");
  push_str("fmt ");
  push_u32(16);
  push_u16(1);
  push_u16(numChannels);
  push_u32(sampleRate);
  push_u32(byteRate);
  push_u16(numChannels * sizeof(int16_t));
  push_u16(16);
  push_str("data");
  push_u32(dataSize);
}

template <typename T>
std::tuple<WASINN::ErrNo, bool> getOption(simdjson::dom::object &Object,
                                          std::string_view Key, T &Result) {
  if (auto Error = Object[Key].get(Result)) {
    if (Error == simdjson::error_code::NO_SUCH_FIELD) {
      return {WASINN::ErrNo::Success, false};
    }
    spdlog::error(
        "[WASI-NN] Piper backend: Unable to retrieve the \"{}\" option: {}"sv,
        Key, simdjson::error_message(Error));
    return {WASINN::ErrNo::InvalidArgument, false};
  }
  return {WASINN::ErrNo::Success, true};
}

template <typename T, typename U = T>
WASINN::ErrNo getOptionalOption(simdjson::dom::object &Object,
                                std::string_view Key,
                                std::optional<T> &Result) {
  auto Value = U{};
  auto [Err, HasValue] = getOption(Object, Key, Value);
  if (HasValue) {
    Result = Value;
  }
  return Err;
}

WASINN::ErrNo parseSynthesisConfig(SynthesisConfig &SynthesisConfig,
                                   simdjson::dom::object &Object,
                                   const bool JsonInput) {
  {
    auto Value = std::optional<std::string_view>{};
    if (auto Err = getOptionalOption(Object, "output_type", Value);
        Err != WASINN::ErrNo::Success) {
      return Err;
    }
    if (Value) {
      if (Value.value() == "wav") {
        SynthesisConfig.OutputType = SynthesisConfigOutputType::OUTPUT_WAV;
      } else if (Value.value() == "raw") {
        SynthesisConfig.OutputType = SynthesisConfigOutputType::OUTPUT_RAW;
      } else {
        spdlog::error(
            "[WASI-NN] Piper backend: The output_type option has an unknown value {}."sv,
            Value.value());
        return WASINN::ErrNo::InvalidArgument;
      }
    }
  }

  if (JsonInput) {
    auto s_id = std::optional<int64_t>{};
    if (auto Err = getOptionalOption(Object, "speaker_id", s_id);
        Err != WASINN::ErrNo::Success) {
      return Err;
    }
    if (s_id.has_value()) {
      SynthesisConfig.SpeakerId = static_cast<int>(s_id.value());
    }
  } else {
    auto s_id = std::optional<int64_t>{};
    // Try "speaker" as an integer ID for legacy compatibility
    if (auto Err = getOptionalOption(Object, "speaker", s_id);
        Err == WASINN::ErrNo::Success) {
      if (s_id.has_value()) {
        SynthesisConfig.SpeakerId = static_cast<int>(s_id.value());
      }
    }
  }

  if (auto Err = getOptionalOption<float, double>(Object, "noise_scale",
                                                  SynthesisConfig.NoiseScale);
      Err != WASINN::ErrNo::Success) {
    return Err;
  }
  if (auto Err = getOptionalOption<float, double>(Object, "length_scale",
                                                  SynthesisConfig.LengthScale);
      Err != WASINN::ErrNo::Success) {
    return Err;
  }
  if (auto Err = getOptionalOption<float, double>(Object, "noise_w",
                                                  SynthesisConfig.NoiseW);
      Err != WASINN::ErrNo::Success) {
    return Err;
  }

  return WASINN::ErrNo::Success;
}

WASINN::ErrNo parseRunConfig(RunConfig &RunConfig,
                             const std::string &String) noexcept {
  simdjson::dom::parser Parser;
  simdjson::dom::element Doc;
  if (auto Error = Parser.parse(String).get(Doc)) {
    spdlog::error("[WASI-NN] Piper backend: Parse run config error: {}"sv,
                  simdjson::error_message(Error));
    return WASINN::ErrNo::InvalidEncoding;
  }
  simdjson::dom::object Object;
  if (auto Error = Doc.get(Object)) {
    spdlog::error(
        "[WASI-NN] Piper backend: The run config is not an object: {}"sv,
        simdjson::error_message(Error));
    return WASINN::ErrNo::InvalidArgument;
  }

  auto ModelPath = std::optional<std::string_view>{};
  if (auto Err = getOptionalOption(Object, "model", ModelPath);
      Err != WASINN::ErrNo::Success) {
    return Err;
  }
  if (ModelPath) {
    auto Path = std::filesystem::u8path(ModelPath.value());
    if (!std::filesystem::exists(Path)) {
      spdlog::error("[WASI-NN] Piper backend: Model file doesn't exist"sv);
      return WASINN::ErrNo::InvalidArgument;
    }
    RunConfig.ModelPath = Path;
  } else {
    spdlog::error(
        "[WASI-NN] Piper backend: The model option is required but not provided"sv);
    return WASINN::ErrNo::InvalidArgument;
  }

  auto ModelConfigPath = std::optional<std::string_view>{};
  if (auto Err = getOptionalOption(Object, "config", ModelConfigPath);
      Err != WASINN::ErrNo::Success) {
    return Err;
  }
  if (ModelConfigPath) {
    RunConfig.ModelConfigPath =
        std::filesystem::u8path(ModelConfigPath.value());
  } else {
    RunConfig.ModelConfigPath = RunConfig.ModelPath;
    RunConfig.ModelConfigPath += ".json";
  }
  if (!std::filesystem::exists(RunConfig.ModelConfigPath)) {
    spdlog::error("[WASI-NN] Piper backend: Model config doesn't exist"sv);
    return WASINN::ErrNo::InvalidArgument;
  }

  if (auto Err =
          parseSynthesisConfig(RunConfig.DefaultSynthesisConfig, Object, false);
      Err != WASINN::ErrNo::Success) {
    return Err;
  }
  {
    auto Path = std::optional<std::string_view>{};
    if (auto Err = getOptionalOption(Object, "espeak_data", Path);
        Err != WASINN::ErrNo::Success) {
      return Err;
    }
    if (Path) {
      RunConfig.ESpeakDataPath = std::filesystem::u8path(Path.value());
    }
  }
  {
    auto Path = std::optional<std::string_view>{};
    if (auto Err = getOptionalOption(Object, "tashkeel_model", Path);
        Err != WASINN::ErrNo::Success) {
      return Err;
    }
    if (Path) {
      RunConfig.TashkeelModelPath = std::filesystem::u8path(Path.value());
    }
  }
  if (auto Err =
          std::get<0>(getOption(Object, "json_input", RunConfig.JsonInput));
      Err != WASINN::ErrNo::Success) {
    return Err;
  }
  return WASINN::ErrNo::Success;
}

void updatePiperOptions(const SynthesisConfig &SynthesisConfig,
                        piper_synthesize_options &Options) {
  if (SynthesisConfig.SpeakerId) {
    Options.speaker_id = SynthesisConfig.SpeakerId.value();
  }
  if (SynthesisConfig.NoiseScale) {
    Options.noise_scale = SynthesisConfig.NoiseScale.value();
  }
  if (SynthesisConfig.LengthScale) {
    Options.length_scale = SynthesisConfig.LengthScale.value();
  }
  if (SynthesisConfig.NoiseW) {
    Options.noise_w_scale = SynthesisConfig.NoiseW.value();
  }
}

Expect<WASINN::ErrNo> load(WASINN::WasiNNEnvironment &Env,
                           Span<const Span<uint8_t>> Builders, WASINN::Device,
                           uint32_t &GraphId) noexcept {
  if (Builders.size() != 1) {
    spdlog::error(
        "[WASI-NN] Piper backend: Wrong GraphBuilder Length {:d}, expect 1"sv,
        Builders.size());
    return WASINN::ErrNo::InvalidArgument;
  }

  uint32_t GId = Env.newGraph(Backend::Piper);
  auto &GraphRef = Env.NNGraph[GId].get<Graph>();
  GraphRef.Config = std::make_unique<RunConfig>();

  auto String = std::string{Builders[0].begin(), Builders[0].end()};
  if (auto Res = parseRunConfig(*GraphRef.Config, String);
      Res != WASINN::ErrNo::Success) {
    Env.deleteGraph(GId);
    spdlog::error("[WASI-NN] Piper backend: Failed to parse run config."sv);
    return Res;
  }

  std::string espeakPath = "";
  if (GraphRef.Config->ESpeakDataPath) {
    espeakPath = GraphRef.Config->ESpeakDataPath->string();
  }

  piper_synthesizer *synth =
      piper_create(GraphRef.Config->ModelPath.string().c_str(),
                   GraphRef.Config->ModelConfigPath.string().c_str(),
                   espeakPath.empty() ? nullptr : espeakPath.c_str());

  if (!synth) {
    spdlog::error(
        "[WASI-NN] Piper backend: Failed to create piper synthesizer."sv);
    Env.deleteGraph(GId);
    return WASINN::ErrNo::InvalidArgument;
  }

  GraphRef.Synth = std::unique_ptr<piper_synthesizer, PiperDeleter>(synth);

  GraphId = GId;
  Env.NNGraph[GId].setReady();
  return WASINN::ErrNo::Success;
}

Expect<WASINN::ErrNo> initExecCtx(WASINN::WasiNNEnvironment &Env,
                                  uint32_t GraphId,
                                  uint32_t &ContextId) noexcept {
  ContextId = Env.newContext(GraphId, Env.NNGraph[GraphId]);
  Env.NNContext[ContextId].setReady();
  return WASINN::ErrNo::Success;
}

Expect<WASINN::ErrNo> setInput(WASINN::WasiNNEnvironment &Env,
                               uint32_t ContextId, uint32_t Index,
                               const TensorData &Tensor) noexcept {
  if (Index != 0) {
    spdlog::error("[WASI-NN] Piper backend: Input index must be 0."sv);
    return WASINN::ErrNo::InvalidArgument;
  }
  if (!(Tensor.Dimension.size() == 1 && Tensor.Dimension[0] == 1)) {
    spdlog::error(
        "[WASI-NN] Piper backend: Input tensor dimension must be [1]."sv);
    return WASINN::ErrNo::InvalidArgument;
  }

  auto &CxtRef = Env.NNContext[ContextId].get<Context>();
  auto Line = std::string{Tensor.Tensor.begin(), Tensor.Tensor.end()};
  auto &GraphRef = Env.NNGraph[CxtRef.GraphId].get<Graph>();

  if (GraphRef.Config->JsonInput) {
    simdjson::dom::parser Parser;
    simdjson::dom::element Doc;
    if (auto Error = Parser.parse(Line).get(Doc)) {
      spdlog::error("[WASI-NN] Piper backend: Parse json input error: {}"sv,
                    simdjson::error_message(Error));
      return WASINN::ErrNo::InvalidEncoding;
    }
    simdjson::dom::object Object;
    if (auto Error = Doc.get(Object)) {
      spdlog::error(
          "[WASI-NN] Piper backend: The json input is not an object: {}"sv,
          simdjson::error_message(Error));
      return WASINN::ErrNo::InvalidArgument;
    }

    auto Text = std::string_view{};
    if (auto Error = Object["text"].get(Text)) {
      spdlog::error(
          "[WASI-NN] Piper backend: Unable to retrieve required \"text\" from json input: {}"sv,
          simdjson::error_message(Error));
      return WASINN::ErrNo::InvalidArgument;
    }
    Line = Text;

    auto JsonInputSynthesisConfig = SynthesisConfig{};
    if (auto Err = parseSynthesisConfig(JsonInputSynthesisConfig, Object, true);
        Err != WASINN::ErrNo::Success) {
      return Err;
    }

    if (!CxtRef.JsonInputSynthesisConfig) {
      CxtRef.JsonInputSynthesisConfig =
          std::make_unique<std::optional<SynthesisConfig>>();
    }
    *CxtRef.JsonInputSynthesisConfig = JsonInputSynthesisConfig;
  }
  CxtRef.Line = Line;
  return WASINN::ErrNo::Success;
}

Expect<WASINN::ErrNo> getOutput(WASINN::WasiNNEnvironment &Env,
                                uint32_t ContextId, uint32_t Index,
                                Span<uint8_t> OutBuffer,
                                uint32_t &BytesWritten) noexcept {
  if (Index != 0) {
    spdlog::error("[WASI-NN] Piper backend: Output index must be 0."sv);
    return WASINN::ErrNo::InvalidArgument;
  }

  auto &CxtRef = Env.NNContext[ContextId].get<Context>();

  if (!CxtRef.Output) {
    spdlog::error("[WASI-NN] Piper backend: No output available."sv);
    return WASINN::ErrNo::InvalidArgument;
  }

  if (CxtRef.Output->size() > OutBuffer.size_bytes()) {
    spdlog::error(
        "[WASI-NN] Piper backend: Output size {} is greater than buffer size {}."sv,
        CxtRef.Output->size(), OutBuffer.size_bytes());
    return WASINN::ErrNo::InvalidArgument;
  }

  std::memcpy(OutBuffer.data(), CxtRef.Output->data(), CxtRef.Output->size());
  BytesWritten = CxtRef.Output->size();
  return WASINN::ErrNo::Success;
}

Expect<WASINN::ErrNo> compute(WASINN::WasiNNEnvironment &Env,
                              uint32_t ContextId) noexcept {
  auto &CxtRef = Env.NNContext[ContextId].get<Context>();
  auto &GraphRef = Env.NNGraph[CxtRef.GraphId].get<Graph>();

  if (!CxtRef.Line) {
    spdlog::error("[WASI-NN] Piper backend: Input is not set."sv);
    return WASINN::ErrNo::InvalidArgument;
  }

  piper_synthesize_options options =
      piper_default_synthesize_options(GraphRef.Synth.get());
  updatePiperOptions(GraphRef.Config->DefaultSynthesisConfig, options);

  auto OutputType = SynthesisConfigOutputType::OUTPUT_WAV;
  if (GraphRef.Config->DefaultSynthesisConfig.OutputType) {
    OutputType = GraphRef.Config->DefaultSynthesisConfig.OutputType.value();
  }

  if (CxtRef.JsonInputSynthesisConfig &&
      CxtRef.JsonInputSynthesisConfig->has_value()) {
    updatePiperOptions(CxtRef.JsonInputSynthesisConfig->value(), options);
    if (CxtRef.JsonInputSynthesisConfig->value().OutputType) {
      OutputType = CxtRef.JsonInputSynthesisConfig->value().OutputType.value();
    }
  }

  int res = piper_synthesize_start(GraphRef.Synth.get(),
                                   CxtRef.Line.value().c_str(), &options);
  if (res != PIPER_OK) {
    spdlog::error("[WASI-NN] Piper backend: piper_synthesize_start failed."sv);
    return WASINN::ErrNo::RuntimeError;
  }

  std::vector<int16_t> audioBuffer;
  piper_audio_chunk chunk;
  int sampleRate = 0;
  const float MAX_WAV_VALUE = 32767.0f;

  while (piper_synthesize_next(GraphRef.Synth.get(), &chunk) != PIPER_DONE) {
    if (chunk.num_samples == 0)
      continue;
    sampleRate = chunk.sample_rate;
    audioBuffer.reserve(audioBuffer.size() + chunk.num_samples);

    for (size_t i = 0; i < chunk.num_samples; i++) {
      float sample = chunk.samples[i];
      if (sample > 1.0f)
        sample = 1.0f;
      else if (sample < -1.0f)
        sample = -1.0f;
      audioBuffer.push_back(static_cast<int16_t>(sample * MAX_WAV_VALUE));
    }
  }

  CxtRef.Output = std::vector<uint8_t>{};

  if (OutputType == SynthesisConfigOutputType::OUTPUT_WAV) {
    if (sampleRate == 0)
      sampleRate = 22050;

    size_t totalSize = 44 + (audioBuffer.size() * sizeof(int16_t));
    CxtRef.Output->reserve(totalSize);

    writeWavHeader(sampleRate, 1, audioBuffer.size(), *CxtRef.Output);

    const uint8_t *rawData =
        reinterpret_cast<const uint8_t *>(audioBuffer.data());
    CxtRef.Output->insert(CxtRef.Output->end(), rawData,
                          rawData + (audioBuffer.size() * sizeof(int16_t)));

  } else {
    size_t totalSize = audioBuffer.size() * sizeof(int16_t);
    CxtRef.Output->resize(totalSize);
    std::memcpy(CxtRef.Output->data(), audioBuffer.data(), totalSize);
  }

  return WASINN::ErrNo::Success;
}
#else
namespace {
Expect<WASINN::ErrNo> reportBackendNotSupported() noexcept {
  spdlog::error("[WASI-NN] Piper backend is not supported."sv);
  return WASINN::ErrNo::InvalidArgument;
}
} // namespace

Expect<WASINN::ErrNo> load(WASINN::WasiNNEnvironment &,
                           Span<const Span<uint8_t>>, WASINN::Device,
                           uint32_t &) noexcept {
  return reportBackendNotSupported();
}
Expect<WASINN::ErrNo> initExecCtx(WASINN::WasiNNEnvironment &, uint32_t,
                                  uint32_t &) noexcept {
  return reportBackendNotSupported();
}
Expect<WASINN::ErrNo> setInput(WASINN::WasiNNEnvironment &, uint32_t, uint32_t,
                               const TensorData &) noexcept {
  return reportBackendNotSupported();
}
Expect<WASINN::ErrNo> getOutput(WASINN::WasiNNEnvironment &, uint32_t, uint32_t,
                                Span<uint8_t>, uint32_t &) noexcept {
  return reportBackendNotSupported();
}
Expect<WASINN::ErrNo> compute(WASINN::WasiNNEnvironment &, uint32_t) noexcept {
  return reportBackendNotSupported();
}
#endif
} // namespace WasmEdge::Host::WASINN::Piper
