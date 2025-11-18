#include "protocol/protocol_handler.h"

#include <ArduinoJson.h>
#include <esp_system.h>
#include <M5Unified.h>
#include <cstring>
#include <vector>

namespace {
constexpr uint32_t kDefaultScanDurationSec = 3;

// Utility to read optional string field.
const char* ReadString(const JsonVariantConst& value) {
  if (value.is<const char*>()) {
    return value.as<const char*>();
  }
  return nullptr;
}
}  // namespace

ProtocolHandler::ProtocolHandler(CommandDispatcher& commands,
                                 SendCallback sender)
    : commands_(commands), send_(std::move(sender)) {
  commands_.SetTimelineCallback(
      [this](size_t index, const ToioController::TimelineFrame& frame) {
        StaticJsonDocument<192> doc;
        doc["type"] = "timeline-apply";
        doc["index"] = static_cast<uint32_t>(index);
        doc["t"] = frame.time_s;
        doc["up"] = frame.use_position;
        doc["uh"] = frame.use_heading;
        doc["x"] = frame.x;
        doc["y"] = frame.y;
        doc["ang"] = frame.angle_deg;
        std::string payload;
        serializeJson(doc, payload);
        send_(payload);
      });
}

void ProtocolHandler::HandleClientConnected() {
  connected_ = true;
  commands_.SetStatusSubscription(false);
  SendHello();
}

void ProtocolHandler::HandleClientDisconnected() {
  connected_ = false;
  commands_.SetStatusSubscription(false);
  commands_.StopTimeline(true);
  commands_.Disconnect();
  esp_restart();  // WebSocket断時に自動リセット
}

void ProtocolHandler::HandleMessage(const std::string& payload) {
  // Increased capacity to allow large timeline-load (up to ~240 frames).
  StaticJsonDocument<16384> doc;
  const auto err = deserializeJson(doc, payload);
  const char* id = doc["id"];
  if (err) {
    SendError(id, "json-parse-failed");
    return;
  }
  const char* type = doc["type"];
  if (!type) {
    SendError(id, "missing-type");
    return;
  }

  if (strcmp(type, "scan") == 0) {
    auto result = commands_.Scan(kDefaultScanDurationSec);
    SendScanResult(id, result);
    return;
  }

  if (strcmp(type, "connect") == 0) {
    const char* suffix = ReadString(doc["suffix"]);
    std::string suffix_str = suffix ? suffix : "";
    auto status = commands_.Connect(suffix_str);
    SendConnectResult(id, suffix_str, status);
    return;
  }

  if (strcmp(type, "disconnect") == 0) {
    commands_.Disconnect();
    SendAck("disconnect-result", id, true);
    return;
  }

  if (strcmp(type, "led") == 0) {
    int r = doc["r"] | -1;
    int g = doc["g"] | -1;
    int b = doc["b"] | -1;
    if (r < 0 || r > 255 || g < 0 || g > 255 || b < 0 || b > 255) {
      SendError(id, "invalid-led");
      return;
    }
    bool ok = commands_.SetLed(static_cast<uint8_t>(r),
                               static_cast<uint8_t>(g),
                               static_cast<uint8_t>(b));
    SendAck("led-result", id, ok, ok ? nullptr : "no-active-core");
    return;
  }

  if (strcmp(type, "motor") == 0) {
    int left = doc["left"] | -101;
    int right = doc["right"] | -101;
    if (left < -100 || left > 100 || right < -100 || right > 100) {
      SendError(id, "invalid-motor");
      return;
    }
    bool ok = commands_.DriveMotor(static_cast<int8_t>(left),
                                   static_cast<int8_t>(right));
    SendAck("motor-result", id, ok, ok ? nullptr : "no-active-core");
    return;
  }

  if (strcmp(type, "goal-set") == 0) {
    const bool use_position = doc["use_position"] | true;
    const bool use_heading =
        doc["use_rotation"] | doc["use_heading"] | false;  // use_rotation を優先
    if (!use_position && !use_heading) {
      SendError(id, "invalid-goal");
      return;
    }

    float x = 0.0f;
    float y = 0.0f;
    if (use_position) {
      if (!doc["x"].is<float>() || !doc["y"].is<float>()) {
        SendError(id, "invalid-goal");
        return;
      }
      x = doc["x"];
      y = doc["y"];
    }

    float angle = 0.0f;
    if (use_heading) {
      if (doc["rotation"].is<float>()) {
        angle = doc["rotation"];
      } else if (doc["angle"].is<float>()) {
        angle = doc["angle"];
      } else {
        SendError(id, "invalid-goal");
        return;
      }
    }

    const float stop = doc["stop_distance"] | 20.0f;
    const float angle_tol = doc["angle_tolerance"] | 5.0f;
    commands_.SetGoal(x, y, stop, use_position, use_heading, angle, angle_tol);
    SendAck("goal-set-result", id, true);
    return;
  }

  if (strcmp(type, "goal-clear") == 0) {
    commands_.ClearGoal();
    SendAck("goal-clear-result", id, true);
    return;
  }

  if (strcmp(type, "goal-tuning") == 0) {
    if (!doc["vmax"].is<float>() || !doc["wmax"].is<float>() ||
        !doc["k_r"].is<float>() || !doc["k_a"].is<float>()) {
      SendError(id, "invalid-goal-tuning");
      return;
    }
    const float vmax = doc["vmax"];
    const float wmax = doc["wmax"];
    const float k_r = doc["k_r"];
    const float k_a = doc["k_a"];
    const float reverse_threshold = doc["reverse_threshold_deg"] | 90.0f;
    const float reverse_hysteresis = doc["reverse_hysteresis_deg"] | 10.0f;
    commands_.SetGoalTuning(vmax, wmax, k_r, k_a, reverse_threshold,
                            reverse_hysteresis);
    SendAck("goal-tuning-result", id, true);
    return;
  }

  if (strcmp(type, "timeline-load") == 0) {
    if (!doc["frames"].is<JsonArray>()) {
      SendError(id, "invalid-timeline");
      return;
    }
    std::vector<ToioController::TimelineFrame> frames;
    for (const auto& f : doc["frames"].as<JsonArray>()) {
      const bool use_position = f["up"] | f["use_position"] | false;
      const bool use_heading =
          f["ur"] | f["use_rotation"] | f["uh"] | f["use_heading"] | false;
      float time_s = 0.0f;
      if (f["t"].is<float>()) {
        time_s = f["t"];
      } else if (f["time"].is<float>()) {
        time_s = f["time"];
      } else {
        SendError(id, "invalid-timeline");
        return;
      }
      ToioController::TimelineFrame frame;
      frame.time_s = time_s;
      frame.use_position = use_position;
      frame.use_heading = use_heading;
      if (use_position) {
        if (!f["x"].is<float>() || !f["y"].is<float>()) {
          SendError(id, "invalid-timeline");
          return;
        }
        frame.x = f["x"];
        frame.y = f["y"];
        frame.stop_distance = f["sd"] | f["stop_distance"] | 20.0f;
      }
      if (use_heading) {
        if (f["rotation"].is<float>()) {
          frame.angle_deg = f["rotation"];
        } else if (f["ang"].is<float>()) {
          frame.angle_deg = f["ang"];
        } else if (f["angle"].is<float>()) {
          frame.angle_deg = f["angle"];
        } else {
          SendError(id, "invalid-timeline");
          return;
        }
        frame.angle_tolerance = f["at"] | f["angle_tolerance"] | 5.0f;
      }
      if (f["sound"].is<const char*>()) {
        frame.sound_id = f["sound"].as<const char*>();
      }
      frames.push_back(frame);
    }
    const bool ok = commands_.LoadTimeline(frames);
    if (!ok) {
      SendError(id, "invalid-timeline");
      return;
    }
    SendAck("timeline-load-result", id, true);
    return;
  }

  if (strcmp(type, "timeline-start") == 0) {
    uint32_t delay_ms = doc["delay_ms"] | 0;
    const uint64_t start_epoch_ms = doc["start_epoch_ms"] | 0ULL;
    if (start_epoch_ms > 0) {
      const uint64_t now_ms = commands_.EpochMillis();
      if (now_ms > 0 && start_epoch_ms > now_ms) {
        delay_ms = static_cast<uint32_t>(start_epoch_ms - now_ms);
      } else {
        delay_ms = 0;
      }
    }
    const bool ok = commands_.StartTimeline(delay_ms);
    SendAck("timeline-start-result", id, ok, ok ? nullptr : "no-timeline");
    return;
  }

  if (strcmp(type, "timeline-stop") == 0) {
    commands_.StopTimeline(true);
    SendAck("timeline-stop-result", id, true);
    return;
  }

  if (strcmp(type, "status-request") == 0) {
    SendStatus(id);
    return;
  }

  if (strcmp(type, "status-subscribe") == 0) {
    if (!doc["enable"].is<bool>() && !doc["enable"].is<int>()) {
      SendError(id, "invalid-subscribe");
      return;
    }
    const bool enable = doc["enable"];
    commands_.SetStatusSubscription(enable);
    SendAck("status-subscribe-result", id, true);
    return;
  }

  SendError(id, "unknown-type");
}

void ProtocolHandler::MaybeSendStatus(bool pose_dirty, bool battery_dirty) {
  if (!connected_) {
    return;
  }
  if (!commands_.StatusSubscriptionEnabled()) {
    return;
  }
  if (pose_dirty || battery_dirty) {
    SendStatus(nullptr);
  }
}

void ProtocolHandler::SendHello() {
  StaticJsonDocument<96> doc;
  doc["type"] = "hello";
  std::string payload;
  serializeJson(doc, payload);
  send_(payload);
}

void ProtocolHandler::SendScanResult(
    const char* id, const CommandDispatcher::ScanResult& result) {
  StaticJsonDocument<384> doc;
  doc["type"] = "scan-result";
  if (id) doc["id"] = id;
  doc["status"] = static_cast<int>(result.status);
  doc["count"] = result.suffixes.size();
  auto array = doc.createNestedArray("suffixes");
  for (const auto& suffix : result.suffixes) {
    array.add(suffix);
  }

  std::string payload;
  serializeJson(doc, payload);
  send_(payload);
}

void ProtocolHandler::SendConnectResult(const char* id,
                                        const std::string& suffix,
                                        ToioController::InitStatus status) {
  StaticJsonDocument<192> doc;
  doc["type"] = "connect-result";
  if (id) doc["id"] = id;
  doc["suffix"] = suffix;
  doc["status"] = MapConnectStatus(status);

  std::string payload;
  serializeJson(doc, payload);
  send_(payload);
}

void ProtocolHandler::SendStatus(const char* id) {
  auto snapshot = commands_.GetStatus();

  StaticJsonDocument<320> doc;
  doc["type"] = "status";
  if (id) doc["id"] = id;
  doc["connected"] = snapshot.has_core;

  if (snapshot.has_pose) {
    doc["x"] = snapshot.pose.x;
    doc["y"] = snapshot.pose.y;
    doc["angle"] = snapshot.pose.angle;
    doc["on_mat"] = snapshot.pose.on_mat ? 1 : 0;
  }
  if (snapshot.has_battery) {
    doc["batt"] = snapshot.battery_level;
  }
  if (snapshot.has_board_voltage) {
    doc["board_v"] = snapshot.board_voltage;
  }
  doc["goal_active"] = snapshot.goal_active;

  auto led = doc.createNestedArray("led");
  led.add(snapshot.led.r);
  led.add(snapshot.led.g);
  led.add(snapshot.led.b);

  auto motor = doc.createNestedArray("motor");
  motor.add(snapshot.motor.left_speed);
  motor.add(snapshot.motor.right_speed);

  std::string payload;
  serializeJson(doc, payload);
  send_(payload);
}

void ProtocolHandler::SendAck(const char* type, const char* id, bool success,
                              const char* message) {
  StaticJsonDocument<160> doc;
  doc["type"] = type;
  if (id) doc["id"] = id;
  doc["ok"] = success;
  if (message) {
    doc["message"] = message;
  }
  std::string payload;
  serializeJson(doc, payload);
  send_(payload);
}

void ProtocolHandler::SendError(const char* id, const char* message) {
  StaticJsonDocument<160> doc;
  doc["type"] = "error";
  if (id) doc["id"] = id;
  doc["message"] = message;

  std::string payload;
  serializeJson(doc, payload);
  send_(payload);
}

const char* ProtocolHandler::MapConnectStatus(ToioController::InitStatus status) {
  switch (status) {
    case ToioController::InitStatus::kConnected:
      return "connected";
    case ToioController::InitStatus::kTargetNotFound:
      return "not_found";
    case ToioController::InitStatus::kConnectionFailed:
      return "failed";
    case ToioController::InitStatus::kNoCubeFound:
      return "no_scan";
    default:
      return "invalid";
  }
}
