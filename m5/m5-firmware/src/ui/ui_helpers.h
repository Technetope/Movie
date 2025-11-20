#pragma once

#include <stdint.h>
#include <string>
#include <vector>

#include "controller/toio_controller.h"

class UiHelpers {
 public:
  enum class DeviceState {
    kBoot,
    kWifiConnected,
    kWsReady,
    kScanDone,
    kCubeConnected,
    kWriteDone,
  };

  void Begin();
  void SetBackground(DeviceState state);
  void DrawHeader(const char* message, bool small = false);
  void ShowInitResult(ToioController::InitStatus status);
   // スキャン結果（suffix 一覧）のログ＋画面出力
  void LogScanResults(const std::vector<std::string>& suffixes);
  void SetCustomLabel(const std::string& label);
  void UpdateStatus(const CubePose& pose, bool has_pose, uint8_t battery_level,
                    bool has_battery, float board_voltage,
                    const ToioLedColor& led, const ToioMotorState& motor,
                    bool pose_dirty, bool battery_dirty,
                    uint64_t epoch_ms, const std::string& active_suffix,
                    uint32_t refresh_interval_ms);

 private:
  struct UiStatus {
    CubePose pose{};
    bool has_pose = false;
    uint8_t battery_level = 0;
    bool has_battery = false;
    float board_voltage = 0.0f;
    ToioLedColor led{};
    ToioMotorState motor{};
    uint64_t epoch_ms = 0;
    std::string active_suffix;
  };

  void ShowStatus(uint32_t now_ms);

  UiStatus status_{};
  DeviceState last_state_ = DeviceState::kBoot;
  uint16_t bg_color_ = 0x0000;   // BLACK
  uint16_t text_color_ = 0xFFFF; // WHITE
  uint32_t last_display_ms_ = 0;
  std::string custom_label_;
  std::string last_header_;
  bool last_header_small_ = false;
};
