#pragma once

#include <string>
#include <vector>

#include "../controller/toio_controller.h"

class CommandDispatcher {
 public:
  explicit CommandDispatcher(ToioController& controller);

  struct ScanResult {
    ToioController::InitStatus status = ToioController::InitStatus::kInvalidArgument;
    std::vector<std::string> suffixes;
  };

  ScanResult Scan(uint32_t duration_sec);
  ToioController::InitStatus Connect(const std::string& suffix);
  void Disconnect();

  bool SetLed(uint8_t r, uint8_t g, uint8_t b);
  bool DriveMotor(int8_t left_speed, int8_t right_speed);

  void SetGoal(float x, float y, float stop_distance, bool use_position,
               bool use_heading, float target_angle_deg,
               float angle_tolerance_deg);
  void SetGoalTuning(float vmax, float wmax, float k_r, float k_a,
                     float reverse_threshold_deg, float reverse_hysteresis_deg);
  bool LoadTimeline(const std::vector<ToioController::TimelineFrame>& frames);
  bool StartTimeline(uint32_t delay_ms);
  void StopTimeline(bool clear_goal = true);
  uint64_t EpochMillis() const;
  void SetTimelineCallback(ToioController::TimelineCallback cb);
  void ClearGoal();

  void SetStatusSubscription(bool enable);
  bool StatusSubscriptionEnabled() const { return status_subscription_enabled_; }
  void SetBoardVoltage(float voltage);

  struct StatusSnapshot {
    bool has_core = false;
    bool has_pose = false;
    CubePose pose{};
    bool has_battery = false;
    uint8_t battery_level = 0;
    bool has_board_voltage = false;
    float board_voltage = 0.0f;
    ToioLedColor led{};
    ToioMotorState motor{};
    bool goal_active = false;
  };
  StatusSnapshot GetStatus() const;

 private:
  ToioController& controller_;
  bool status_subscription_enabled_ = false;
  bool has_board_voltage_ = false;
  float board_voltage_ = 0.0f;
};
