#include "commands/command_dispatcher.h"

CommandDispatcher::CommandDispatcher(ToioController& controller)
    : controller_(controller) {}

CommandDispatcher::ScanResult CommandDispatcher::Scan(uint32_t duration_sec) {
  ScanResult result;
  result.status = controller_.scan(duration_sec, &result.suffixes);
  return result;
}

ToioController::InitStatus CommandDispatcher::Connect(
    const std::string& suffix) {
  return controller_.connectBySuffix(suffix);
}

void CommandDispatcher::Disconnect() {
  controller_.disconnect();
}

bool CommandDispatcher::SetLed(uint8_t r, uint8_t g, uint8_t b) {
  return controller_.setLedColor(r, g, b);
}

bool CommandDispatcher::DriveMotor(int8_t left_speed, int8_t right_speed) {
  return controller_.driveMotor(left_speed, right_speed);
}

void CommandDispatcher::SetGoal(float x, float y, float stop_distance,
                                bool use_position, bool use_heading,
                                float target_angle_deg,
                                float angle_tolerance_deg) {
  controller_.setGoal(x, y, stop_distance, use_position, use_heading,
                      target_angle_deg, angle_tolerance_deg);
}

void CommandDispatcher::SetGoalTuning(float vmax, float wmax, float k_r,
                                      float k_a, float reverse_threshold_deg,
                                      float reverse_hysteresis_deg) {
  controller_.setGoalTuning(vmax, wmax, k_r, k_a, reverse_threshold_deg,
                            reverse_hysteresis_deg);
}

bool CommandDispatcher::LoadTimeline(
    const std::vector<ToioController::TimelineFrame>& frames) {
  return controller_.loadTimeline(frames);
}

bool CommandDispatcher::StartTimeline(uint32_t delay_ms) {
  return controller_.startTimeline(delay_ms);
}

void CommandDispatcher::StopTimeline(bool clear_goal) {
  controller_.stopTimeline(clear_goal);
}

void CommandDispatcher::SetTimelineCallback(
    ToioController::TimelineCallback cb) {
  controller_.setTimelineCallback(std::move(cb));
}

void CommandDispatcher::ClearGoal() {
  controller_.clearGoal();
}

void CommandDispatcher::SetStatusSubscription(bool enable) {
  status_subscription_enabled_ = enable;
}

CommandDispatcher::StatusSnapshot CommandDispatcher::GetStatus() const {
  StatusSnapshot snapshot;
  snapshot.has_core = controller_.hasActiveCore();
  snapshot.has_pose = controller_.hasPose();
  snapshot.pose = controller_.pose();
  snapshot.has_battery = controller_.hasBatteryLevel();
  snapshot.battery_level = controller_.batteryLevel();
  snapshot.led = controller_.ledColor();
  snapshot.motor = controller_.motorState();
  snapshot.goal_active = controller_.hasGoal();
  return snapshot;
}
