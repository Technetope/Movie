#pragma once

#include <Arduino.h>
#include <Toio.h>

#include <array>
#include <functional>
#include <string>
#include <vector>

#include "../goal_tracker/cube_pose.h"
#include "../goal_tracker/goal_tracker.h"

struct ToioLedColor {
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
};

struct ToioMotorState {
  int8_t left_speed = 0;
  int8_t right_speed = 0;
};

class ToioController {
 public:
  enum class InitStatus {
    kScanReady,
    kConnected,
    kNoCubeFound,
    kConnectionFailed,
    kInvalidArgument,
    kTargetNotFound,
  };

  InitStatus scan(uint32_t scan_duration_sec,
                  std::vector<std::string>* out_suffixes);
  InitStatus connectBySuffix(const std::string& suffix);
  void loop();

  bool hasActiveCore() const { return active_core_ != nullptr; }
  const std::string& activeSuffix() const { return active_suffix_; }
  bool hasGoal() const { return goal_tracker_.hasGoal(); }

  bool hasPose() const { return has_pose_; }
  const CubePose& pose() const { return pose_; }
  bool poseDirty() const { return pose_dirty_; }
  void clearPoseDirty() { pose_dirty_ = false; }

  bool hasBatteryLevel() const { return has_battery_; }
  uint8_t batteryLevel() const { return battery_level_; }
  bool batteryDirty() const { return battery_dirty_; }
  void clearBatteryDirty() { battery_dirty_ = false; }

  ToioLedColor ledColor() const { return led_color_; }
  ToioMotorState motorState() const { return motor_state_; }

  bool setLedColor(uint8_t r, uint8_t g, uint8_t b);
  bool driveMotor(int8_t left_speed, int8_t right_speed);
  void disconnect();

  void setGoal(float x, float y, float stop_distance, bool use_position,
               bool use_heading, float target_angle_deg,
               float angle_tolerance_deg);
  void clearGoal();
  void setGoalTuning(float vmax, float wmax, float k_r, float k_a,
                     float reverse_threshold_deg = 90.0f,
                     float reverse_hysteresis_deg = 10.0f);

  struct TimelineFrame {
    float time_s = 0.0f;
    bool use_position = true;
    bool use_heading = false;
    float x = 0.0f;
    float y = 0.0f;
    float angle_deg = 0.0f;
    float stop_distance = 20.0f;
    float angle_tolerance = 10.0f;
    std::string sound_id;
  };

  bool loadTimeline(const std::vector<TimelineFrame>& frames);
  bool startTimeline(uint32_t delay_ms);
  void stopTimeline(bool clear_goal = true);
  using TimelineCallback =
      std::function<void(size_t index, const TimelineFrame& frame)>;
  void setTimelineCallback(TimelineCallback cb) { timeline_callback_ = cb; }
  void setSoundCallback(std::function<void(const std::string&)> cb) {
    sound_callback_ = std::move(cb);
  }
  // Returns epoch milliseconds if available, otherwise 0.
  uint64_t epochMillis() const;

 private:
  struct ScanEntry {
    ToioCore* core = nullptr;
    std::string suffix;
  };

  InitStatus connectCore(ToioCore* core);
  void configureCore(ToioCore* core);
  void handleIdData(const ToioCoreIDData& data);
  void handleBatteryLevel(uint8_t level);
  void updateGoalTracking();
  void updateTimelinePlayback();

  Toio toio_;
  ToioCore* active_core_ = nullptr;
  std::vector<ScanEntry> last_scan_results_;

  CubePose pose_{};
  bool has_pose_ = false;
  bool pose_dirty_ = false;
  uint32_t pose_updated_ms_ = 0;

  uint8_t battery_level_ = 0;
  bool has_battery_ = false;
  bool battery_dirty_ = false;
  uint32_t battery_updated_ms_ = 0;

  ToioLedColor led_color_{};
  ToioMotorState motor_state_{};

  uint32_t scan_duration_sec_ = 0;

  // Maximum number of frames that can be preloaded for a timeline.
  static constexpr size_t kMaxTimelineFrames = 400;
  std::array<TimelineFrame, kMaxTimelineFrames> timeline_frames_{};
  size_t timeline_count_ = 0;
  size_t timeline_next_index_ = 0;
  bool timeline_loaded_ = false;
  bool timeline_playing_ = false;
  uint32_t timeline_start_ms_ = 0;
  TimelineCallback timeline_callback_;
  std::function<void(const std::string&)> sound_callback_;

  GoalTracker goal_tracker_;

  std::string active_suffix_;
};
