#include "ui/ui_helpers.h"

#include <M5Unified.h>
#include <ctime>

namespace {
constexpr uint32_t kHeaderTitleY = 20;
constexpr uint32_t kStatusAreaY = 40;
}  // namespace

void UiHelpers::Begin() {
  auto& display = M5.Display;
  display.fillScreen(BLACK);
  display.setTextDatum(TL_DATUM);
  display.setTextColor(WHITE, BLACK);
  last_display_ms_ = millis();
}

void UiHelpers::DrawHeader(const char* message, bool small) {
  auto& display = M5.Display;
  const char* text = (message && message[0] != '\0') ? message
                                                     : "Toio Position Monitor";
  display.fillScreen(BLACK);
  display.setTextDatum(MC_DATUM);
  display.setTextColor(WHITE, BLACK);
  display.setTextSize(small ? 2 : 3);
  display.drawString(text, display.width() / 2, kHeaderTitleY);
  display.setTextSize(1);
  display.setTextDatum(TL_DATUM);
}

void UiHelpers::ShowInitResult(ToioController::InitStatus status) {
  const char* message = nullptr;
  switch (status) {
    case ToioController::InitStatus::kScanReady:
      message = "Scan succeeded";
      break;
    case ToioController::InitStatus::kConnected:
      message = "Connected";
      break;
    case ToioController::InitStatus::kNoCubeFound:
      message = "No cube found.";
      break;
    case ToioController::InitStatus::kTargetNotFound:
      message = "Target cube not found.";
      break;
    case ToioController::InitStatus::kConnectionFailed:
      message = "Connection failed.";
      break;
  case ToioController::InitStatus::kInvalidArgument:
  default:
    message = "Invalid request.";
    break;
  }
  DrawHeader(message, true);
  M5.Log.println(message);
}

void UiHelpers::LogScanResults(const std::vector<std::string>& suffixes) {
  M5.Log.printf("Scan results: %zu device(s)\n",
                static_cast<unsigned long>(suffixes.size()));
  auto& display = M5.Display;
  display.fillRect(0, kStatusAreaY, display.width(),
                   display.height() - kStatusAreaY, BLACK);
  display.setCursor(0, kStatusAreaY);
  display.setTextColor(WHITE, BLACK);
  display.printf("Scan: %zu device(s)\n",
                 static_cast<unsigned long>(suffixes.size()));

  for (size_t i = 0; i < suffixes.size(); ++i) {
    M5.Log.printf("  [%zu] suffix=%s\n", i, suffixes[i].c_str());
    display.printf("  [%zu] %s\n", i, suffixes[i].c_str());
  }
}

void UiHelpers::SetCustomLabel(const std::string& label) {
  custom_label_ = label;
  DrawHeader(custom_label_.c_str(), false);
}

void UiHelpers::UpdateStatus(const CubePose& pose, bool has_pose,
                             uint8_t battery_level, bool has_battery,
                             float board_voltage, const ToioLedColor& led,
                             const ToioMotorState& motor, bool pose_dirty,
                             bool battery_dirty, uint64_t epoch_ms,
                             const std::string& active_suffix,
                             uint32_t refresh_interval_ms) {
  status_.pose = pose;
  status_.has_pose = has_pose;
  status_.battery_level = battery_level;
  status_.has_battery = has_battery;
  status_.board_voltage = board_voltage;
  status_.led = led;
  status_.motor = motor;
  status_.epoch_ms = epoch_ms;
  status_.active_suffix = active_suffix;

  const uint32_t now_ms = millis();
  const bool needs_update = pose_dirty || battery_dirty ||
                            (now_ms - last_display_ms_ >= refresh_interval_ms);
  if (!needs_update) {
    return;
  }

  ShowStatus(now_ms);
  last_display_ms_ = now_ms;
}

void UiHelpers::ShowStatus(uint32_t now_ms) {
  auto& display = M5.Display;
  display.fillRect(0, kStatusAreaY, display.width(),
                   display.height() - kStatusAreaY, BLACK);
  display.setCursor(6, kStatusAreaY + 4);

  display.printf("t:%08lu ms\n", static_cast<unsigned long>(now_ms));
  M5.Log.printf("[%08lu ms][display] ", static_cast<unsigned long>(now_ms));

  if (status_.epoch_ms > 0) {
    time_t sec = static_cast<time_t>(status_.epoch_ms / 1000ULL);
    struct tm tm_now;
    localtime_r(&sec, &tm_now);
    char buf[32];
    strftime(buf, sizeof(buf), "%H:%M:%S", &tm_now);
    const uint32_t msec = static_cast<uint32_t>(status_.epoch_ms % 1000ULL);
    display.printf("Time: %s.%03u\n", buf, static_cast<unsigned>(msec));
  } else {
    display.printf("Time: n/a\n");
  }

  if (status_.has_pose) {
    display.printf("Cube  X:%4u  Y:%4u \n Angle:%3u, on_mat:%s\n", status_.pose.x,
                   status_.pose.y, status_.pose.angle,
                   status_.pose.on_mat ? "yes" : "no");
    M5.Log.printf("x=%u y=%u angle=%u on_mat=%s ", status_.pose.x,
                  status_.pose.y, status_.pose.angle,
                  status_.pose.on_mat ? "yes" : "no");
  } else {
    display.println("No position (off mat)");
    M5.Log.print("no position ");
  }

  if (status_.has_battery) {
    display.printf("Battery: %3u%%", status_.battery_level);
    M5.Log.printf("battery=%u%%", status_.battery_level);
  }
  display.printf("Board V: %.2fV\n", status_.board_voltage);
  M5.Log.printf(" board_v=%.2fV", status_.board_voltage);

  display.printf("LED RGB:(%3u,%3u,%3u)\n", status_.led.r, status_.led.g,
                 status_.led.b);
  display.printf("Motor L:%4d R:%4d\n", status_.motor.left_speed,
                 status_.motor.right_speed);
  M5.Log.printf(" LED RGB:(%u,%u,%u) Motor L:%d R:%d", status_.led.r,
                status_.led.g, status_.led.b, status_.motor.left_speed,
                status_.motor.right_speed);
  M5.Log.println();

  // Show active toio suffix at bottom-left in larger font.
  display.setTextSize(2);
  const char* suffix = status_.active_suffix.empty()
                           ? "no core"
                           : status_.active_suffix.c_str();
  display.setCursor(6, display.height() - 22);
  display.printf("ID: %s", suffix);
  display.setTextSize(1);
}
