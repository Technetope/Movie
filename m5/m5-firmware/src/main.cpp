#include <M5Unified.h>
#include <cstdio>
#include <string>
#include <sys/time.h>
#include <time.h>
#include <esp_system.h>

#include "commands/command_dispatcher.h"
#include "controller/toio_controller.h"
#include "audio/wav_player.h"
#include "net/websocket_server.h"
#include "protocol/protocol_handler.h"
#include "ui/ui_helpers.h"

namespace {
constexpr uint32_t kRefreshIntervalMs = 1000;
constexpr uint16_t kWebsocketPort = 9000;

// Wi-Fi credentials: replace with your network settings.
constexpr char kWifiSsid[] = "TomoshibiTechnology_IoT";
constexpr char kWifiPassword[] = "All_outlook";

ToioController g_toio;
UiHelpers g_ui;
WebsocketServer g_server;
CommandDispatcher g_commands(g_toio);
WavPlayer g_wav_player;
ProtocolHandler g_protocol(
    g_commands, [](const std::string& payload) { return g_server.Send(payload); },
    g_ui);

void InitializeM5Hardware() {
  auto cfg = M5.config();
  cfg.clear_display = true;
  cfg.output_power = true;
  cfg.serial_baudrate = 115200;
  cfg.external_speaker.hat_spk2 = true;  // SPK HAT2 (M5StickC Plus2)
  M5.begin(cfg);

  // Optional: tune speaker sample rate for better sound quality.
  auto spk_cfg = M5.Speaker.config();
  if (spk_cfg.use_dac || spk_cfg.buzzer) {
    spk_cfg.sample_rate = 192000;
    M5.Speaker.config(spk_cfg);
  }
  M5.Speaker.begin();

  M5.Display.setRotation(3);
  g_ui.Begin();
  g_ui.SetBackground(UiHelpers::DeviceState::kBoot);
  g_ui.DrawHeader("Wi-Fi connecting...", true);
  g_wav_player.Begin();
}
}  // namespace

void setup() {
  InitializeM5Hardware();

  g_commands.SetSoundCallback(
      [](const std::string& id) { g_wav_player.PlayById(id); });

  const bool net_ok = g_server.Begin(
      kWifiSsid, kWifiPassword, kWebsocketPort,
      [](const std::string& message) { g_protocol.HandleMessage(message); },
      []() { g_protocol.HandleClientConnected(); },
      []() { g_protocol.HandleClientDisconnected(); });
  if (!net_ok) {
    g_ui.DrawHeader("Wi-Fi failed");
    return;
  }
  g_ui.SetBackground(UiHelpers::DeviceState::kWifiConnected);

  // NTP time sync for timeline absolute start
  configTime(0, 0, "ntp.nict.jp", "pool.ntp.org");

  char header[64];
  snprintf(header, sizeof(header), "WS %s",
           g_server.local_ip().toString().c_str());
  g_ui.DrawHeader(header, true);
}

void loop() {
  M5.update();
  if (M5.BtnA.pressedFor(1500)) {
    M5.Log.println("Power off requested (BtnA hold)");
    M5.Power.powerOff();
  }
  if (M5.BtnA.wasReleased() && !M5.BtnA.pressedFor(1500)) {
    M5.Log.println("Reset requested (BtnA)");
    esp_restart();
  }
  g_toio.loop();
  g_server.Loop();
  g_wav_player.Loop();

  const bool pose_dirty = g_toio.poseDirty();
  const bool battery_dirty = g_toio.batteryDirty();

  const float board_voltage = M5.Power.getBatteryVoltage()*(3.3f/4096.0f);
  g_commands.SetBoardVoltage(board_voltage);

  const uint64_t epoch_ms = []() -> uint64_t {
    timeval tv{};
    if (gettimeofday(&tv, nullptr) != 0) return 0;
    return static_cast<uint64_t>(tv.tv_sec) * 1000ULL +
           static_cast<uint64_t>(tv.tv_usec) / 1000ULL;
  }();

  g_ui.UpdateStatus(g_toio.pose(), g_toio.hasPose(), g_toio.batteryLevel(),
                    g_toio.hasBatteryLevel(), board_voltage,
                    g_toio.ledColor(), g_toio.motorState(), pose_dirty,
                    battery_dirty, epoch_ms, g_toio.activeSuffix(),
                    kRefreshIntervalMs);
  if (pose_dirty) {
    g_toio.clearPoseDirty();
  }
  if (battery_dirty) {
    g_toio.clearBatteryDirty();
  }

  g_protocol.MaybeSendStatus(pose_dirty, battery_dirty);

  // delay(10);
}
