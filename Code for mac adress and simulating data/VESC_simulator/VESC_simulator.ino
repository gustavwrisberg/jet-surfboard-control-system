// ---------- VESC Bridge Simulator with Ramping ----------
#include <esp_now.h>
#include <WiFi.h>

// Peer MAC (ESP32 #1 controller)
uint8_t controllerMac[] = {0x24, 0x6F, 0x28, 0x12, 0x34, 0x56};

// Packet structures
struct ThrottlePacket {
  uint32_t seq;
  float throttle_pct; // 0..100
};

struct TelemetryPacket {
  uint32_t seq_echo;
  bool failsafe;
  float throttle_echo;
  float speed_kmh;
  float voltage_v;
  float motor_current_a;
  float fet_temp_c;
  float watt_hours;
  uint32_t age_ms;
};

// Globals
volatile ThrottlePacket gLastRx{};
volatile bool gHaveRx = false;

// Ramping variables
float duty_cmd = 0.0f;       // current “duty cycle” 0..1
const float MAX_DUTY_SLEW_PER_S = 0.6f; // max change per second
const float DUTY_MAX = 0.95f;

uint32_t lastLoop = 0;

// ESP-NOW callbacks
void onRecv(con
