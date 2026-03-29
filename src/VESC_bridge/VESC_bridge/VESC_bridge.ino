// ---------- ESP32 Bridge : ESP-NOW <-> VESC UART ----------
#include <esp_now.h>
#include <WiFi.h>
#include <VescUart.h>

// ---------- VESC UART ----------
// Change these if your wiring is different
#define VESC_RX_PIN 16   // ESP32 RX  <- VESC TX
#define VESC_TX_PIN 17   // ESP32 TX  -> VESC RX
#define VESC_BAUD   115200

HardwareSerial VESCSerial(2);
VescUart UART;

// ---------- Control tuning ----------
const float MAX_DUTY = 0.35f;               // 0.0 .. 1.0, tune carefully
const uint32_t RX_TIMEOUT_MS = 500;         // failsafe if no throttle packets
const uint32_t TELEMETRY_PERIOD_MS = 100;   // send telemetry at 10 Hz
const uint32_t KEEPALIVE_PERIOD_MS = 100;   // VESC keepalive
const float THROTTLE_DEADBAND_PCT = 0.5f;

// ---------- Packet formats ----------
struct ThrottlePacket {
  uint32_t seq;
  float throttle_pct;   // 0..100
};

struct TelemetryPacket {
  uint32_t seq_echo;
  bool failsafe;
  float throttle_echo;
  float rpm;
  float voltage_v;
  float motor_current_a;
  float tempMotor;
  float watt_hours;
  uint32_t age_ms;
};

// ---------- Globals ----------
volatile ThrottlePacket gThrottle = {};
volatile bool gHaveThrottle = false;
volatile uint32_t gLastRxMillis = 0;

uint8_t controllerMac[6] = {0};
bool controllerKnown = false;
bool peerAdded = false;

uint32_t lastTelemetryTx = 0;
uint32_t lastKeepalive = 0;

// ---------- Helpers ----------
static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

void rememberControllerPeer(const uint8_t *mac) {
  memcpy(controllerMac, mac, 6);
  controllerKnown = true;

  if (peerAdded) {
    esp_now_del_peer(controllerMac);
    peerAdded = false;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) == ESP_OK) {
    peerAdded = true;
  }
}

float throttlePctToDuty(float throttlePct) {
  float pct = clampf(throttlePct, 0.0f, 100.0f);

  if (pct < THROTTLE_DEADBAND_PCT) {
    return 0.0f;
  }

  pct = (pct - THROTTLE_DEADBAND_PCT) * 100.0f / (100.0f - THROTTLE_DEADBAND_PCT);
  pct = clampf(pct, 0.0f, 100.0f);

  return (pct / 100.0f) * MAX_DUTY;
}

// ---------- ESP-NOW callbacks ----------
// Newer ESP32 Arduino core callback signatures
void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;
  (void)status;
}

void onRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (len != sizeof(ThrottlePacket)) return;

  ThrottlePacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  memcpy((void*)&gThrottle, &pkt, sizeof(pkt));
  gHaveThrottle = true;
  gLastRxMillis = millis();

  if (recv_info && recv_info->src_addr) {
    rememberControllerPeer(recv_info->src_addr);
  }
}

void sendTelemetry(bool failsafeActive, float throttleEcho) {
  if (!controllerKnown || !peerAdded) return;

  TelemetryPacket t = {};
  ThrottlePacket rxCopy;

  noInterrupts();
  memcpy(&rxCopy, (const void*)&gThrottle, sizeof(rxCopy));
  interrupts();

  t.seq_echo = rxCopy.seq;
  t.failsafe = failsafeActive;
  t.throttle_echo = throttleEcho;
  t.age_ms = millis() - gLastRxMillis;

  if (UART.getVescValues()) {
    t.rpm = UART.data.rpm;
    t.voltage_v = UART.data.inpVoltage;
    t.motor_current_a = UART.data.avgMotorCurrent;
    t.tempMotor = UART.data.tempMotor;
    t.watt_hours = UART.data.wattHours;
  } else {
    t.rpm = 0.0f;
    t.voltage_v = 0.0f;
    t.motor_current_a = 0.0f;
    t.tempMotor = 0.0f;
    t.watt_hours = 0.0f;
  }

  esp_now_send(controllerMac, (uint8_t*)&t, sizeof(t));
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(500);

  // VESC UART
  VESCSerial.begin(VESC_BAUD, SERIAL_8N1, VESC_RX_PIN, VESC_TX_PIN);
  UART.setSerialPort(&VESCSerial);

  // WiFi / ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }

  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onRecv);

  Serial.println("Bridge ready");
}

// ---------- Loop ----------
void loop() {
  ThrottlePacket rxCopy = {};
  bool haveThrottle = false;
  uint32_t lastRx = 0;

  noInterrupts();
  memcpy(&rxCopy, (const void*)&gThrottle, sizeof(rxCopy));
  haveThrottle = gHaveThrottle;
  lastRx = gLastRxMillis;
  interrupts();

  bool failsafeActive = (!haveThrottle) || ((millis() - lastRx) > RX_TIMEOUT_MS);

  float commandedThrottlePct = failsafeActive ? 0.0f : clampf(rxCopy.throttle_pct, 0.0f, 100.0f);
  float dutyCmd = throttlePctToDuty(commandedThrottlePct);

  // Drive VESC
  if (failsafeActive) {
    UART.setDuty(0.0f);
  } else {
    UART.setDuty(dutyCmd);
  }

  // Keepalive
  if (millis() - lastKeepalive >= KEEPALIVE_PERIOD_MS) {
    UART.sendKeepalive();
    lastKeepalive = millis();
  }

  // Telemetry back to controller
  if (millis() - lastTelemetryTx >= TELEMETRY_PERIOD_MS) {
    sendTelemetry(failsafeActive, commandedThrottlePct);
    lastTelemetryTx = millis();
  }

  // Debug
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 250) {
    Serial.print("failsafe=");
    Serial.print(failsafeActive ? "YES" : "NO");
    Serial.print(" thr%=");
    Serial.print(commandedThrottlePct, 1);
    Serial.print(" duty=");
    Serial.println(dutyCmd, 3);
    lastPrint = millis();
  }

  delay(10);
}