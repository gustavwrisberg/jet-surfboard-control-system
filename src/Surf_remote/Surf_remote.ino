// ---------- ESP32 #1 : Controller + OLED (SS495A) ----------
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ADC input pin for throttle:
const int THROTTLE_ADC_PIN = 34;

// Calibrate these to your sensor:
const int ADC_MIN = 2000;         // raw ADC at minimum magnet position
const int ADC_MAX = 3600;         // raw ADC at maximum magnet position
const float DEAD_BAND_PCT = 0.0;  // optional deadband at low end

// Low-pass filter (0..1): higher = snappier
const float EMA_ALPHA = 0.25f;

// ESP-NOW peer (ESP32 #2 MAC) — CHANGE THIS
uint8_t bridgeMac[] = { 0x80, 0xF3, 0xDA, 0xBA, 0x73, 0x68 };

// --- Packet formats ---
struct ThrottlePacket {
  uint32_t seq;
  float throttle_pct;  // 0..100
};

struct TelemetryPacket {
  uint32_t seq_echo;    // last throttle seq seen by ESP32 #2
  bool failsafe;        // true if bridge is in failsafe (throttle lost)
  float throttle_echo;  // echo of throttle pct being applied (pre-ramp)
  float rpm;
  float voltage_v;
  float motor_current_a;
  float tempMotor;  // changed from tempFET to tempMotor
  float watt_hours;
  uint32_t age_ms;  // how "old" the telemetry is at sender side
};

volatile TelemetryPacket gTelem = {};
volatile bool gTelemFresh = false;

uint32_t txSeq = 0;
uint32_t lastTelemMillis = 0;

// --------------------------------------
// ESP-NOW callback fixes for ESP32 v5.x
// --------------------------------------
void onSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
  // info contains MAC and other TX info
}

void onRecv(const esp_now_recv_info_t* recv_info, const uint8_t* data, int len) {
  // copy telemetry safely
  if (len == sizeof(TelemetryPacket)) {
    memcpy((void*)&gTelem, data, sizeof(TelemetryPacket));
    gTelemFresh = true;
    lastTelemMillis = millis();
  }
}

// --------------------------------------
static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

float ema(float prev, float cur, float alpha) {
  return prev + alpha * (cur - prev);
}

void drawOLED(float throttlePct, const TelemetryPacket& t) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  // Line 1: Link/failsafe
  display.print("Link: ");
  uint32_t age = millis() - lastTelemMillis;
  display.print(age < 1000 ? "OK " : "OLD");
  display.print("  FS: ");
  display.println(t.failsafe ? "YES" : "NO ");

  // Line 2: Throttle
  display.print("Thr: ");
  display.print(throttlePct, 1);
  display.println("%");

  // Line 3: Speed
  display.print("Spd: ");
  display.print(t.rpm, 1);
  display.println(" rpm");

  // Line 4: Voltage
  display.print("Volt: ");
  display.print(t.voltage_v, 2);
  display.println(" V");

  // Line 5: Current
  display.print("Curr: ");
  display.print(t.motor_current_a, 1);
  display.println(" A");

  // Line 6: Temp & Wh
  display.print("Tmp: ");
  display.print(t.tempMotor, 1);
  display.print(" C  ");
  display.print("Wh: ");
  display.print(t.watt_hours, 2);

  display.display();
}

// --------------------------------------
void setup() {
  Serial.begin(115200);

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 failed");
    while (true) delay(1000);
  }
  display.clearDisplay();
  display.display();

  // ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }
  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, bridgeMac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Add peer failed");
    while (true) delay(1000);
  }
}

// --------------------------------------
void loop() {
  static float filtAdc = 0;
  static uint32_t lastTx = 0;

  int raw = analogRead(THROTTLE_ADC_PIN);
  filtAdc = ema(filtAdc == 0 ? raw : filtAdc, raw, EMA_ALPHA);

  // Map ADC to 0..100%
  const int ADC_MID = (ADC_MIN + ADC_MAX) / 2;
  float pct = (filtAdc < ADC_MID) ? 0.0f
                                  : (filtAdc - ADC_MID) * 100.0f / (ADC_MAX - ADC_MID);
  pct = clampf(pct, 0.0f, 100.0f);

  if (pct < DEAD_BAND_PCT) pct = 0.0f;

  // Send throttle at ~20 Hz
  if (millis() - lastTx >= 50) {
    ThrottlePacket tx{};
    tx.seq = ++txSeq;
    tx.throttle_pct = pct;
    esp_now_send(bridgeMac, (uint8_t*)&tx, sizeof(tx));
    lastTx = millis();
  }

  // Draw telemetry safely
  TelemetryPacket t;
  noInterrupts();  // protect against concurrent writes
  memcpy(&t, (const TelemetryPacket*)&gTelem, sizeof(TelemetryPacket));
  interrupts();
  drawOLED(pct, t);

  delay(500);
}
