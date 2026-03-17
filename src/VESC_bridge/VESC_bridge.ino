// ---------- ESP32 #1 : Controller + OLED ----------
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- OLED ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ---------- ADC input (Hall sensor / throttle pot) ----------
const int THROTTLE_ADC_PIN = 34;

// Calibrate these for your sensor:
const int ADC_MIN = 2000;         // raw ADC at minimum position
const int ADC_MAX = 3600;         // raw ADC at maximum position
const float DEAD_BAND_PCT = 0.0;  // optional deadband

// Smoothing filter
const float EMA_ALPHA = 0.25f;

// ---------- ESP-NOW peer (bridge MAC) ----------
uint8_t bridgeMac[] = { 0x20, 0x6E, 0xF1, 0x6A, 0x20, 0x54 };

// ---------- Packet formats ----------
struct ThrottlePacket {
  uint32_t seq;
  float throttle_pct;  // 0..100
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
volatile TelemetryPacket gTelem = {};
uint32_t txSeq = 0;
uint32_t lastTelemMillis = 0;

// ---------- Warning thresholds ----------
#define TEMP_WARNING 80.0  // °C
#define VOLTAGE_LOW 12.0   // V
#define BLINK_PERIOD 1000  // ms

// ---------- ESP-NOW callbacks ----------
void onSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {}

void onRecv(const esp_now_recv_info_t* recv_info, const uint8_t* data, int len) {
  if (len == sizeof(TelemetryPacket)) {
    memcpy((void*)&gTelem, data, sizeof(TelemetryPacket));
    lastTelemMillis = millis();
  }
}

// ---------- Helpers ----------
static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

float ema(float prev, float cur, float alpha) {
  return prev + alpha * (cur - prev);
}

bool blinkState() {
  return (millis() % BLINK_PERIOD) < (BLINK_PERIOD / 2);
}

void drawOLED(float throttlePct, const TelemetryPacket& t, bool linkActive) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  // Line 1: Link/failsafe (blink if no link)
  display.print("Link: ");
  if (!linkActive) {
    if (blinkState()) display.print("NO");
    else display.print("  ");
  } else {
    display.print("OK");
  }
  display.print("  FS: ");
  display.println(t.failsafe ? "YES" : "NO ");

  // Line 2: Throttle
  display.print("Thr: ");
  display.print(throttlePct, 1);
  display.println("%");

  // Line 3: Speed
  display.print("Spd: ");
  display.print(t.rpm, 0);
  display.println(" rpm");

  // Line 4: Voltage (blink if too low)
  display.print("Volt: ");
  if (t.voltage_v < VOLTAGE_LOW && blinkState()) {
    display.println("!!");
  } else {
    display.print(t.voltage_v, 2);
    display.println(" V");
  }

  // Line 5: Current
  display.print("Curr: ");
  display.print(t.motor_current_a, 1);
  display.println(" A");

  // Line 6: Temp & Wh (blink if too hot)
  display.print("Tmp: ");
  if (t.tempMotor > TEMP_WARNING && blinkState()) {
    display.print("!!");
  } else {
    display.print(t.tempMotor, 1);
    display.print("C");
  }
  display.print("  Wh: ");
  display.print(t.watt_hours, 1);

  display.display();
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 init failed");
    while (true) delay(1000);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OLED Ready!");
  display.display();
  delay(2000); // show startup message

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

  // Add bridge peer
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, bridgeMac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Add peer failed");
    while (true) delay(1000);
  }
}


// ---------- Loop ----------
void loop() {
  static float filtAdc = 0;
  static uint32_t lastTx = 0;

  // Read throttle
  int raw = analogRead(THROTTLE_ADC_PIN);
  filtAdc = ema(filtAdc == 0 ? raw : filtAdc, raw, EMA_ALPHA);

  // Map ADC to %
  const int ADC_MID = (ADC_MIN + ADC_MAX) / 2;
  float pct = (filtAdc < ADC_MID) ? 0.0f
                                  : (filtAdc - ADC_MID) * 100.0f / (ADC_MAX - ADC_MID);
  pct = clampf(pct, 0.0f, 100.0f);

  if (pct < DEAD_BAND_PCT) pct = 0.0f;

  // Send throttle every 50 ms
  if (millis() - lastTx >= 50) {
    ThrottlePacket tx{};
    tx.seq = ++txSeq;
    tx.throttle_pct = pct;
    esp_now_send(bridgeMac, (uint8_t*)&tx, sizeof(tx));
    lastTx = millis();
  }

  // Copy telemetry safely
  TelemetryPacket t;
  noInterrupts();
  memcpy(&t, (const TelemetryPacket*)&gTelem, sizeof(TelemetryPacket));
  interrupts();

  // Check if telemetry is fresh
  bool linkActive = (millis() - lastTelemMillis) < 2000;

  // Update OLED
  drawOLED(pct, t, linkActive);

  delay(100);  // ~10 Hz display refresh
}
