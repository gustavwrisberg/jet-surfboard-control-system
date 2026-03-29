// ---------- ESP32-C3 : Controller + OLED ----------
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- OLED ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
#define OLED_SDA 8
#define OLED_SCL 9

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ---------- ADC input (Hall sensor / throttle pot) ----------
const int THROTTLE_ADC_PIN = 0;

// Learned at startup
int adcZero = 0;

// Full throttle ADC value
// Your throttle goes from about 3500 (rest) down to 1900 (full)
const int ADC_FULL = 1900;

const float DEAD_BAND_PCT = 10.0f;
const float EMA_ALPHA = 0.25f;

// ---------- ESP-NOW peer (bridge MAC) ----------
uint8_t bridgeMac[] = { 0x20, 0x6E, 0xF1, 0x6A, 0x20, 0x54 };

// ---------- Packet formats ----------
struct ThrottlePacket {
  uint32_t seq;
  float throttle_pct;
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
#define TEMP_WARNING 80.0
#define VOLTAGE_LOW 12.0
#define BLINK_PERIOD 1000

// ---------- ESP-NOW callbacks ----------
void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
}

void onRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
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

int calibrateThrottleZero(int samples = 100) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(THROTTLE_ADC_PIN);
    delay(5);
  }
  return sum / samples;
}

void drawOLED(float throttlePct, const TelemetryPacket& t, bool linkActive) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.print("Link: ");
  if (!linkActive) {
    if (blinkState()) display.print("NO");
    else display.print("  ");
  } else {
    display.print("OK");
  }
  display.print(" FS: ");
  display.println(t.failsafe ? "YES" : "NO");

  display.print("Thr: ");
  display.print(throttlePct, 1);
  display.println("%");

  display.print("Spd: ");
  display.print(t.rpm, 0);
  display.println(" rpm");

  display.print("Volt: ");
  if (t.voltage_v > 0.01f && t.voltage_v < VOLTAGE_LOW && blinkState()) {
    display.println("!!");
  } else {
    display.print(t.voltage_v, 2);
    display.println(" V");
  }

  display.print("Curr: ");
  display.print(t.motor_current_a, 1);
  display.println(" A");

  display.print("Tmp: ");
  if (t.tempMotor > TEMP_WARNING && blinkState()) {
    display.print("!!");
  } else {
    display.print(t.tempMotor, 1);
    display.print("C");
  }
  display.print(" Wh:");
  display.print(t.watt_hours, 1);

  display.display();
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(OLED_SDA, OLED_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 init failed");
    while (true) delay(1000);
  }

  display.setRotation(2);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OLED OK");
  display.display();
  delay(1000);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Calibrate zero at startup
  adcZero = calibrateThrottleZero();

  Serial.print("adcZero = ");
  Serial.println(adcZero);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }

  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer = {};
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

  int raw = analogRead(THROTTLE_ADC_PIN);
  filtAdc = ema((filtAdc == 0) ? raw : filtAdc, raw, EMA_ALPHA);

  // Rest = adcZero, full = ADC_FULL
  // ADC decreases as throttle increases
  float pct = (adcZero - filtAdc) * 100.0f / (adcZero - ADC_FULL);
  pct = clampf(pct, 0.0f, 100.0f);

  if (pct < DEAD_BAND_PCT) {
    pct = 0.0f;
  }

  if (millis() - lastTx >= 50) {
    ThrottlePacket tx = {};
    tx.seq = ++txSeq;
    tx.throttle_pct = pct;
    esp_now_send(bridgeMac, (uint8_t*)&tx, sizeof(tx));
    lastTx = millis();
  }

  TelemetryPacket t;
  noInterrupts();
  memcpy(&t, (const void*)&gTelem, sizeof(TelemetryPacket));
  interrupts();

  bool linkActive = (millis() - lastTelemMillis) < 2000;

  Serial.print("raw=");
  Serial.print(raw);
  Serial.print(" zero=");
  Serial.print(adcZero);
  Serial.print(" pct=");
  Serial.println(pct, 1);

  drawOLED(pct, t, linkActive);

  delay(100);
}