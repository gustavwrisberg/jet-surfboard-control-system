#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port
  }
  Serial.println("Boot OK");

  WiFi.mode(WIFI_STA);
}

void loop() {
  Serial.println("Loop running...");

  String mac = WiFi.macAddress();
  Serial.print("MAC: ");
  Serial.println(mac);

  delay(2000);
}
