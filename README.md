# Jet-Driven Surfboard Control System

## Project Overview
This project is a **jet-driven surfboard control system** inspired by commercial boards.  
It integrates embedded controllers, wireless communication, and a high-power propulsion system.

### Components Built
- **Controller:** ESP32-based throttle input with OLED display (SS495A Hall sensor)
- **Bridge:** ESP32 VESC bridge for UART communication to VESC and telemetry relay
- **VESC Motor Controller:** Off-the-shelf Makerbase VESC 84V 200A
- **Battery Pack:** Custom 10s5p Li-ion (Enpower INR21700-30SG)
- **Wireless Charging Pad:** For 3.7V cells powering the controller

> Most components are off-the-shelf to reduce cost and accelerate prototyping.  

---

## System Architecture

**Description:**
- Controller reads throttle from Hall sensor and displays telemetry on OLED.
- Throttle is sent to VESC bridge via ESP-NOW.
- VESC bridge communicates throttle to VESC via UART and relays telemetry back to controller.
- Battery supplies power to VESC and controller; wireless charger charges 3.7V cells for the controller.

---

### Wiring Diagram
![Wiring Diagram](wiring_diagram.png)

**Highlights:**
- Color-coded wires for clarity:
  - **Red:** Power lines
  - **Black:** Ground
  - **Blue:** Signal lines
  - **Green:** Communication (UART/ESP-NOW)
- Proper spacing to illustrate connections clearly between Controller, Bridge, VESC, Motor, and Battery.

---

## Build & Flash
Build & Flash Instructions
Install Arduino IDE or PlatformIO.
Install dependencies:
- Adafruit_SSD1306
- Adafruit_GFX
- ESP32 board definitions
Connect ESP32 to your PC.
run the read mac adress simulator, and write that into the code of the opposite controller.

Open controller.ino or bridge.ino.
Select the correct board (ESP32 Dev Module) and COM port.
Click Upload.

## Code Reference
### Controller (Throttle + OLED)
```cpp
// See src/controller.ino
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// ... full controller code ...
