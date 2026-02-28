# CNC Controller Firmware

Firmware CNC 3-axis untuk ESP8266/ESP32. Motion planning, input shaping (ZVD/MZV/EI), parsing G-code (text + .gx), web interface + WebSocket, ESP-NOW, LittleFS, OTA, zero-cross AC detection, power-fail auto-resume.

## Struktur

| File | Fungsi |
|------|--------|
| `cnc.ino` | Main firmware: setup, loop, motion control, ESP-NOW, WiFi |
| `gcode.cpp/h` | Parsing G-code (text & binary .gx) |
| `ui.cpp` | LCD UI handling |
| `karyalcd.cpp/h` | LCD driver |
| `structures.h` | Data structures, constants, config defaults |
| `data/` | Web UI (served dari LittleFS) |
| `script/` | Compile & flash scripts (Linux + Windows) |

## Compile

### Linux / macOS (bash)

```bash
# ESP8266
./script/espcompile.sh cnc.ino                      # compile saja
./script/espcompile.sh 192.168.1.46 cnc.ino         # compile + OTA flash
./script/espcompile.sh /dev/ttyUSB0 cnc.ino         # compile + USB flash
```

```bash
# ESP32 (default board lolin_c3_mini)
./script/esp32compile.sh cnc.ino
./script/esp32compile.sh esp32 192.168.1.46 cnc.ino       # board esp32 classic
./script/esp32compile.sh esp32c3 /dev/ttyUSB0 cnc.ino     # board C3 via USB
```

Board ESP32 yang tersedia: `esp32`, `esp32c3`, `esp32s2`, `esp32s3`, `lolin_c3_mini`, `lolin_s2_mini`, `lolin_s3_mini`, atau full FQBN `esp32:esp32:...`.

### Windows (batch)

```bat
:: ESP8266
script\espcompile.bat cnc.ino
script\espcompile.bat 192.168.1.46 cnc.ino

:: ESP32
script\esp32compile.bat cnc.ino
script\esp32compile.bat esp32 192.168.1.46 cnc.ino
```

## Persyaratan

1. **Arduino CLI toolchain** (arduino-builder + toolchains):
   - ESP8266 core 3.1.2 (`esp8266:esp8266:d1_mini`)
   - ESP32 core 3.3.6 (`esp32:esp32`)
   - Install via Arduino IDE Boards Manager, atau arduino-cli:
     ```bash
     arduino-cli core install esp8266:esp8266
     arduino-cli core install esp32:esp32
     ```
2. **Python 3** (dibutuhkan esptool untuk flashing)
3. **Libraries** (install via Arduino IDE Library Manager):
   - **WebSockets** by Markus Sattler (`Links2004/arduinoWebSockets`)
   - **ArduinoJson** by Benoit Blanchon (v6.x)
   - Built-in (sudah termasuk di core): `ESP8266WiFi`, `ESP8266WebServer`, `LittleFS`, `WiFi`, `WebServer`, `FS`, `ESP-NOW`, `ArduinoOTA`

Alternatif PlatformIO: `platformio.ini` sudah disediakan:

```bash
pio run -e esp8266 -t upload
pio run -e esp32 -t upload
```

## Flash

- **OTA**: target berupa IP address mesin (port 8266)
- **USB**: target berupa serial port (`/dev/ttyUSB0`, `/dev/ttyACM0`, `COM3`, dst.)

## Konfigurasi

Default config ada di `structures.h` (`configJson[]` PROGMEM). User config tersimpan di `/config.json` pada LittleFS. Edit via web interface `/api/config`.
