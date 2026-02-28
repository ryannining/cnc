# CNC Controller Project - Agent Guide

## Project Overview

This is an **ESP8266/ESP32-based CNC Controller** firmware written in C++/Arduino. It controls 3-axis CNC machines (X, Y, Z) with features like motion planning, input shaping, homing, and web-based control.

**Key Features:**
- 3-axis motion control with stepper motors
- Input shaping (ZVD, MZV, EI) for vibration reduction
- G-code parsing (text and binary .gx format)
- Web interface + WebSocket for real-time control
- ESP-NOW protocol for communication with CNC Manager
- LittleFS for file storage
- OTA updates
- **Zero-cross AC detection** dengan interrupt untuk kontrol AC load
- **TOOLON pin** terpisah dari PWM untuk kontrol laser yang lebih presisi
- **Power-fail auto-resume** - Simpan dan restore posisi mesin saat mati listrik

---

## Architecture

### Core Files

| File | Purpose |
|------|---------|
| `cnc.ino` | Main firmware (~92KB) - setup, loop, motion control, ESP-NOW, WiFi |
| `gcode.cpp/h` | G-code parsing (text and .gx binary format) |
| `ui.cpp` | LCD UI handling and input processing |
| `karyalcd.cpp/h` | LCD driver library |
| `structures.h` | Shared data structures, constants, config defaults |

### Data Structures (structures.h)

**Key Structs:**
- `tmove` - Motion segment buffer entry
- `AxisConfig` - Per-axis configuration (vel, acc, steps/mm, pins, shaper)
- `AxisShaper` - Input shaper state for vibration control
- `MotionConfig` - Global motion settings
- `ToolConfig` - Spindle/Laser PWM configuration (lihat section Tool Control)
- `HomingConfig` - Endstop and homing parameters
- `SystemConfig` - Misc system settings
- `ResumeState` - Power-fail state untuk auto-resume

**Buffers:**
- `moves[NUMBUFFER]` - Motion segment ring buffer (32 ESP8266 / 64 ESP32)
- Step event buffer queue (`sebhead`, `sebtail`)
- `pwmTokenQueue` - Queue Token RLE PWM (Run-Length Encoding) untuk efisiensi memori sinkronisasi spasial PWM ke steps riil (kebal dari delay getaran shaper)

---

## Command System

### ESP-NOW Commands (cnc.ino)

```cpp
#define CMD_START   0x01  // param: filename
#define CMD_STOP    0x02  // no param
#define CMD_PAUSE   0x03  // no param
#define CMD_RESUME  0x04  // no param
#define CMD_HOME    0x05  // param: "X", "Y", "Z", "ALL"
#define CMD_JOG     0x06  // param: "X +10", "Y -5", etc.
#define CMD_RESET   0x07  // no param - reset position
```

**Handler:** `handleManagerCommand(ESPNowMessage *msg)` in cnc.ino

### HTTP API Endpoints (cnc.ino setupRoutes())

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/config` | GET/POST | Read/write configuration |
| `/api/files` | GET | List files in LittleFS |
| `/upload` | GET/POST | File upload (supports gzip) |
| `/api/delete` | POST | Delete file |
| `/api/job/start` | POST | Start job from file |
| `/api/job/preview` | POST | Preview job boundaries |
| `/api/file` | GET | Read file content |
| WebSocket | WS | Real-time status at port 81 |

---

## Motion Control System

### Key Variables
```cpp
tmove moves[NUMBUFFER];  // Motion buffer
uint32_t head, tail;     // Buffer indices
tmove *m;                // Active motion pointer
float cx1, cy1, cz1;     // Current position (mm)
```

### Motion Planning Functions
- `addmove(...)` - Queue a motion (menghitung limit maxv/ac segmen baru)
- `backforward()` - O(1) Optimized Forward/Backward pass planner. Iterasi Break Awal sangat menghemat CPU.
- `segmentloop()` & `startmove()` - Runtime loop controller transisi hot-path segmen
- `stepgen()` - Generate step pulses & delay timing tanpa operasi mathematical division lambat per-ticks. Menggunakan cache constants speed.

### Input Shaping
- Configured per-axis via `AxisConfig.shaperType` (0=none, 1=ZVD, 2=MZV, 3=EI)
- Implemented in `stepgen()` using impulse convolution
- `initShaper()` - Initialize shaper coefficients

### Tool Control (Spindle/Laser)

**PWM Modes:**
- `pwmMode = 0` - DDA Dither (High frequency, default)
- `pwmMode = 1` - PWM Duty Cycle (250Hz, untuk laser PSU yang lambat)
- `pwmMode = 2` - Zero Cross PWM (untuk kontrol AC dengan zero-cross detection)

**Pin Configuration:**
- `pinPwm` - Pin untuk kontrol power analog (PWM voltage)
- `pinToolOn` - Pin terpisah untuk kontrol On/Off (255 = NC/Not Connected)
- `pinZeroCross` - Pin untuk deteksi zero-cross AC (default GPIO22)
- `useZeroCross` - Enable/disable zero-cross detection
- `invertToolOn` - Invert logic TOOLON pin

**Zero Cross Detection:**
- Menggunakan interrupt eksternal pada pin zero-cross
- Auto-calibrate half-cycle menggunakan exponential moving average
- Implementasi di `zeroCrossISR()` (cnc.ino)
- Threshold: deteksi mati PLN jika tidak ada ZC dalam 100ms

**TOOLON Pin Logic:**
- TOOLON pin High saat PWM > 0 (laser/tool aktif)
- Memungkinkan pemisahan kontrol power (analog) dari on/off (digital)
- Ideal untuk laser PSU yang memiliki pin terpisah untuk voltage control dan enable

**Laser Safety / Anti-Burn Failsafe:**
- Mengandalkan kombinasi `sebempty` (G-code WiFi terlambat datang) dan `shaperEmpty` (getaran langkah mekanik tuntas) di dalam ISR.
- Apabila mesin kelaparan data/delay di tengah job, program akan otomatis mem-Bypass memori status RLE Token dengan me-nol-kan (`apwmValue = 0`) laser tembak, sampai koneksi suplai Data pulih kembali.

---

## Configuration

### Default Config Location
`structures.h` contains `configJson[]` PROGMEM string with factory defaults.

### Config Files (LittleFS)
- `/config.json` - User configuration
- Format matches the JSON in `configJson[]`

### Key Config Sections
```json
{
  "wifi": { "ssid": "...", "password": "...", "useStatic": true, "lastOctet": 46 },
  "X": { "vel": 200, "acc": 1000, "stepsPerMm": 100, "shaperType": 1, "pinStep": 4, "pinDir": 5 },
  "Y": { "vel": 60, "acc": 200, "stepsPerMm": 190, "shaperType": 1, "pinStep": 12, "pinDir": 13 },
  "Z": { "vel": 50, "acc": 500, "stepsPerMm": 80, "shaperType": 0, "pinStep": 14, "pinDir": 16 },
  "tool": { "pinPwm": 2, "scale": 100, "mode": 0, "pwmMode": 0 },
  "homing": { "pinEndstop": 16, "homingSpeed": 100, "homingPullback": 2 },
  "tool": { "pinPwm": 2, "pinToolOn": 255, "pinZeroCross": 22, "scale": 100, "mode": 0, "pwmMode": 0, "useZeroCross": false, "invertToolOn": false }
}
```

### Power-Fail Auto-Resume

**Deteksi Mati PLN:**
- Menggunakan zero-cross detection untuk deteksi mati PLN
- Jika tidak ada zero-cross dalam 100ms (`POWER_FAIL_THRESHOLD_MS`), dianggap mati PLN
- Deteksi dilakukan di main loop, bukan di ISR

**Force Stop:**
- Saat mati PLN terdeteksi, flag `forceStop` di-set
- `shaperISR()` memeriksa flag ini dan segera:
  - Membersihkan semua buffer (step event, laser event, motion buffer)
  - Reset shaper accumulators
  - Mematikan tool (PWM dan TOOLON)
  - Reset job flags

**Save State:**
- State disimpan dalam format binary (24 byte) ke `/powerfail` di LittleFS
- Format: `[realStepX][realStepY][realStepZ][jobX][jobY][jobZ]` (12 bytes int32 + 12 bytes float)
- Simpan step count (integer) alih-alih koordinat mm untuk presisi penuh
- File dihapus setelah berhasil di-load

**Auto-Resume:**
- Saat boot, cek file `/powerfail`
- Jika ada, load dan restore:
  - `shapers[i].realStep` = step count tersimpan
  - `JobX/Y/Z` = posisi awal job tersimpan
- Posisi mesin langsung benar tanpa perlu homing
- Sangat berguna untuk mesin tanpa endstop

**Testing:**
1. Jog mesin ke posisi tertentu
2. Cabut kabel listrik
3. Tancapkan lagi
4. Koordinat LCD harus menunjukkan posisi sebelum dicabut

---

## G-Code Support

### Supported Commands
- `G0/G1` - Linear move (rapid/feed)
- `G28` - Home
- `M3/M4/M5` - Spindle control
- `M2/M30` - Program end

### File Formats
1. **Text G-code** (.gcode, .nc, .txt) - Standard format
2. **Binary GX** (.gx) - Compressed binary format with header "GX" + version byte

---

## Hardware Abstraction

### Pin Mapping
ESP8266 uses D0-D10 defines, ESP32 uses IOxx defines. Mapped in `structures.h`.

### Fast GPIO
```cpp
#if defined(ESP8266)
  #define GPOSet(mask) GPOS = (mask)
  #define GPOClear(mask) GPOC = (mask)
#elif defined(ESP32)
  #define GPOSet(mask) GPIO.out_w1ts = (mask)
  #define GPOClear(mask) GPIO.out_w1tc = (mask)
#endif
```

---

## Coding Conventions

### Naming
- Functions: camelCase (`addmove`, `planMoves`)
- Globals: often short names (`cx1`, `m`, `cfg`)
- Structs: lowercase with prefix (`tmove`, `axisConfig`)
- Defines: UPPERCASE with underscores

### Language Mix
Code comments are primarily in **Indonesian** (e.g., "Memulai filesystem", "gerakan", "kecepatan")

### Important Macros
```cpp
#define log(...)  // Conditional serial logging
#define nextbuff(x) ((x + 1) & (NUMBUFFER - 1))  // Ring buffer macro
#define fmax(a,b) ((a < b) ? b : a)  // Float max
```

---

## Common Tasks

### Adding a New ESP-NOW Command
1. Add `#define CMD_NEW 0x08` in cnc.ino (line ~65-71)
2. Add case in `handleManagerCommand()` (line ~744+)
3. Update CNC Manager to send the new command

### Adding HTTP Endpoint
1. Add `server.on("/api/new", ...)` in `setupRoutes()` (line ~2309+)
2. Follow existing patterns for JSON responses

### Modifying Motion Planning
- Core planner in `planMoves()` and `moveruntime()`
- Check `modifyRunningSegment()` for feed override during motion
- Buffer management uses `head`/`tail` indices with `nextbuff()` macro

### Configuration Changes
- Add field to appropriate struct in `structures.h`
- Update `configJson[]` default values
- Update `loadAllConfig()` and save functions in cnc.ino

---

## Debugging

### Serial Output
- Use `log("format", args...)` for conditional logging
- Set `debuglog 1` in structures.h to enable

### WebSocket
- Real-time status broadcast on port 81
- Connect to `ws://<cnc-ip>:81`

---

## Motion Buffer Pipeline

### Overview Pipeline
```
File G-code (LittleFS)
  ↓ processJob() — loop sampai bufffull atau timeout 5ms
Move Buffer [NUMBUFFER=32/64 slot] (tmove)
  ↓ yield() / motionloop() — loop multi-segment sampai step buffer 75% penuh
Step Event Buffer [STEP_EVENT_BUFFER_SIZE=2048 slot]
  ↓ shaperISR() — setiap TICK_MICROS (16µs)
Motor Step Output
```

### processJob() — G-code Parsing
```cpp
// Isi move buffer sampai penuh atau file habis, max 5ms per siklus
bool bufferSafe = (sebFree < (STEP_EVENT_BUFFER_SIZE >> 2));
while (!bufffull && !isParsingComplete && jobRunning) {
    parseNextGCodeSegments();
    if (millis() - parseStart >= 5) break;  // max 5ms
    if (!bufferSafe) break;                 // stop jika buffer belum aman (prioritas ISR)
}
```
- Harus ada guard `jobRunning` agar tidak infinite loop saat idle
- `bufferSafe`: jika step event buffer masih kosong (< 75%), cukup parse 1x saja

### yield() — Step Event Buffer Fill
- Override `yield()` di ESP8266 untuk mengisi step event buffer
- Loop multi-segment: panggil `segmentloop()` + `startmove()` sampai buffer 75% penuh
- `server.handleClient()` hanya dipanggil jika step event buffer ≥ 75% penuh atau job tidak running
- Interval handleClient: 100ms minimum

### Buffer Health Constants
```cpp
STEP_EVENT_BUFFER_SIZE = 2048  // step events
NUMBUFFER = 32 (ESP8266) / 64 (ESP32)  // move segments
POSITION_HISTORY_SIZE = 15     // record per WS broadcast
// Record interval ≈ 50ms → broadcast interval ≈ 750ms
```

### FREEZE Recovery (shaperISR)
```cpp
bool DETECTFREEZE = false;  // Hanya aktif setelah 5 detik job berjalan
// DETECTFREEZE diset di processJob():
DETECTFREEZE = !(stopping || buffempty) && (millis()-jobStartTime > 5000);

if (UNLIKELY(FREEZING && DETECTFREEZE)) {
    // Ramp recovery: scale realV dari 0→100% dalam RECOVERY_TIME_US=2048 ticks (32ms)
    realV = (realV * scale) >> 10;
    if (realV < (1<<12)) realV = (1<<12);  // minimum speed agar tidak crash div-by-zero
}
```

---

## Web UI — Buffer Health & Freeze Detection

### Buffer Health Display
- Ditampilkan di coord overlay: `BUF: X% / Y%`
  - `X%` = Step Event Buffer health (`sebHealth`, 0–100%)
  - `Y%` = Move Buffer health (`moveHealth`, 0–100%)
- Dikirim firmware sebagai **2 byte tambahan** di akhir paket WS `0x01` (setelah job filename)
- Warna: hijau ≥75%, kuning ≥40%, merah <40%

### WebSocket Packet Format `0x01`
```
[0x01][count][pos×16 bytes each][jobRunning][jobPaused][progress][nameLen][name...][sebHealth][moveHealth]
```
- Setiap pos = `int32_t X, Y, Z, L` (little-endian, ×100mm)
- 2 byte health di paling akhir (backward compatible: cek `data.length > afterName + 1`)

### Freeze Detection (Web UI)
Dilakukan di `renderCanvas()` yang dipanggil tiap 70ms via `setInterval`:
```js
// Di dalam renderCanvas, setelah currentPosition di-assign:
if (posisi sama dengan lastRenderedPos) {
    stillMs = Date.now() - lastPosChangeTime;
    if (stillMs >= 1000ms && !isFreezing) → catat freeze point
} else {
    lastPosChangeTime = Date.now();  // posisi bergerak, reset timer
    lastRenderedPos = {x, y, z};
}
```
- Freeze marker tampil di canvas: lingkaran merah (aktif) / orange (selesai)
- Radius sebanding durasi freeze, label `⚠ FREEZE 1.2s`
- Reset saat job baru dimulai, tetap tampil setelah job selesai

### Canvas Persistence
- `posHistory` **tidak dihapus** saat WebSocket reconnect jika job sudah selesai
- Hanya dihapus saat reconnect jika job **sedang running** (data mungkin stale)
- Ini memastikan path job besar tetap tampil meski user pergi dan kembali

---

## Build & Flash

> **QUICK NOTE (ingat selalu):** compile + flash = `./script/espcompile.sh 192.168.1.46 /mnt/smbshare/karyacontroller/javascript/cnc/cnc.ino` (IP mesin user untuk OTA). Tanpa IP = **compile saja**: `./script/espcompile.sh /mnt/smbshare/karyacontroller/javascript/cnc/cnc.ino`. Flash saja = `./script/espflash.sh 192.168.1.46 firmware.bin`. Script juga ada di home (`~/espcompile.sh`, `~/espflash.sh`).

Firmware di-compile dan di-flash ke mesin CNC memakai script shell (Arduino CLI based, ESP8266 D1 Mini). Script aslinya terletak di home directory: `~/espcompile.sh` dan `~/espflash.sh` (yang ada di `script/` project ini hanyalah salinan/kopian).

**Compile + Flash:**
```bash
~/espcompile.sh <target> <file.ino>
```
- `<target>` = IP address mesin (OTA flash) atau `/dev/ttyUSB0` (USB flash)
- Jika `<target>` diisi, otomatis lanjut flash via `~/espflash.sh` setelah compile sukses

**Compile saja (tanpa flash):**
```bash
~/espcompile.sh /mnt/smbshare/karyacontroller/javascript/cnc/cnc.ino
```
Tanpa IP address = cuma compile, tidak flash. Contoh yang biasa dipakai user:
```bash
~/espcompile.sh 192.168.1.46 /mnt/smbshare/karyacontroller/javascript/cnc/cnc.ino   # compile + OTA ke mesin
~/espcompile.sh /mnt/smbshare/karyacontroller/javascript/cnc/cnc.ino                  # compile saja
```

**Flash saja:**
```bash
~/espflash.sh 192.168.1.46 firmware.bin     # OTA
~/espflash.sh /dev/ttyUSB0 firmware.bin     # USB serial
```

Script versi ESP32: `~/esp32compile.sh` dan `~/esp32flash.sh` (pemakaian sama, target = IP OTA / `/dev/ttyUSB0`).

Dokumentasi bug/analisis riwayat: `bug.md` (jurnal semua fix), `karyalcd_buganalysis.md`, `cnc_buganalysis/ui_cpp.md`.

- `espcompile.sh` memakai arduino-builder (ESP8266 core 3.1.2), FQBN `esp8266:esp8266:d1_mini`, build/cache di `/tmp/arduino_build_*`
- `espflash.sh` memakai espota.py (port 8266) untuk OTA, atau upload.py untuk USB
- Output BIN firmware ditempatkan di `/tmp/arduino_build_<hash>/`

---

## Dependencies

**PlatformIO Libraries** (platformio.ini):
- WebSockets by Markus Sattler
- ArduinoJson by Benoit Blanchon

**Built-in:**
- LittleFS
- ESP-NOW
- ArduinoOTA

---

## Related Projects

- `cnc_man/` - CNC Manager (companion project for multi-CNC control)
- `karyalcd.h/cpp` - Custom LCD driver

---

## Notes for AI Assistants

1. **Always check structures.h first** for data structure definitions and constants
2. **Buffer operations** use bitwise AND for ring buffer wrapping (`& (NUMBUFFER - 1)`)
3. **Motion units**: mm for distance, mm/min for feed, mm/s² for acceleration
4. **Timer-based**: Motion control runs in ISR context - be careful with blocking operations
5. **ESP-NOW is primary control interface** - HTTP is secondary
6. **Indonesian comments** - Use translation if meaning is unclear
7. **Optimasi** - Selalu prefer optimized code untuk ESP8266 microcontroller