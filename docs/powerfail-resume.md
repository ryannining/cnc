# Power-Fail Auto-Resume Feature

## Overview

Fitur Power-Fail Auto-Resume memungkinkan CNC controller untuk **menyimpan posisi mesin saat mati listrik** dan **mengembalikannya secara otomatis** saat mesin dinyalakan kembali. Fitur ini sangat berguna untuk mesin CNC yang **tidak memiliki endstop**, karena posisi mesin tidak akan hilang meskipun terjadi pemadaman listrik.

## How It Works

### 1. Deteksi Mati PLN

Deteksi menggunakan **zero-cross detection** pada AC line:

```cpp
// Di main loop
if (toolCfg.useZeroCross && !powerFailDetected && !stateSaved) {
    uint32_t elapsedSinceZC = millis() - lastZeroCrossTimeMs;
    if (elapsedSinceZC > POWER_FAIL_THRESHOLD_MS && lastZeroCrossTimeMs > 0) {
        powerFailDetected = true;
        savePowerFailState();
    }
}
```

- **POWER_FAIL_THRESHOLD_MS**: 100ms (jika tidak ada ZC dalam 100ms = mati PLN)
- Zero-cross interrupt (`zeroCrossISR`) mencatat waktu terakhir ZC
- Deteksi dilakukan di main loop (bukan ISR) untuk menghindari blocking

### 2. Force Stop (Emergency Halt)

Saat mati PLN terdeteksi, sistem melakukan **force stop** untuk menghentikan semua aktivitas:

```cpp
// Di shaperISR() - diperiksa setiap tick
if (UNLIKELY(forceStop)) {
    // Bersihkan semua buffer
    sebhead = sebtail;  // Step event buffer
    lebhead = lebtail;  // Laser event buffer
    head = tail;        // Motion buffer
    
    // Reset accumulators
    for (int i = 0; i < 3; i++) {
        shapers[i].accumulator = 0;
        shapers[i].signedAccum[0] = 0;
        shapers[i].signedAccum[1] = 0;
    }
    
    // Matikan tool
    pwmValue = 0;
    GPOClear(maskToolPwm);
    if (toolCfg.pinToolOn < 255) {
        GPOClear(maskToolOn);
    }
    
    // Reset flags
    jobRunning = false;
    jobPaused = false;
    stopping = false;
    
    return;
}
```

### 3. Save State

Posisi mesin disimpan dalam **format binary** (bukan JSON) untuk kecepatan:

```cpp
// Format: 24 byte total
// [4 bytes] realStep X (int32_t)
// [4 bytes] realStep Y (int32_t)  
// [4 bytes] realStep Z (int32_t)
// [4 bytes] JobX (float)
// [4 bytes] JobY (float)
// [4 bytes] JobZ (float)

int32_t realSteps[3] = {
    shapers[0].realStep,
    shapers[1].realStep,
    shapers[2].realStep
};
float jobPos[3] = {JobX, JobY, JobZ};

File f = LittleFS.open("/powerfail", "w");
f.write((uint8_t*)realSteps, sizeof(realSteps));
f.write((uint8_t*)jobPos, sizeof(jobPos));
```

**Kenapa binary?**
- ⚡ **Jauh lebih cepat** - hanya 2x write operation
- 💾 **Lebih kecil** - 24 byte vs ratusan byte JSON
- 🔧 **Sederhana** - tidak perlu library JSON
- 🎯 **Presisi penuh** - simpan integer step count, bukan float mm

### 4. Load State & Auto-Resume

Saat boot, sistem otomatis memeriksa dan memuat state:

```cpp
void setup() {
    // ... setup code ...
    
    if (loadPowerFailState()) {
        log("!!! POWER-FAIL STATE DETECTED - Auto Resume !!!\n");
    }
    
    if (needResume) {
        // Restore step count (tanpa gerakan fisik!)
        shapers[0].realStep = resumeState.realStepX;
        shapers[1].realStep = resumeState.realStepY;
        shapers[2].realStep = resumeState.realStepZ;
        
        // Update cx1, cy1, cz1 (koordinat dalam mm)
        updateRealPos();
        
        // Restore job start position
        JobX = resumeState.jobX;
        JobY = resumeState.jobY;
        JobZ = resumeState.jobZ;
        
        // Hapus file setelah load
        needResume = false;
    }
}
```

**Perhitungan koordinat:**
```cpp
void updateRealPos() {
    cx1 = shapers[0].realStep * shapers[0].invStepsPerMm;
    cy1 = shapers[1].realStep * shapers[1].invStepsPerMm;
    cz1 = shapers[2].realStep * shapers[2].invStepsPerMm;
}
```

## Hardware Requirements

### Minimal Setup
- **Zero-cross detector** sederhana (optocoupler + resistor)
- **Supercap/battery backup** untuk ESP32 (beberapa detik cukup)

### Zero-Cross Detector Circuit
```
AC Line ----[100kΩ]----+----[Optocoupler]---- GPIO22
                       |
                      [Bridge Rectifier]
                       |
AC Neutral -----------+
```

## Testing Procedure

1. **Jog** mesin ke posisi tertentu (misal: X=50, Y=30, Z=5)
2. **Cabut** kabel listrik
3. **Tancapkan** kembali
4. **Cek LCD** - koordinat harus tetap **X=50, Y=30, Z=5** (bukan 0,0,0)

## Configuration

### Web UI
Konfigurasi di section **Tool / Spindle**:
- **Zero Cross Pin**: GPIO pin untuk ZC detection (default: IO22)
- **Use Zero Cross**: Enable/disable deteksi

### Manual (config.json)
```json
{
  "tool": {
    "pinPwm": 15,
    "pinToolOn": 255,
    "pinZeroCross": 22,
    "scale": 50,
    "mode": 1,
    "pwmMode": 0,
    "useZeroCross": true,
    "invertToolOn": false
  }
}
```

## Code Structure

### Key Variables
```cpp
// Power-fail detection
volatile bool powerFailDetected = false;
volatile bool stateSaved = false;
volatile bool forceStop = false;
uint32_t lastZeroCrossTimeMs = 0;
#define POWER_FAIL_THRESHOLD_MS 100

// Resume state
struct ResumeState {
    int32_t realStepX, realStepY, realStepZ;
    float jobX, jobY, jobZ;
    bool valid;
};
ResumeState resumeState;
bool needResume = false;
```

### Key Functions
- `savePowerFailState()` - Simpan state ke LittleFS
- `loadPowerFailState()` - Load state dari LittleFS
- `zeroCrossISR()` - Interrupt handler untuk ZC detection
- `shaperISR()` - Cek forceStop flag setiap tick

## Benefits

1. **No Endstop Required** - Mesin tanpa endstop tetap bisa menjaga posisi akurat
2. **Automatic** - Tidak perlu tindakan manual setelah mati listrik
3. **Fast** - Binary save/load hanya butuh waktu microseconds
4. **Reliable** - Force stop memastikan mesin berhenti aman
5. **Job Resume Ready** - Posisi job tersimpan untuk resume nanti

## Limitations

1. **Butuh Zero-Cross Hardware** - Perlu rangkaian ZC detector
2. **Butuh Power Backup** - ESP32 harus tetap hidup beberapa detik setelah mati PLN
3. **Tidak Menyimpan File Job** - User harus ingat file job yang sedang dijalankan
4. **One-time Save** - Hanya menyimpan sekali per power-fail event

## Related Features

- **Zero-Cross PWM** - Mode PWM untuk kontrol AC load
- **TOOLON Pin** - Pin terpisah untuk on/off kontrol laser

## See Also

- `cnc.ino` - Implementasi utama (line ~1925-2020)
- `structures.h` - Definisi struktur data
- AGENTS.md - Dokumentasi fitur lengkap