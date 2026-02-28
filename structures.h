// Common structures shared between ui.cpp and cnc.ino

#ifndef STRUCTURES_H
#define STRUCTURES_H

// Uncomment baris ini untuk menggunakan pin default yang aman bagi ESP32-C3
// (Tidak memakai GPIO 12-17 yang terhubung ke SPI Flash)
// #define ESP32C3

#include <stdint.h>

#if defined(ESP32)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

// -----------------------------------------------------------------------------
// Makro Helper
// -----------------------------------------------------------------------------
#define debuglog 1
extern bool canUseSerial;   // didefinisi di cnc.ino; keadaan final ditentukan loadAllConfig()
#define log0(...)                                                              \
  if (canUseSerial) {                                                          \
    Serial.printf(__VA_ARGS__);                                                \
  }

#if debuglog
#define log(...)                                                               \
  if (__builtin_expect(canUseSerial, 0)) {                                                          \
    Serial.printf(__VA_ARGS__);                                                \
  }
#else
#define log(...)
#endif

// -----------------------------------------------------------------------------
// Multicore Sync Macros (ESP32)
// -----------------------------------------------------------------------------
#if defined(ESP32)
extern portMUX_TYPE cncMux;
// #define SYNC_START() 
// #define SYNC_END() 
#define SYNC_START() portENTER_CRITICAL(&cncMux)
#define SYNC_END() portEXIT_CRITICAL(&cncMux)
// #define SYNC_START_ISR() 
// #define SYNC_END_ISR() 
#define SYNC_START_ISR() portENTER_CRITICAL_ISR(&cncMux)
#define SYNC_END_ISR() portEXIT_CRITICAL_ISR(&cncMux)
#else
#define SYNC_START() noInterrupts()
#define SYNC_END() interrupts()
#define SYNC_START_ISR() noInterrupts()
#define SYNC_END_ISR() interrupts()
#endif
#define LIKELY(x) __builtin_expect((x), 1)
#define UNLIKELY(x) __builtin_expect((x), 0)
// #define LIKELY(x) (x)
// #define UNLIKELY(x) (x)

#if defined(ESP8266)
#define D0 16
#define D1 5
#define D2 4
#define D3 0
#define D4 2
#define D5 14
#define D6 12
#define D7 13
#define D8 15
#define D9 3
#define D10 1
#define TX 1
#define RX 3
#define CPU_TYPE "ESP8266"
#elif defined(ESP32)
#define IO15 15
#define IO2 2
#define IO0 0
#define IO4 4
#define IO16 16
#define IO17 17
#define IO5 5
#define IO18 18
#define IO19 19
#define IO21 21
#define IO3 3
#define IO1 1
#define IO22 22
#define IO23 23
#define IO13 13
#define IO12 12
#define IO14 14
#define IO27 27
#define IO26 26
#define IO25 25
#define IO33 33
#define IO32 32
#define D0 IO16
#define D1 IO5
#define D2 IO4
#define D3 IO0
#define D4 IO2
#define D5 IO14
#define D6 IO12
#define D7 IO13
#define D8 IO15
#define D9 IO3
#define D10 IO1
#define TX2 TX2
#define RX2 RX2
#define TX TX
#define RX RX
#define A0 A0
#if defined(ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C3)
#define CPU_TYPE "ESP32C3"
#else
#define CPU_TYPE "ESP32"
#endif
#endif

// Position Record Structure
struct PositionRecord {
  int32_t X, Y, Z;
  int32_t l;
};
extern int lcd_rst;
extern int lcd_sda;
extern int lcd_sda2;
extern int lcd_contrast;
extern int lcd_scl;
extern int lcd_addr;
extern int lcd_kind;

extern const char *cpuType;

extern volatile uint32_t head;
extern volatile uint32_t tail;
// -----------------------------------------------------------------------------
// Konstanta Buffer dan Makro
// -----------------------------------------------------------------------------
#define NUMAXIS 3 // Jumlah axis (X, Y, Z)

#if defined(ESP8266)
#define TICK_MICROS 16              // Interval timer dalam mikrodetik
#define NUMBUFFER 128                // Jumlah buffer gerakan
#define STEP_EVENT_BUFFER_SIZE 2048 // Ukuran buffer event step
#define SHAPER_BUFFER_SIZE 2048     // Ukuran buffer input shaper
#elif defined(ESP32)
#define TICK_MICROS 16              // Interval timer dalam mikrodetik
#define NUMBUFFER 128                // Jumlah buffer gerakan
#define STEP_EVENT_BUFFER_SIZE 2048*2 // Ukuran buffer event step
#define SHAPER_BUFFER_SIZE 2048*2     // Ukuran buffer input shaper
#endif

#define CLOCKCONSTANT (1000000) / TICK_MICROS // Konstanta clock

// Constants needed for Shaper
#define SHAPER_SCALE 128 // Skala input shaper (dinaikkan karena shaperBuffer kini int16_t)

// Struktur Impulse untuk Input Shaper
struct Impulse {
  int16_t weight; // Bobot impulse (skala SHAPER_SCALE)
  int16_t delay;  // Delay impulse dalam tick timer
};

// Global Shaper Buffer - Struct of Arrays (SOA) layout
// Layout: [tick_index][axis] = data untuk tick tersebut, hanya X dan Y yang pakai shaper
// Z axis tidak pakai shaper buffer (langsung step)
// Keuntungan: Cache locality lebih baik saat ISR proses axis dalam satu tick
// Hemat memory: hanya 2 axis (X,Y) x SHAPER_BUFFER_SIZE x 2 bytes
// int16_t: positif = forward, negatif = backward. Cancel-out terdeteksi di pushStep via ghost energy.
// Dialokasi malas di heap via syncShaperBuffer(), NULL saat shaping OFF.
extern int16_t (*shaperBuffer)[2];
void syncShaperBuffer();

// Makro untuk akses buffer shaper dengan SOA layout
// shaperBuffer[tick][axis] dimana axis: 0=X, 1=Y
// Z axis (2) tidak ada di buffer ini
#define SHAPER_BUF(tick, axis) shaperBuffer[(tick) & (SHAPER_BUFFER_SIZE - 1)][axis]

struct AxisShaper {
  // Group 4-byte fields (8 fields = 32 bytes)
  volatile int32_t accumulator;     // Akumulator untuk Error Diffusion
  volatile int32_t realStep;        // Posisi motor real (setelah shaper)
  uint32_t stepMask;       // Masker bit untuk Fast GPIO
  float stepsPerMm;        // Konversi unit
  float invStepsPerMm;     // Kebalikan konversi unit
  
  // Group 2-byte fields
  // signedAccum[0] = ghost energy accumulator (energi cancel-out, dikreditkan saat pushStep)
  // signedAccum[1] = normal impulse accumulator (drained dari buffer + direct steps)
  // FIX 2026-08-30: volatile dihapus — hanya diakses dari shaperISR (satu konteks ISR yang
  // sama); volatile cuma memaksa memory round-trip tiap akses & mematikan register caching
  int16_t signedAccum[2];
  int16_t maxDelayTicks;   // maks delay input shaper
  
  // Impulse array (4 × 4 bytes = 16 bytes)
  Impulse impulses[4];     // Daftar impulse untuk shaper (max 4 impulse untuk ZVD/EI)
  
  // Group 1-byte fields di akhir (4 bytes total, no padding)
  uint8_t stepPin;         // Pin fisik step (0-255 cukup untuk pin numbers)
  int8_t numImpulses;      // Jumlah impulse yang aktif
  volatile int8_t lastDir; // Arah terakhir di ISR
  uint8_t reserved;        // Padding untuk alignment 4-byte
};

// Shaper buffer MACRO 16 bit
// int16_t: nilai negatif = backward, positif = forward
// Cancel-out terdeteksi di pushStep via ghost energy accumulator (signedAccum[0])

// Makro untuk buffer circular
#define nextbuff(x) ((x + 1) & (NUMBUFFER - 1))
#define prevbuff(x) ((x - 1) & (NUMBUFFER - 1))
#define fmax(a, b) ((a < b) ? b : a)
#define fmin(a, b) ((a > b) ? b : a)
#define fmax3(a, b, c) fmax(a, fmax(b, c))
#define fmin3(a, b, c) fmin(a, fmin(b, c))

// Makro untuk step event buffer
#define nextseb(x) ((x + 1) & (STEP_EVENT_BUFFER_SIZE - 1))
#define prevseb(x) ((x - 1) & (STEP_EVENT_BUFFER_SIZE - 1))
#define sebfull (nextseb(sebhead) == sebtail)
#define sebempty (sebhead == sebtail)
#define seblength ((sebhead - sebtail + STEP_EVENT_BUFFER_SIZE) & (STEP_EVENT_BUFFER_SIZE - 1))

// Makro untuk laser event buffer
#define nextleb(x) ((x + 1) & (STEP_EVENT_BUFFER_SIZE * 2 - 1))
#define prevleb(x) ((x - 1) & (STEP_EVENT_BUFFER_SIZE * 2 - 1))
#define lebfull (nextleb(lebhead) == lebtail)
#define lebempty (lebhead == lebtail)

// Bitmask untuk step event (uint32_t)
// Bits 0-5 untuk step/dir (6 bit), bits 16-31 untuk time (16 bit)
#define STEP_EVENT_X_DIR_BIT 0
#define STEP_EVENT_X_STEP_BIT 1
#define STEP_EVENT_Y_DIR_BIT 2
#define STEP_EVENT_Y_STEP_BIT 3
#define STEP_EVENT_Z_DIR_BIT 4
#define STEP_EVENT_Z_STEP_BIT 5

// Makro untuk extract/set time offset (bits 16-31, 16 bit, max ~65536 tick
// untuk ESP32 ~0.8 detik)
#define GET_EVENT_TIME(event) ((event) >> 16)
#define SET_EVENT_TIME(time) ((time) << 16)

// Makro untuk extract/set PWM power di step event (bits 6-13, 8 bit 0-255)
#define GET_EVENT_PWM(event) ((event >> 6) & 255)
#define SET_EVENT_PWM(pwm) ((pwm) << 6)

// Makro status buffer
#define bufffull (nextbuff(head) == tail)
#define buffempty (head == tail)
#define bufflength ((head-tail+NUMBUFFER) & (NUMBUFFER-1))

// Jog control flags untuk parameter jogFlags di addmove()
#define JOG_FLAG_LIMIT  0x01  // Bit 0: batasi jarak buffer jog
#define JOG_FLAG_EXTEND 0x02  // Bit 1: extend segmen jog terakhir

// -----------------------------------------------------------------------------
// Struktur Gerakan (tmove)
// -----------------------------------------------------------------------------
struct tmove {
  // Group 4-byte fields together untuk menghindari padding
  float dis;           // Jarak gerakan (mm)
  int32_t ac;          // Akselerasi (untuk backplanner)
  int32_t delta;       // Delta kecepatan
  int32_t maxs;        // Kecepatan maksimum squared
  int32_t fs;          // Kecepatan start squared
  int32_t fn;          // Kecepatan nominal squared
  int32_t dx[NUMAXIS]; // Delta step setiap axis
  // Group 1-byte fields di akhir
  uint8_t maxv;        // Kecepatan maksimum
  volatile uint8_t status;      // Status gerakan (bit 0: ready, bit 2: planned, bit 3: G0, bit 4-5: fast axis)
  uint8_t toolPower;   // Tool power (S value, 0-255)
  // uint8_t waitG;       // wait until no movement (uint8_t lebih baik dari bool untuk alignment)
};

// -----------------------------------------------------------------------------
// Struktur Konfigurasi Axis (Sumbu)
// -----------------------------------------------------------------------------
struct AxisConfig {
  float vel;        // Kecepatan maksimum (mm/min)
  float acc;        // Akselerasi (mm/s^2)
  float stepsPerMm; // Jumlah step per milimeter
  float freq;       // Frekuensi resonansi untuk input shaper
  float damp;       // Rasio redaman (damping ratio)
  float backlash;   // Kompensasi backlash (mm)
  int shaperType;   // Tipe input shaper: 0=none, 1=ZVD, 2=MZV, 3=EI
  int pinStep;      // Pin step motor
  int pinDir;       // Pin arah motor
  int hasEndstop;   // Apakah axis punya endstop (0=tidak, 1=ya)
};

// -----------------------------------------------------------------------------
// Struktur Konfigurasi Motion (Gerakan)
// -----------------------------------------------------------------------------
struct MotionConfig {
  float junctionDev;       // Deviasi junction untuk cornering
  float minPlannerSpeed;   // Kecepatan minimum planner
  bool enableInputShaping; // Enable input shaping
};

// -----------------------------------------------------------------------------
// Struktur Konfigurasi Tool (Spindle/Alat)
// -----------------------------------------------------------------------------
struct ToolConfig {
  int pinPwm;      // Pin PWM untuk kontrol power (analog voltage)
  int pinToolOn;   // Pin TOOLON untuk on/off control (separate dari PWM)
  int pinZeroCross; // Pin zero-cross detection untuk AC sync (default GPIO22)
  int scale;
  int mode;        // 0 = CNC Spindle, 1 = Laser, laser turn off when G1, spindle is not
  int pwmMode;     // 0 = DDA Dither (default), 1 = PWM Duty Cycle, 2 = Zero Cross PWM
  bool useZeroCross; // Enable zero-cross detection
  bool invertToolOn; // Invert TOOLON pin logic
};

// -----------------------------------------------------------------------------
// Struktur Konfigurasi Homing
// -----------------------------------------------------------------------------
struct HomingConfig {
  int pinEndstop;       // Pin endstop shared untuk semua axis (NC, seri)
  float homingSpeed;    // Kecepatan homing (mm/min)
  float homingPullback; // Jarak pullback setelah endstop terdeteksi (mm)
};

// -----------------------------------------------------------------------------
// Struktur Konfigurasi System
// -----------------------------------------------------------------------------
struct SystemConfig {
  bool broadcastPositionWhenJobRunning; // Broadcast posisi saat job berjalan
  int testLaserPower;      // Power untuk test laser (0-255)
  int testLaserTime;       // Waktu test laser dalam milidetik (ms)
  bool jogLimitEnabled;    // Soft-limit jog (blokir gerak ke area negatif/luar window)
};

extern AxisConfig cfg[3];
extern MotionConfig motionCfg;
extern ToolConfig toolCfg;
extern HomingConfig homingCfg;
extern SystemConfig systemCfg;
static const char uploadHtml[] PROGMEM = R"EOF(
<!DOCTYPE html><html><head><title>CNC Gzip Upload</title><script src='https://cdnjs.cloudflare.com/ajax/libs/pako/2.1.0/pako.min.js'></script><style>body{font-family:sans-serif;padding:20px}.container{max-width:400px;margin:auto;border:1px solid #ccc;padding:20px;border-radius:8px}#status{margin-top:10px;color:blue}</style></head><body><div class='container'><h3>Upload & Compress</h3><input type='file' id='fileInput'><br><br><button onclick='handleUpload()'>Upload ke ESP</button><div id='status'></div></div><script>async function handleUpload(){const fileInput=document.getElementById('fileInput');const status=document.getElementById('status');if(fileInput.files.length===0)return alert('Pilih file!');const file=fileInput.files[0];try{if(typeof pako==='undefined'){throw new Error('Library kompresi tidak tersedia')}if(file.name.endsWith('.json')){throw new Error('File JSON tidak dapat dikompresi');}status.innerText='Mengompresi...';const arrayBuffer=await file.arrayBuffer();const compressed=pako.gzip(new Uint8Array(arrayBuffer));const formData=new FormData(); const newFileName=file.name+'.gz';formData.append('name',new Blob([compressed]),newFileName);status.innerText='Mengirim '+newFileName+'...';const response=await fetch('/upload',{method:'POST',body:formData});if(response.ok){status.innerText='Berhasil! File tersimpan sebagai '+newFileName}else{status.innerText='Gagal upload.'}}catch(e){ status.innerText='Mengupload sebagai file biasa...';const formData=new FormData();formData.append('name',file);const response=await fetch('/upload',{method:'POST',body:formData});if(response.ok){status.innerText='Berhasil! File tersimpan sebagai '+file.name}else{status.innerText='Gagal upload.'}}}</script></body></html>)EOF";

#if defined(ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C3)
static const char configJson[] PROGMEM =
    R"EOF({"wifi":{"ssid":"usaha karya","password":"45712319","lastOctet":46,"useStatic":true},"motion":{"junctionDev":0.05,"minPlannerSpeed":0,"enableInputShaping":false},"X":{"vel":200,"acc":1000,"stepsPerMm":100,"freq":55,"damp":0.1,"backlash":0.0,"shaperType":1,"pinStep":4,"pinDir":5,"hasEndstop":0},"Y":{"vel":60,"acc":200,"stepsPerMm":190,"freq":40,"damp":0.1,"backlash":0.0,"shaperType":1,"pinStep":6,"pinDir":7,"hasEndstop":0},"Z":{"vel":50,"acc":500,"stepsPerMm":80,"freq":50,"damp":0.1,"backlash":0.0,"shaperType":0,"pinStep":8,"pinDir":10,"hasEndstop":0},"tool":{"pinPwm":2,"pinToolOn":255,"pinZeroCross":255,"scale":100,"mode":0,"pwmMode":0,"useZeroCross":false,"invertToolOn":false},"homing":{"pinEndstop":3,"homingSpeed":100,"homingPullback":2},"system":{"broadcastPositionWhenJobRunning":false,"testLaserPower":255,"testLaserTime":1,"jogLimitEnabled":true},"cncInfo":{"name":"CNC Machine 1"},"lcd":{"sda":0,"sda2":5,"rst":0,"scl":1,"addr":60,"kind":4,"contrast":6},"karyacnc":{"ip":"","token":"","reportId":""}})EOF";
#else
static const char configJson[] PROGMEM =
    R"EOF({"wifi":{"ssid":"usaha karya","password":"45712319","lastOctet":46,"useStatic":true},"motion":{"junctionDev":0.05,"minPlannerSpeed":0,"enableInputShaping":false},"X":{"vel":200,"acc":1000,"stepsPerMm":100,"freq":55,"damp":0.1,"backlash":0.0,"shaperType":1,"pinStep":4,"pinDir":5,"hasEndstop":0},"Y":{"vel":60,"acc":200,"stepsPerMm":190,"freq":40,"damp":0.1,"backlash":0.0,"shaperType":1,"pinStep":12,"pinDir":13,"hasEndstop":0},"Z":{"vel":50,"acc":500,"stepsPerMm":80,"freq":50,"damp":0.1,"backlash":0.0,"shaperType":0,"pinStep":14,"pinDir":16,"hasEndstop":0},"tool":{"pinPwm":2,"pinToolOn":255,"pinZeroCross":22,"scale":100,"mode":0,"pwmMode":0,"useZeroCross":false,"invertToolOn":false},"homing":{"pinEndstop":16,"homingSpeed":100,"homingPullback":2},"system":{"broadcastPositionWhenJobRunning":false,"testLaserPower":255,"testLaserTime":1,"jogLimitEnabled":true},"cncInfo":{"name":"CNC Machine 1"},"lcd":{"sda":0,"sda2":5,"rst":0,"scl":4,"addr":60,"kind":4,"contrast":6},"karyacnc":{"ip":"","token":"","reportId":""}})EOF";
#endif

#endif // STRUCTURES_H
