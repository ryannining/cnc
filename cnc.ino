// =============================================================================
// CNC Controller untuk ESP8266 / ESP32
// Kontrol gerakan stepper motor dengan input shaping dan G-code parser
// =============================================================================

// =============================================================================
// SECTION 1: INCLUDES & PLATFORM DEFINITIONS
// =============================================================================

// Platform-specific includes
#include <cstdint>
#if defined(ESP8266)
#include <ArduinoOTA.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <LittleFS.h>
#include <WebSocketsServer.h>

#elif defined(ESP32)
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <LittleFS.h>
#include <WebServer.h>
#include <WebSockets.h>
#include <WebSocketsServer.h>
#include <WiFi.h>

#endif

// Common includes
#include "structures.h"
#include <ArduinoJson.h>
#include <time.h>

// ESP32 specific includes
#if defined(ESP32)
#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_struct.h"
#endif

// -----------------------------------------------------------------------------
// Platform-specific Macros
// -----------------------------------------------------------------------------

// ISR attribute untuk ESP8266/ESP32
#if defined(ESP8266)
#define ISR_ATTR ICACHE_RAM_ATTR
#elif defined(ESP32)
#define ISR_ATTR IRAM_ATTR
#endif

// Definisi Direct Port Access (Fast GPIO)
// ESP8266 memiliki register hardware untuk mengontrol GPIO secara langsung.
// Menggunakan register ini jauh lebih cepat daripada fungsi digitalWrite().
// GPOS = GPIO Set Register (Menulis 1 ke bit tertentu membuat pin HIGH)
// GPOC = GPIO Clear Register (Menulis 1 ke bit tertentu membuat pin LOW)
// ESP32 menggunakan register berbeda.
#if defined(ESP8266)
#define GPOSet(mask) GPOS = (mask)
#define GPOClear(mask) GPOC = (mask)
#elif defined(ESP32)
#define GPOSet(mask) GPIO.out_w1ts.val = (mask)
#define GPOClear(mask) GPIO.out_w1tc.val = (mask)
#endif

// =============================================================================
// COMMON MACROS (harus tersedia sebelum digunakan)
// =============================================================================
#define sqr(x) ((x) * (x))

// =============================================================================
// SECTION 2: FORWARD DECLARATIONS
// =============================================================================

// -----------------------------------------------------------------------------
// UI Functions (external)
// -----------------------------------------------------------------------------
void ui_loop();
void initUI();
// -----------------------------------------------------------------------------
// Motion Control Functions
// -----------------------------------------------------------------------------
void initShaper(AxisShaper *s, AxisConfig *cfg, int axis);
void addmove(float cf, float cx2, float cy2, float cz2, int toolPower, int g0,
             int rel, uint8_t jogFlags);
void prepareramp(int32_t bpos);
int stepgen();
void segmentloop();
void backforward(uint32_t nhead);
void planner(int32_t h);
inline int32_t startmove();
void motionloop();
void resetShapers();
void updateRealPos();
float getBufferedDistance();
void flushJogBuffer();

// -----------------------------------------------------------------------------
// Job Management Functions
// -----------------------------------------------------------------------------
void processJob();
void startJob(String filename);
void stopJob();
void toggleJobPause();
void remotePause();
void remoteResume();
void processPauseSequence();
void pauseParkReset();

// -----------------------------------------------------------------------------
// Manual Control Functions
// -----------------------------------------------------------------------------
void jogMulti(float distX, float distY, float distZ);
void applyZTrim(float deltaMm);
void resetZTrim();
void updateZTrimTiming();
extern void touchLcdActivity();
extern void requestLcdReset(); // reset LCD deferred via flag (ui.cpp)
extern volatile bool lcdResetPending;
extern void lcdForceFullRedraw(); // re-init controller LCD (ui.cpp)
void startHoming(int axis);
void processHoming();
void homeAxisFromUI(int axis);
void resetAll();

// -----------------------------------------------------------------------------
// Homing Helper Functions
// -----------------------------------------------------------------------------
bool isEndstopTriggered();
void setupEndstopPin();

// -----------------------------------------------------------------------------
// Tool Control Functions
// -----------------------------------------------------------------------------
void setToolPower(int power);
void testLaser();

// -----------------------------------------------------------------------------
// Configuration Functions
// -----------------------------------------------------------------------------
void loadAxisConfig(JsonObject &axis, int axisIdx);
void writeConfigJson();
bool setGlobalSpeed(int speed);
int getGlobalSpeed();
bool loadAllConfig();

// -----------------------------------------------------------------------------
// Network & Server Functions
// -----------------------------------------------------------------------------
bool connectToWiFi();
void setupRoutes();
void setupAxisPins();
void broadcastPositionHistory();
void recordPosition();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload,
                    size_t length);

// -----------------------------------------------------------------------------
// Math Helper Functions
// -----------------------------------------------------------------------------
int32_t isqrt(int32_t x);

// -----------------------------------------------------------------------------
// Report Functions
// -----------------------------------------------------------------------------
String urlEncode(const String &str);
void reportJobComplete(const char *jobName, bool success);

// -----------------------------------------------------------------------------
// G-Code Parser Functions (external)
// -----------------------------------------------------------------------------
extern void parseNextGCodeSegments();
extern void previewJob(String filename, bool perform);
extern bool openGxFile(String filepath);

// =============================================================================
// SECTION 3: GLOBAL VARIABLES (grouped by purpose)
// =============================================================================

// -----------------------------------------------------------------------------
// 3.1 Motion Control Variables
// -----------------------------------------------------------------------------

volatile uint32_t head = 0, tail = 0; // Indeks head dan tail buffer
tmove moves[NUMBUFFER];               // Array gerakan
volatile tmove *volatile m = 0;       // Pointer ke gerakan aktif
AxisShaper shapers[3];                // Shaper untuk X, Y, Z

// Global Shaper Buffer - Struct of Arrays (SOA) layout
// Layout: [tick_index][axis] untuk cache locality yang lebih baik
// Hanya X dan Y yang pakai shaper buffer, Z langsung step tanpa buffer
// Dialokasi malas di heap via syncShaperBuffer(), NULL saat shaping OFF
int16_t (*shaperBuffer)[2] = nullptr;

volatile uint32_t absTime = 0; // Waktu absolut dalam tick
uint32_t lastRecordTimeMs = 0;

// Step Event Buffer
volatile uint32_t sebhead = 0; // Indeks head step event buffer
volatile uint32_t sebtail = 0; // Indeks tail step event buffer
// sebUsed dihitung on-demand via makro seblength (hemat cycle CPU di ISR & stepgen)
volatile uint32_t stepEventBuffer[STEP_EVENT_BUFFER_SIZE]; // Buffer event step

// --- Sistem Antrian Token PWM (Kompresi RLE) ---
struct PwmToken {
  uint16_t
      targetPoppedSteps; // Penundaan berdasarkan Jarak Spasial Mutlak (16-bit)
  uint8_t pwmValue;
};

#define PWM_TOKEN_QUEUE_SIZE                                                   \
  64 // Optimized: testing shows max usage only 1-2 slots (0.4%), 64 is plenty
volatile PwmToken pwmTokenQueue[PWM_TOKEN_QUEUE_SIZE];
volatile uint8_t tokenHead = 0;
volatile uint8_t tokenTail = 0;
volatile uint16_t globalShaperPushedSteps =
    0; // Target Fractional Langkah (16-bit wraparound max 65535)
volatile uint16_t globalShaperPoppedSteps =
    0; // Hitungan pencapaian Langkah Mutlak (16-bit)

volatile uint8_t lastPushedPwm = 0;    // Buffer tracking memori Push
volatile uint8_t currentActivePwm = 0; // Memori status Laser Realtime
volatile bool shaper_enabled = false;

// Sinkronkan buffer dengan config. Dipanggil dari loadAllConfig() (boot saat
// ISR mati, maupun POST /api/config saat ISR hidup). Urutan publish menjaga
// invarian shaper_enabled==true berarti shaperBuffer!=NULL.
void syncShaperBuffer() {
  bool want = motionCfg.enableInputShaping;
  if (want && shaperBuffer)
    return;
  if (!want) {
    SYNC_START();
    shaper_enabled = false;
    int16_t (*old)[2] = shaperBuffer;
    shaperBuffer = nullptr;
    SYNC_END();
    if (old)
      free(old);
    return;
  }
  int16_t (*nb)[2] =
      (int16_t (*)[2])calloc(SHAPER_BUFFER_SIZE, sizeof(*shaperBuffer));
  if (!nb) {
    motionCfg.enableInputShaping = false;
    shaper_enabled = false;
    return;
  }
  SYNC_START();
  shaperBuffer = nb;
  shaper_enabled = true;
  SYNC_END();
}

// PWM Queue monitoring and safety
volatile bool pwmQueueOverflow = false;    // Flag: true saat queue penuh
volatile uint32_t pwmOverflowCount = 0;   // Counter: total overflow events
volatile uint8_t pwmMaxUsage = 0;         // Tracking: penggunaan maksimum buffer
#define PWM_QUEUE_WARNING_THRESHOLD 230 // Warning saat 90% penuh (230/256)

// Masker Bit untuk Fast GPIO (Direct Port Access)
uint32_t maskDir[3];
uint32_t maskToolPwm;
uint32_t maskToolOn;
uint32_t allStepMask = 0; // Masker gabungan seluruh pin STEP (X, Y, Z)

// -----------------------------------------------------------------------------
// 3.2 Position & Planning Variables
// -----------------------------------------------------------------------------
volatile float cx1, cy1, cz1; // Posisi G-code saat ini
float mmdis[NUMAXIS];         // Jarak dalam mm setiap axis
float curru[3];               // Vektor unit saat ini
float prevu[3];               // Vektor unit sebelumnya
tmove *pcurr, *pprev;         // Pointer gerakan saat ini dan sebelumnya

// Bresenham Variables
int32_t mcx[NUMAXIS];  // Counter step setiap axis
int32_t bsdx[NUMAXIS]; // Bresenham delta step
int8_t sx[NUMAXIS];    // Arah step setiap axis
uint32_t fastaxis;     // Axis dengan step terbanyak

// Ramp Variables
int32_t s2, s4, s6; // Panjang segmen ramp
int32_t acup, acdn; // Akselerasi naik dan turun
int32_t V;          // Kecepatan saat ini (scaled)
int32_t a2;         // Akselerasi squared
int32_t Sdest;      // Langkah tujuan segmen
int32_t sg;         // Status gerakan (0-4)

int32_t va, vi, vc, ve; // Kecepatan: actual, initial, cruise, end
int32_t lV;             // Kecepatan terakhir
int32_t realV;          // Kecepatan real

uint32_t stepdiv2;     // Divisor step
int32_t totalstep;     // Total step gerakan
int32_t mctr;          // Counter move
volatile bool isG0 = false;     // Flag G0
volatile bool isHoming = false; // Flag homing

// -----------------------------------------------------------------------------
// 3.3 Backlash Compensation Variables
// -----------------------------------------------------------------------------
int8_t lastMoveDir[3] = {
    0, 0, 0}; // Tracking arah terakhir setiap axis untuk backlash

// -----------------------------------------------------------------------------
// 3.4 Homing Variables
// -----------------------------------------------------------------------------
enum HomingState {
  HOMING_IDLE = 0,
  HOMING_SEARCH = 1,  // Mencari endstop
  HOMING_PULLBACK = 2 // Pullback setelah endstop terdeteksi
};
HomingState homingState = HOMING_IDLE;
int homingQueue[4] = {-1, -1, -1, -1}; // Queue axis untuk di-home
int homingQueueIdx = 0;                // Index queue saat ini
int homingCurrentAxis = -1;            // Axis yang sedang di-home
uint32_t homingTimeout = 0;            // Timeout untuk homing

// -----------------------------------------------------------------------------
// 3.5 Job Management Variables
// -----------------------------------------------------------------------------
extern File currentJobFile;
int progressPercent = 0;
extern size_t jobFileSize;
extern size_t jobBytesRead;
extern bool isParsingComplete;
extern bool isGxMode;
volatile bool jobRunning = false;
volatile bool jobPaused = false;
// State pause-with-park (definisi di dekat toggleJobPause)
extern volatile uint8_t pauseSeq;
extern volatile int32_t pausedRealStepZ;
extern volatile bool resumeWanted;
extern volatile int32_t parkTargetSteps;
extern volatile bool parkMoveDone;
extern volatile uint32_t parkTimer;
extern volatile uint32_t parkStepInterval;
extern volatile uint8_t savedPausePwm;
volatile bool shaperEmpty = false;
volatile bool stopping = false;
static uint32_t lastResumeMs = 0; // waktu resume terakhir (dipakai startJob)
#define PAUSE_COOLDOWN_MS                                                      \
  3000 // jeda minimal resume → pause berikutnya (anti spam toggle)
// Ramp pause/resume (time-based):
// - DECEL (mode 1, shaperISR): eventTime += (50×el)>>16, cap +255 tick —
//   mulai full speed tanpa lonjakan, melambat mulus; selesai saat buffer habis.
// - ACCEL (mode 2, stepgen): realV2 ×= mult Q8, mult = 16+((50×el)>>8)
//   → 0,06→1,0 dalam ~73ms. Pemicu: RESUME & freeze-recovery (via FREEZING).
volatile uint8_t isrRampMode = 0;
volatile uint32_t rampStartTicks =
    0; // absTime saat mode di-set (acuan elapsed)
// Lengan idle ISR: true = tick sunyi sebelumnya sudah simpan state diam,
// tick ini boleh pulang cepat. Dibuka (false) oleh tiap titik masuk energi.
volatile bool idleArmed = false;
volatile uint8_t isrRampWindowShift =
    16; // tidak terpakai (sisa eksperimen ramp, dibiarkan)

String currentJobFilename = "";
float G0Feed = 2000; // Feedrate default G0 (mm/min)
float G1Feed = 2000; // Feedrate default G1 (mm/min)

float JobX = 0;
// Z tertinggi yang pernah dicapai selama job — dipakai menu "Go to zero" /
// "Go to job start" sebagai ketinggian aman sebelum traverse XY.
float jobMaxZ = 0;
float JobY = 0;
float JobZ = 0;

// -----------------------------------------------------------------------------
// 3.5b Z-Trim Offset (pengganti kenop motor Z — dioperasikan remote saat
// cutting). Prinsip: target trim tersimpan di trimTargetSteps (step), lalu
// di-drain oleh shaperISR (trimAppliedSteps) dengan kecepatan tidak melebihi
// vel max Z, dan HANYA saat motor Z sedang diam (tiap event Z dari job
// me-reset zTrimTimer).
// -----------------------------------------------------------------------------
#define Z_TRIM_QUIET_TICKS 3000 // ±48ms quiet window setelah Z job berhenti
// Dua counter terpisah (tanpa read-modify-write lintas-konteks → aman di
// ESP8266 single-core & ESP32 dual-core):
// - trimTargetSteps : target trim total (step). Ditulis main, dibaca ISR.
// - trimAppliedSteps : step yang sudah diterapkan. HANYA ditulis ISR.
volatile int32_t trimTargetSteps = 0;
volatile int32_t trimAppliedSteps = 0;
volatile uint32_t zTrimTimer = 0; // counter drain (tick), di-reset event Z job
volatile uint32_t zTrimIntervalTicks = 1000; // jeda antar step trim (dari config)

// Job Timing
uint32_t jobStartTime = 0;    // Waktu job dimulai (millis)
uint32_t lastJobDuration = 0; // Durasi job terakhir yang selesai (ms)
time_t bootEpoch = 0;         // NTP epoch saat boot (GMT+7)
int tailok;
// int32_t lastf;

// -----------------------------------------------------------------------------
// 3.6 Tool Control Variables
// -----------------------------------------------------------------------------
volatile int pwmValue = 0;       // Nilai PWM saat ini (0-255)
volatile int pwmAccumulator = 0; // Accumulator untuk dither software PWM
volatile int currentSegmentPeakV =
    50; // Kecepatan referensi segment aktif (mm/min)
volatile uint32_t invCurrentSegmentPeakV =
    0; // Pre-calculated inverse peak kecepatan laser (16-bit shift)
int currentToolPower = 0; // Current tool power (S value dari G-code, 0-255)

volatile bool isTestLaser = false;
volatile uint32_t testingLaser = 0;

// -----------------------------------------------------------------------------
// Zero Cross Detection Variables
// -----------------------------------------------------------------------------
volatile bool zeroCrossDetected = false; // Flag zero-cross terdeteksi
volatile uint32_t zeroCrossTime =
    0;                          // Waktu terakhir zero-cross dalam absTime ticks
volatile uint32_t toolOnAt = 0; // Waktu (absTime) saat tool harus dinyalakan
volatile uint8_t zeroCrossTargetPower =
    0; // Target power untuk zero-cross mode (0-255)

// Auto-calculated half-cycle ticks menggunakan exponential moving average
volatile uint32_t zcHalfCycleTicks = 625; // Default 10ms @ 50Hz, auto-update
volatile uint32_t zcLastAbsTime = 0;      // Waktu ZC sebelumnya dalam absTime

// -----------------------------------------------------------------------------
// 3.6b Power-Fail Detection & State Save Variables
// -----------------------------------------------------------------------------
#define POWER_FAIL_THRESHOLD_MS 100       // Threshold 100ms tanpa ZC = mati PLN
#define POWER_FAIL_FILE "/powerfail.json" // File untuk simpan state
#define HEADPOS_FILE "/headpos.bin" // posisi head utk rescue setelah restart
#define HEADPOS_SAVE_MS 5000        // Interval simpan periodik posisi head
#define LCD_HEAL_QUIET_MS 3000 // heal LCD tunggu motor/spindle diam tenang
volatile bool powerFailDetected = false; // Flag power fail terdeteksi
volatile bool stateSaved = false;        // Flag state sudah disimpan
volatile bool forceStop = false;         // Flag emergency stop - ISR stop semua
int32_t lastZeroCrossTimeMs = 0;         // Waktu terakhir ZC dalam ms

// Resume state structure - hanya untuk simpan posisi mesin saat mati PLN
struct ResumeState {
  int32_t realStepX, realStepY, realStepZ; // Step count untuk setiap axis
  float jobX, jobY, jobZ;                  // Posisi awal job (mm)
  bool valid;                              // Flag state valid
};
ResumeState resumeState = {0, 0, 0, 0.0f, 0.0f, 0.0f, false};
bool needResume = false; // Flag menandakan perlu resume posisi

// -----------------------------------------------------------------------------
// 3.7 Jog Control Variables
// -----------------------------------------------------------------------------
uint32_t lastJogTime = 0; // Waktu terakhir jog (millis)
volatile uint32_t bufferFillStartTime = 0;
#define JOG_FLUSH_TIMEOUT                                                      \
  2000 // Hapus buffer jog setelah 2 detik tidak ada aktivitas

// -----------------------------------------------------------------------------
// 3.8 Network Variables
// -----------------------------------------------------------------------------
#if defined(ESP8266)
ESP8266WebServer server(80);
WebSocketsServer webSocket(81);
#elif defined(ESP32)
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
#endif

String wifiSSID = "usaha karya";
String wifiPassword = "45712319";
bool wifiConnected = false;
bool isAccessPoint = false;
const char *apSSID = nullptr;
const char *apPassword = "";

String staticIPPrefix = "192.168.1";
int staticIPLastOctet = 45;
bool useStaticIP = true;
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 1, 1);

char cncName[32] = "CNC Machine 1";
char cncIP[16] = "";

uint32_t lastPositionBroadcast = 0;
volatile bool pendingPositionBroadcast = false;

// Karyacnc Configuration
String karyacncIP = "";
String karyacncToken = "";
String karyacncReportId = "";

#if defined(ESP8266)
WiFiClient karyacncClient;
#elif defined(ESP32)
WiFiClient karyacncClient;
#endif

// -----------------------------------------------------------------------------
// 3.9 Configuration Variables
// -----------------------------------------------------------------------------
AxisConfig cfg[3] = {
    {200.0, 1000.0, 100.0, 55.0, 0.1, 0.0, 1, D6, D5, 0}, // X axis
    {60.0, 200.0, 190.0, 40.0, 0.1, 0.0, 1, D4, D7, 0},   // Y axis
    {50.0, 500.0, 80.0, 50.0, 0.1, 0.0, 0, D3, D0, 0}     // Z axis
};

MotionConfig motionCfg = {0.5, 10.0, false};
ToolConfig toolCfg = {
    D8, 255,   22,   100, 1,
    0,  false, false}; // pinPwm, pinToolOn, pinZeroCross, scale, mode, pwmMode,
                       // useZeroCross, invertToolOn
HomingConfig homingCfg = {D0, 100.0, 2.0};
SystemConfig systemCfg = {false, 255,
                            1, true}; // broadcast, testLaserPower, testLaserTime, jogLimitEnabled

volatile int globalSpeed = 16;
int fixed_max_multiplier = 16;

// Kapasitas parse config.json — dialokasi di heap per pemakaian agar tak
// menetap 2 KB di BSS (lihat loadAllConfig).
const size_t CONFIG_DOC_SIZE = 2048;

// -----------------------------------------------------------------------------
// 3.10 Pin Configuration
// -----------------------------------------------------------------------------
#ifdef ESP8266
int pinStep[3] = {D2, D6, D5};
int pinDir[3] = {D1, D7, D4};
#elif defined(ESP32)
int pinStep[3] = {IO15, IO2, IO0};
int pinDir[3] = {IO4, IO16, IO17};
#endif

// -----------------------------------------------------------------------------
// 3.11 Position History Variables
// -----------------------------------------------------------------------------
#define POSITION_HISTORY_SIZE 15
PositionRecord positionHistory[POSITION_HISTORY_SIZE];
int positionHistoryIndex = 0;

// -----------------------------------------------------------------------------
// 3.12 File Upload Variables
// -----------------------------------------------------------------------------
File fileU;

// -----------------------------------------------------------------------------
// 3.13 System Variables
// -----------------------------------------------------------------------------
const char *cpuType = CPU_TYPE;
bool canUseSerial = false; // true hanya jika TX/RX tidak dipakai perangkat lain

#if defined(ESP32)
portMUX_TYPE cncMux = portMUX_INITIALIZER_UNLOCKED;
#endif

// -----------------------------------------------------------------------------
// 3.14 Math Helper Variables
// -----------------------------------------------------------------------------
int mpower = 0;

// -----------------------------------------------------------------------------
// 3.15 LCD & Remote Variables (external - defined in ui.cpp)
// -----------------------------------------------------------------------------
extern int lcd_rst;
extern int lcd_sda;
extern int lcd_sda2;
extern int lcd_contrast;
extern int lcd_scl;
extern int lcd_addr;
extern int lcd_kind;
extern int lcd_cs;

// -----------------------------------------------------------------------------
// 3.16 ISR Helper Variables
// -----------------------------------------------------------------------------
volatile uint8_t tPwm = 0;
volatile uint32_t lastEventTime = 0;
volatile uint32_t avgCycle = 0;

// =============================================================================
// SECTION 4: LOW-LEVEL MOTION (ISR)
// =============================================================================

// -----------------------------------------------------------------------------
// Push Step ke Shaper Buffer
// -----------------------------------------------------------------------------
// Alih-alih menggerakkan motor langsung, kita "mendorong" step ke masa depan
// dalam bentuk beberapa impulse.
// Contoh: ZV shaper memecah 1 step menjadi:
// - 56% segera
// - 44% delay sekian milidetik (sesuai frekuensi resonansi mesin)
#define addContrib(x) s->accumulator += x;
// signedAccum[1] = akumulasi absolut impulse (direct + buffer drain)
// ghost energy (cancel-out) dikreditkan ke signedAccum[0] saat pushStep
#define addContribSign(x) s->signedAccum[1] += (int16_t)((x) < 0 ? -(x) : (x));

// -----------------------------------------------------------------------------
// pushStep — Kurangi branch, cache pointer
// -----------------------------------------------------------------------------
ICACHE_RAM_ATTR void pushStep(int axis, int direction, uint8_t pwm) {
  AxisShaper *const s = &shapers[axis];

  if (LIKELY(axis < 2)) {
    // Waktu eksekusi PWM ini ditunda berdasarkan antrean langkah aktual absolut
    // motor
    globalShaperPushedSteps++;

    // Kompresi (RLE): Hanya push jika NILAI PWM BERBEDA dengan sebelumnya!
    if (UNLIKELY(pwm != lastPushedPwm)) {
      uint8_t nextHead = (tokenHead + 1) & (PWM_TOKEN_QUEUE_SIZE - 1);
      if (nextHead != tokenTail) { // Pastikan queue tidak full
        pwmTokenQueue[tokenHead].targetPoppedSteps = globalShaperPushedSteps;
        pwmTokenQueue[tokenHead].pwmValue = pwm;
        tokenHead = nextHead;
        lastPushedPwm = pwm;

        // Track max usage - lightweight check every 16 pushes to reduce ISR
        // overhead
        if ((globalShaperPushedSteps & 0x0F) == 0) { // Check every 16 steps
          uint8_t currentUsage =
              (tokenHead - tokenTail) & (PWM_TOKEN_QUEUE_SIZE - 1);
          if (currentUsage > pwmMaxUsage) {
            pwmMaxUsage = currentUsage;
          }
        }
      } else {
        // MONITORING ONLY: Queue penuh - catat untuk analisis
        // Tidak mengubah behavior laser karena belum terjadi masalah di mesin
        // riil
        pwmQueueOverflow = true;
        pwmOverflowCount++;
      }
    }
  }

  if (!shaper_enabled || (axis == 2)) {
    // Direct step — tanpa shaper
    int c = direction * SHAPER_SCALE;
    addContrib(c);
    if (LIKELY(axis < 2))
      addContribSign(c);

    return;
  }

  // === Shaped path (X/Y only) ===
  const int8_t numImp = s->numImpulses;
  const Impulse *const imp = s->impulses;
  const uint32_t baseTime = absTime;

// Semua impulse (termasuk imp[0] delay=0) masuk ke shaperBuffer
// agar ghost energy detection seragam untuk semua impulse.
// Impulse 0 ditulis ke slot baseTime → langsung dibaca di tick ini
// karena urutan ISR: pushStep() dipanggil sebelum processAxisMoveInline().
//
// Deteksi cancel-out: ghost = |oldVal| + |c| - |newVal|
// #define bufferContrib(c)                                                       \
//   {                                                                            \
//     int16_t _old = shaperBuffer[idx][axis];                                    \
//     int16_t _new = _old + (int16_t)(c);                                        \
//     shaperBuffer[idx][axis] = _new;                                            \
//     /* ghost = energi yang musnah akibat cancel-out */                         \
//     int16_t _absOld = (_old < 0) ? -_old : _old;                               \
//     int16_t _absC = ((c) < 0) ? -(c) : (c);                                    \
//     int16_t _absNew = (_new < 0) ? -_new : _new;                               \
//     int16_t _ghost = _absOld + _absC - _absNew;                                \
//     if (_ghost > 0)                                                            \
//       s->signedAccum[0] += _ghost;                                             \
//   }
// (c) Early-out ghost energy — cancel hanya bila tanda berlawanan:
#define bufferContrib(c) {                                                    \
  int16_t _old = shaperBuffer[idx][axis];                                     \
  int16_t _new = _old + (int16_t)(c);                                         \
  shaperBuffer[idx][axis] = _new;                                             \
  if ((_old ^ (c)) < 0) {            /* tanda berlawanan → baru cek abs (kasus jarang)*/ \
    int16_t _ghost = ((_old<0)?-_old:_old) + (((c)<0)?-(c):(c))             \
                   - ((_new<0)?-_new:_new);                                   \
    if (_ghost > 0) s->signedAccum[0] += _ghost;                            \
  }                                                                           \
}
  for (int i = 0; i < numImp; i++) {
    int c = imp[i].weight;
    if (LIKELY(c != 0)) {
      c *= direction;
      uint32_t idx = (baseTime + imp[i].delay) & (SHAPER_BUFFER_SIZE - 1);
      bufferContrib(c);
    }
  }
}

// -----------------------------------------------------------------------------
// Helper inline untuk menggerakkan axis di ISR
// -----------------------------------------------------------------------------
// Signature harus satu baris — prototipe otomatis Arduino kehilangan
// 'static' bila nama fungsi terpisah dari return type.
ICACHE_RAM_ATTR static inline uint32_t processAxisMoveInline(int axisIdx, uint32_t labsTime) {
  AxisShaper *const ss = &shapers[axisIdx];

  // 1. Drain shaper buffer → accumulator + signedAccum
  if (LIKELY(shaper_enabled & (axisIdx < 2))) {
    uint32_t idx = labsTime & (SHAPER_BUFFER_SIZE - 1);
    int16_t val = shaperBuffer[idx][axisIdx];

    if (UNLIKELY(val != 0)) {
      shaperBuffer[idx][axisIdx] = 0;

      // Net value ke accumulator (cancel-out sudah terjadi di buffer)
      ss->accumulator += (int32_t)val;

      // Kredit absolut net value ke signedAccum[1] untuk laser sync
      // (ghost energy untuk bagian yang cancel sudah dikreditkan di pushStep)
      int16_t absVal = (val < 0) ? -val : val;
      ss->signedAccum[1] += absVal;
    }
  }

  // Pop laser events — signedAccum tracks total impulses resolved
  // Unrolled: biasanya hanya 0-1 pop per tick
  // Tapi bisa >1 jika multiple impulses resolve di tick yang sama
  for (int i = 0; i < 2; i++) {
    // FIX: tanpa volatile (lihat structures.h signedAccum) — hemat
    // 2 forced memory-load per axis per tick; konteks akses tetap ISR yang sama
    int16_t *sa = &ss->signedAccum[i];
    if (UNLIKELY(*sa >= SHAPER_SCALE)) {
      *sa -= SHAPER_SCALE;
      if (axisIdx < 2)
        globalShaperPoppedSteps++;
    }
  }

  // 2. Accumulator → motor step (error diffusion dithering)
  int32_t acc;
  acc = ss->accumulator;

  // Slow path: step needed
  int8_t dir = 0;
  if (UNLIKELY(acc >= SHAPER_SCALE)) {
    ss->accumulator = acc - SHAPER_SCALE;
    ss->realStep++;
    dir = 1;
  } else if (UNLIKELY(acc <= -SHAPER_SCALE)) {
    ss->accumulator = acc + SHAPER_SCALE;
    ss->realStep--;
    dir = -1;
  }
  if (LIKELY(dir == 0))
    return 0;
  // Set direction — hanya jika berubah
  if (UNLIKELY(dir != ss->lastDir)) {
    ss->lastDir = dir;
    if (UNLIKELY(pinDir[axisIdx] >= 16)) {
      digitalWrite(pinDir[axisIdx], dir > 0 ? HIGH : LOW);
    } else {
      if (dir > 0)
        GPOSet(maskDir[axisIdx]);
      else
        GPOClear(maskDir[axisIdx]);
    }
  }

  return ss->stepMask;
}
extern volatile bool isPreviewMode;

// -----------------------------------------------------------------------------
// Zero Cross Detection ISR
// -----------------------------------------------------------------------------
ISR_ATTR void zeroCrossISR() {
  if (!toolCfg.useZeroCross)
    return;

  zeroCrossDetected = true;
  lastZeroCrossTimeMs = millis(); // Catat waktu ZC untuk deteksi mati PLN

  // Update exponential moving average dari half-cycle ticks
  if (zcLastAbsTime > 0) {
    uint32_t delta = absTime - zcLastAbsTime;
    // Exponential moving average: avg = (avg + new) / 2
    zcHalfCycleTicks = (zcHalfCycleTicks + delta) >> 1;
  }
  zcLastAbsTime = absTime;

  // Jika mode zero-cross PWM aktif
  if (toolCfg.pwmMode == 2) {
    // Matikan tool saat zero-cross terdeteksi
    GPOClear(maskToolPwm);

    // Hitung kapan tool harus dinyalakan kembali
    // Power 255 = nyala segera (delay 0)
    // Power 0 = tidak pernah nyala dalam setengah siklus ini
    if (zeroCrossTargetPower > 0) {
      // Delay proportional dengan (255 - power)
      // Semakin kecil power, semakin lama delay
      uint32_t delayTicks = ((255 - zeroCrossTargetPower) * zcHalfCycleTicks) >>
                            8; // div 255 -> >> 8 approx
      if (delayTicks == 0)
        delayTicks = 1; // minimal delay 1 tick

      // Catat waktu nyala berikutnya dalam absTime ticks
      toolOnAt = absTime + delayTicks;
    } else {
      // Power = 0, tidak perlu nyala
      toolOnAt = 0;
    }
  }
}

// -----------------------------------------------------------------------------
// ISR Timer untuk input shaper
// -----------------------------------------------------------------------------
// uint32_t lastEventOff = 0; // tidak dipakai; dipertahankan utk kompatibilitas
volatile bool FREEZING = false; // penanda starvation mesin (recovery ramp-nya
// nonaktif — sumber starvation utama = WiFi broadcast saat job, kini OFF di
// config; anti-burn tidak tergantung flag ini, pakai sebempty langsung)
ISR_ATTR void shaperISR() {
  GPOClear(allStepMask); // matikan semua pin STEP (pulsa = lebar 1 tick)

  uint32_t startCycles = 0;
#if defined(ESP32)
  startCycles = ESP.getCycleCount();
#endif

  uint32_t labsTime = ++absTime;

  // ----- Emergency Stop (Force Stop) -----
  // Jika forceStop flag di-set, bersihkan semua buffer dan hentikan aktivitas
  if (UNLIKELY(forceStop)) {
    // Bersihkan semua buffer
    sebhead = sebtail; // Kosongkan step event buffer
    tokenHead = tokenTail = 0;
    // Berhentikan seketika trim Z yang masih terantre
    trimTargetSteps = 0;
    trimAppliedSteps = 0;
    zTrimTimer = 0;
    // Batalkan park/restore (pause)
    pauseSeq = 0;
    parkTargetSteps = 0;
    parkMoveDone = false;
    parkTimer = 0;
    isrRampMode = 0; // emergency stop tanpa ramp
    globalShaperPushedSteps = globalShaperPoppedSteps = 0;
    lastPushedPwm = currentActivePwm = 0;
    head = tail; // Kosongkan motion buffer

    // Reset shaper accumulators
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

    return; // Keluar dari ISR
  }

  // ----- Step Event Consumption -----
  if (LIKELY(!sebempty)) {
    // Cache buffer read
    uint32_t lsebtail, event, eventTime;
    lsebtail = sebtail;
    event = stepEventBuffer[lsebtail];
    eventTime = GET_EVENT_TIME(event);

    // DECEL (mode 1): eventTime += addt, addt = (50×el)>>16 cap +255 tick —
    // mulai full speed (addt=0) lalu melambat mulus tanpa lonjakan. Field
    // eventTime 16-bit, nilai normal ≪ 65535 → +255 aman. ACCEL tidak ada di
    // ISR — resume selalu mulai dengan buffer kosong; accel milik stepgen.
    if (UNLIKELY(isrRampMode == 1)) {
      uint32_t el = labsTime - rampStartTicks;
      uint32_t addt = ((50 * el) >> 16); 
      if (addt>255) addt=255;
      eventTime = eventTime + addt;
    }

    if (UNLIKELY((labsTime - lastEventTime) >= eventTime)) {
      // Carry sisa overshoot agar jadwal tak drift; telat kebangetan (>4x
      // interval, mis. habis freeze) → sinkron ulang ke jam sekarang.
      uint32_t behind = labsTime - lastEventTime;
      lastEventTime += eventTime;
      if (UNLIKELY(behind > (eventTime << 2) + 8))
        lastEventTime = labsTime;
      sebtail = nextseb(lsebtail);

      uint8_t pwm = GET_EVENT_PWM(event);

      // Decode step bits — branchless extraction
      // Setiap axis: cek step bit, extract direction, call pushStep
      uint32_t ev = event; // Local cache

      if (ev & (1 << STEP_EVENT_X_STEP_BIT)) {
        pushStep(0, (ev & (1 << STEP_EVENT_X_DIR_BIT)) ? 1 : -1, pwm);
      }
      if (ev & (1 << STEP_EVENT_Y_STEP_BIT)) {
        pushStep(1, (ev & (1 << STEP_EVENT_Y_DIR_BIT)) ? 1 : -1, pwm);
      }
      if (ev & (1 << STEP_EVENT_Z_STEP_BIT)) {
        pushStep(2, (ev & (1 << STEP_EVENT_Z_DIR_BIT)) ? 1 : -1, pwm);
        // Z job sedang bergerak → tunda drain trim sampai Z diam.
        zTrimTimer = zTrimIntervalTicks + Z_TRIM_QUIET_TICKS;
      }
    }
  } else {
      if (UNLIKELY(isrRampMode == 1)) {
        isrRampMode = 0; // decel selesai: buffer habis
      }
  }

  uint32_t stepMask = processAxisMoveInline(0, labsTime);
  stepMask |= processAxisMoveInline(1, labsTime);
  stepMask |= processAxisMoveInline(2, labsTime);

  // ----- Z-Trim drain: 1 step Z per interval, hanya saat Z job diam -----
  // Arah mengikuti mekanisme standar sistem: pushStep(2,...) →
  // processAxisMoveInline(2) menulis pin DIR sesuai lastDir & config pin axis.
  // Tidak ada logika arah baru di sini.
  if (UNLIKELY(trimTargetSteps != trimAppliedSteps)) {
    if (zTrimTimer > 0) {
      zTrimTimer--;
    } else {
      zTrimTimer = zTrimIntervalTicks;
      int8_t dir = trimTargetSteps > trimAppliedSteps ? 1 : -1;
      pushStep(2, dir, 0);
      trimAppliedSteps += dir;
    }
  }

  // ----- Park drain (pause): gerakkan Z menuju parkTargetSteps (Z0 saat
  // pause, Z tersimpan saat restore). Arah mengikuti mekanisme standar,
  // kecepatan dibatasi interval yang sama dengan trim.
  if (UNLIKELY(pauseSeq == 2 || pauseSeq == 4)) {
    if (!parkMoveDone) {
      if (parkTimer > 0) {
        parkTimer--;
      } else {
        parkTimer = parkStepInterval;
        int32_t diff = parkTargetSteps - shapers[2].realStep;
        if (diff == 0) {
          parkMoveDone = true;
        } else {
          pushStep(2, diff > 0 ? 1 : -1, 0);
        }
      }
    }
  }

  // Fast-path idle: dua tick sunyi beruntun → lewati token/PWM/dither yang
  // hasilnya konstan. Dibuka oleh tiap titik masuk energi (event, ramp,
  // test-laser, forceStop/FREEZING). Pin STEP + PWM tetap ditulis (2 register
  // write) agar glitch ESD terkoreksi tiap tick walau idle sejam.
  if (UNLIKELY(sebempty && shaperEmpty && !isTestLaser && isrRampMode == 0 &&
               !FREEZING && !forceStop &&
               trimTargetSteps == trimAppliedSteps &&
               !(pauseSeq == 2 || pauseSeq == 4))) {
    if (idleArmed) {
      if ((uint32_t)(labsTime - lastEventTime) > 15000)
        FREEZING = true;
      GPOClear(maskToolPwm);
      GPOSet(stepMask);
      return;
    }
    idleArmed = true;
  } else {
    idleArmed = false;
  }

  if (LIKELY(jobRunning || isPreviewMode)) {
    // Sinkronisasi sinkron: Apakah jumlah step riil motor sudah
    // mencapai/melewati titik tiket? Pakai (int16_t) supaya aman walau counter
    // uint16 menabrak overflow ~65 ribu steps!
    while (tokenHead != tokenTail &&
           (int16_t)(globalShaperPoppedSteps -
                     pwmTokenQueue[tokenTail].targetPoppedSteps) >= 0) {

      currentActivePwm = pwmTokenQueue[tokenTail].pwmValue;
      tokenTail = (tokenTail + 1) & (PWM_TOKEN_QUEUE_SIZE - 1);
      pwmValue = currentActivePwm; // tulis hanya saat token pop (bukan tiap tick)
    }
  }

  // TOOLON pin: HIGH saat PWM > 0 — tulis GPIO hanya saat state berubah
  if (toolCfg.pinToolOn < 255) {
    static bool lastToolOnState = false;
    bool shouldBeOn = (pwmValue > 0);
    if (toolCfg.invertToolOn)
      shouldBeOn = !shouldBeOn;
    if (UNLIKELY(shouldBeOn != lastToolOnState)) {
      lastToolOnState = shouldBeOn;
      if (shouldBeOn) {
        GPOSet(maskToolOn);
      } else {
        GPOClear(maskToolOn);
      }
    }
  }
  if (UNLIKELY(toolCfg.mode == 1 && isTestLaser)) {
    // Gunakan selisih untuk handle overflow uint32_t dengan aman
    if ((int32_t)(labsTime - testingLaser) >= 0) {
      isTestLaser = false;
      pwmValue = 0;
    } else {
      pwmValue = systemCfg.testLaserPower;
    }
  }

  // Cache skala PWM — recompute hanya saat pwmValue/scale berubah.
  // Override preventBurn di bawah hanya menyentuh salinan lokal, cache tetap
  // valid.
  static int lastPwmValue = -1;
  static int lastPwmScale = -1;
  static int apwmCached = 0;
  if (pwmValue != lastPwmValue || toolCfg.scale != lastPwmScale) {
    lastPwmValue = pwmValue;
    lastPwmScale = toolCfg.scale;
    apwmCached = pwmValue < 253 ? (pwmValue * toolCfg.scale) >> 10 : 255;
  }
  int apwmValue = apwmCached;

  // Kosong yang benar: tak ada event + tak ada sisa accumulator + tak ada
  // energi antre (ghost maupun impulse). Impulse di shaperBuffer delay s/d
  // 3*td (~27 ms di 55 Hz) — abaikan ini = ekor getaran terpotong + laser
  // mati dini di processJob.
  // bool accZero = ((shapers[0].accumulator | shapers[1].accumulator |
  //                  shapers[2].accumulator) == 0);
  // // bool nrgZero = ((shapers[0].signedAccum[0] | shapers[0].signedAccum[1] |
  // //                  shapers[1].signedAccum[0] | shapers[1].signedAccum[1]) == 0);
  // // // shaperEmpty = accZero && nrgZero && sebempty;
  shaperEmpty = ((shapers[0].accumulator | shapers[1].accumulator |
                  shapers[2].accumulator) == 0);
  // SAFETY: Anti-burn laser — jika mesin berhenti total karena data kelaparan
  // (sebempty) DAN getaran motor sudah tuntas (shaperEmpty), paksa laser mati.
  bool preventBurn = sebempty;
  // Starvation ~240ms (15000 tick × 16us) tanpa event = mesin nyangkut; tandai FREEZING agar
  // stepgen menjalankan recovery ramp saat gerak dilanjutkan.
  if (!FREEZING && sebempty && (uint32_t)(absTime - lastEventTime) > 15000) {
    FREEZING = true;
  }
  // Selama pause (parkir) PWM dipaksa mati. Saat fase restore (seq 4)
  // diizinkan — spindle sudah dinyalakan sebelum turun menuju kedalaman.
  bool restoring = (pauseSeq == 4);
  if (jobRunning && ((jobPaused && !restoring) || (preventBurn && !restoring)) &&
      shaperEmpty)
    apwmValue = 0;

  // PWM Output based on selected mode
  if (LIKELY(toolCfg.pwmMode == 1)) {
    // PWM Duty Cycle Mode - lower frequency, better for some laser PSUs
    static volatile uint8_t pwmCounter = 0;
    pwmCounter++;
    if (apwmValue == 0) {
      GPOClear(maskToolPwm);
    } else if (apwmValue >= 255) {
      GPOSet(maskToolPwm);
    } else {
      if (pwmCounter < apwmValue) {
        GPOSet(maskToolPwm);
      } else {
        GPOClear(maskToolPwm);
      }
    }
  } else if (toolCfg.pwmMode == 2) {
    // Zero Cross PWM Mode - untuk kontrol AC dengan zero-cross detection
    // Update target power untuk zeroCrossISR
    zeroCrossTargetPower = apwmValue;

    if (apwmValue == 0) {
      GPOClear(maskToolPwm);
    } else if (apwmValue >= 255) {
      GPOSet(maskToolPwm);
    } else {
      // Cek apakah sudah waktunya nyalakan tool
      // zeroCrossISR akan mematikan tool dan set toolOnAt saat ZC terdeteksi
      // Di sini kita nyalakan tool saat labsTime >= toolOnAt
      if (toolOnAt > 0 && (int32_t)(labsTime - toolOnAt) >= 0) {
        GPOSet(maskToolPwm);
        toolOnAt = 0; // Reset setelah nyala
      }
    }
  } else {
    // Dither Bresenham (Pulse Density Modulation) - default, high frequency
    // Tambah nilai penuh: power kecil pun proporsional (mis. 1 → nyala 1x
    // tiap 255 tick), bukan nol selamanya seperti apwmValue/4.
    pwmAccumulator += apwmValue;
    if (pwmAccumulator >= 255) {
      pwmAccumulator -= 255;
      GPOSet(maskToolPwm);
    } else {
      GPOClear(maskToolPwm);
    }
  }
  GPOSet(stepMask);

#if defined(ESP32)
  avgCycle = (avgCycle * 31 + (ESP.getCycleCount() - startCycles)) >> 5;
#endif
}

// =============================================================================
// SECTION 5: MOTION PLANNER
// =============================================================================

// -----------------------------------------------------------------------------
// Helper inline untuk generate step event berdasarkan axis
// -----------------------------------------------------------------------------
// Precomputed — set sekali di setup atau compile time
static const uint32_t AXIS_STEP_BIT[3] = {(1 << STEP_EVENT_X_STEP_BIT),
                                          (1 << STEP_EVENT_Y_STEP_BIT),
                                          (1 << STEP_EVENT_Z_STEP_BIT)};
static const uint32_t AXIS_DIR_BIT[3] = {(1 << STEP_EVENT_X_DIR_BIT),
                                         (1 << STEP_EVENT_Y_DIR_BIT),
                                         (1 << STEP_EVENT_Z_DIR_BIT)};

inline void processAxisStep(int ix, uint32_t &event, int32_t totalstep) {
  if ((mcx[ix] -= bsdx[ix]) < 0) {
    mcx[ix] += totalstep;
    event |= AXIS_STEP_BIT[ix];
    if (sx[ix] > 0)
      event |= AXIS_DIR_BIT[ix];
    // if (ix == 2) Serial.print("Z");
  }
}
// =============================================================================
// SECTION 11: MATHEMATICS HELPERS
// =============================================================================
// Note: Macro 'sqr' sudah didefinisikan di SECTION 1

// -----------------------------------------------------------------------------
// Hitung akar kuadrat integer
// -----------------------------------------------------------------------------
// int32_t isqrt(int32_t x) {
//   if (x < 64 * 64)
//     return 64;
//   int32_t q = 1, r = 0;
//   while ((q <<= 2) <= x) {
//   }
//   while (q >>= 2) {
//     int32_t t;
//     t = x - r - q;
//     r >>= 1;
//     if (t >= 0) {
//       x = t;
//       r += q;
//     }
//   }
//   return r;
// }

  int32_t isqrt(int32_t x) {
      if (x < 4096) return 64;
      uint32_t msb = 31 - __builtin_clz(x);
      uint32_t q = 1UL << ((msb >> 1) << 1);
      uint32_t r = 0;
      while (q != 0) {
        uint32_t t = x - r - q;
        r >>= 1;
        if ((int32_t)t >= 0) { x = t; r += q; }
        q >>= 2;
      }
      return r;
    }

  //   inline uint32_t isqrt_hybrid(uint32_t V, uint32_t prev) {
  //     if (V < 4096) return 64;
  //     uint32_t x = prev;
  //     // if (x < 64) x = 64;
  //     // if (x > 46340) x = 46340;
  //     uint8_t n = 0;
  //     while ((x + 1) * (x + 1) <= V) {
  //       x++;
  //       if (++n >= 8) return isqrt_clz(V);
  //     }
  //     n = 0;
  //     while (x * x > V) {
  //       x--;
  //       if (++n >= 8) return isqrt_clz(V);
  //     }
  //     return x;
  //   }
    uint32_t isqrt_incr(uint32_t V, uint32_t prev) {
      if (V < 4096) return 64;
      uint32_t x = prev;
      // if (x < 64) x = 64;
      // if (x > 46340) x = 46340;
      if ((x + 1) * (x + 1) <= V) {
        do { x++; } while ((x + 1) * (x + 1) <= V);
      } else {
        while (x * x > V) x--;
      }
      return x;
    }

// -----------------------------------------------------------------------------
// Generator step
// -----------------------------------------------------------------------------
static uint32_t lastError = 0;

inline int stepgen() {
  if (sg != 2 || lV != V) {
    lV = V;
    V += a2;
    realV = isqrt_incr((uint32_t)V, (uint32_t)realV);
  }
  int32_t realV2=realV;
  // ACCEL (mode 2): realV2 ×= mult Q8 — 0,06→1,0 dalam ~73ms. Yang diskala
  // VELOCITY → timing event & kompensasi laser (B1/B4 memakai realV2) ikut
  // ramp → tanpa overburn. Ini satu-satunya blok accel — jangan duplikasi di
  // ISR (buffer resume selalu kosong; duplikasi = double-scaling V²).
  if (FREEZING && jobRunning) {
    // hanya jika sedang menjalankan job kita perlu ramp up
    isrRampMode = 2;
    rampStartTicks=lastEventTime=absTime;
    FREEZING=false;
  }
  if (UNLIKELY(isrRampMode == 2)) {
      uint32_t el = absTime - rampStartTicks;
      int32_t mult = 16+((50 * el) >> 8); 
    if (mult >=256) { 
      isrRampMode = 0; // ramp tuntas (mult ≥ 1,0) → kembali normal penuh
    } 
    else realV2 = (realV2 * mult) /256;
  }

  // B1: cache pembagian — div software ESP8266 ~40 siklus. Selama cruise
  // (sg==2) realV2 & stepdiv2 konstan → div hanya saat berubah.
  // Hemat ~10% CPU pada step rate tinggi. Semantik hasil identik.
  // Resolusi 1/256 tick: pembulatan, bukan truncasi — sisa bagi tak hilang.
  // stepdiv2 berskala Q8 (lihat startmove) → geser 6 agar hasil akhir sama.
  static uint32_t cacheRealV = 0, cacheStepdiv2 = 0, cacheTm = 0;
  if (realV2 != cacheRealV || stepdiv2 != cacheStepdiv2) {
    cacheRealV = realV2;
    cacheStepdiv2 = stepdiv2;
    cacheTm = (stepdiv2 << 6) / realV2;
  }
  uint32_t tm = cacheTm + lastError;
  lastError = tm & 255;
  uint32_t event = 0;
  // Unrolled loop untuk 3 axis - lebih cepat tanpa overhead loop
  processAxisStep(0, event, totalstep);
  processAxisStep(1, event, totalstep);
  processAxisStep(2, event, totalstep);

  // Clamp 16-bit: gerak sangat pelan bisa > 65535 tick → tanpa clamp wrap
  // menjadi step jauh lebih dini.
  uint32_t et = tm >> 8;
  if (et > 65535)
    et = 65535;
  event |= SET_EVENT_TIME(et);

  // Laser PWM event — cache B4: recompute hanya jika salah satu input berubah;
  // currentSegmentPeakV masuk key karena menentukan jalur & hasil kompensasi.
  static int cLaserMode = -1, cIsG0 = -1, cMpower = -1, cPeak = -1;
  static uint32_t cRealV = 0;
  static int laserPwmCached = 0;
  if (toolCfg.mode != cLaserMode || isG0 != cIsG0 || mpower != cMpower ||
      realV2 != cRealV || currentSegmentPeakV != cPeak) {
    cLaserMode = toolCfg.mode;
    cIsG0 = isG0;
    cMpower = mpower;
    cRealV = realV2;
    cPeak = currentSegmentPeakV;
    int lp = 0;
    if (toolCfg.mode == 0) {
      // Router mode: kirim PWM power (0-255)
      lp = mpower;
    } else if (!isG0 && mpower > 0) {
      // Laser mode: kompensasi power berdasarkan kecepatan
      if (realV2 >= (uint32_t)currentSegmentPeakV) {
        lp = mpower;
      } else {
        // Fixed-point math — hindari pembagian per step yang lambat
        lp = (mpower * realV2 * invCurrentSegmentPeakV) >> 16;
      }
      if (lp < 1)
        lp = 0;
    }
    laserPwmCached = lp;
  }
  event |= SET_EVENT_PWM(laserPwmCached);

  // Check buffer full before writing (with interrupts disabled for thread
  // safety)
  stepEventBuffer[sebhead] = event;
  sebhead = nextseb(sebhead);

  return --Sdest > 0;
}

// -----------------------------------------------------------------------------
// Siapkan ramp percepatan/decelerasi
// -----------------------------------------------------------------------------
bool ok = 0;
inline void prepareramp(int32_t bpos) {
  tmove *m, *next2, *next;
  m = &moves[bpos];
  int32_t nve;
  vi = m->fs;

  ve = 0;
  if (bpos != (head)) {
    int npos = nextbuff(bpos);
    next = &moves[npos];
    ve = next->fs;
  }
  int32_t fm =
      globalSpeed < fixed_max_multiplier ? globalSpeed : fixed_max_multiplier;
  int32_t fn = m->fn * fm * fm / 256;
  int32_t vcc = vi + (ve - vi + m->delta) / 2;
  vc = fmin(fn, vcc);
  int32_t fn_eff = isqrt(vc);

  int32_t accel3 = (int32_t)(m->dis * m->ac);
  if (accel3 <= 0)
    accel3 = 1;
  if (totalstep <= 0)
    totalstep = 1;

  // Optimasi: Gunakan fixed-point reciprocal untuk menghindari 64-bit division
  // yang lambat
  int32_t inv_accel3 = (int32_t)((1LL << 30) / accel3);
  acup = acdn = (int32_t)(((int64_t)accel3 << 12) / totalstep);

  // Hitung panjang segmen dengan fixed-point math
  int32_t delta_v_up = vc - vi;
  int32_t delta_v_dn = vc - ve;
  s2 = (int32_t)((int64_t(totalstep) * delta_v_up * inv_accel3) >> 30);
  s6 = (int32_t)((int64_t(totalstep) * delta_v_dn * inv_accel3) >> 30);
  // Guard: s2 dan s6 tidak boleh negatif
  if (s2 < 0)
    s2 = 0;
  if (s6 < 0)
    s6 = 0;
  s4 = totalstep - (s2 + s6);

  sg = 0;
  Sdest = 0;
  V = vi << 12;
  lV = 0;
  ok = 0;
  lastError = 0;

  // Set kecepatan referensi segment untuk kompensasi laser (pakai fn_eff)
  currentSegmentPeakV = fn_eff;
  if (currentSegmentPeakV == 0)
    currentSegmentPeakV = 1; // Mencegah Divide by Zero
  invCurrentSegmentPeakV = (1 << 16) / currentSegmentPeakV;
}
// -----------------------------------------------------------------------------
// Loop segmen gerakan - HOT PATH, harus secepat mungkin
// -----------------------------------------------------------------------------
void segmentloop() {
  if (UNLIKELY(nextseb(sebhead) == sebtail)) {
    return;
  }

  if (!ok) {
    sg++;
    // Cascade dengan early-exit pattern - menghindari multiple comparisons
    switch (sg) {
    case 1:
      if (LIKELY(s2 > 0)) {
        a2 = acup;
        Sdest = s2;
        break;
      }
      sg = 2;
      // fall through
    case 2:
      if (LIKELY(s4 > 0)) {
        V = vc << 12;
        Sdest = s4;
        a2 = 0;
        break;
      }
      sg = 3;
      // fall through
    case 3:
      if (LIKELY(s6 > 0)) {
        a2 = -acdn;
        Sdest = s6;
        break;
      }
      sg = 4;
      // fall through
    case 4:
      mctr = 0;
      return;
    }
  }

  ok = stepgen();
}

// -----------------------------------------------------------------------------
// Planner backward dan forward pass dengan optimasi stable flag
// -----------------------------------------------------------------------------

void backforward(uint32_t nhead) {
  uint32_t h;
  tmove *next, *curr;

  // --- Backward Pass ---
  h = nhead;

  if (h == (uint32_t)tailok)
    return;

  curr = 0;
  int32_t fes = motionCfg.minPlannerSpeed;

  while (1) {
    next = curr;
    curr = &moves[h];
    if (curr->fs != curr->maxs) {
      curr->fs =
          fmin(fes + curr->delta, curr->maxs); // maximum speed from next start
    }
    fes = curr->fs;
    // if (curr->fs == curr->maxs) break;   // ← BARU: hulu tak mungkin berubah lagi
    if ((h = prevbuff(h)) == tailok) break;
  }
  // --- Forward Pass ---
  // Cukup dari titik berhenti backward: hulu titik itu tak berubah sehingga
  // pindai ulang dari tailok pemborosan. Mundur satu langkah agar edge masuk
  // ke titik berhenti ikut dievaluasi ulang; bila berhenti tepat di tailok,
  // pindai penuh seperti semula.
  uint32_t fwdStart = (h == (uint32_t)tailok) ? (uint32_t)tailok : prevbuff(h);
  next = &moves[fwdStart];
  int ph = fwdStart;
  h = fwdStart;
  uint32_t stopH = nextbuff(nhead);
  while ((h = nextbuff(h)) != stopH) {
    curr = next;
    next = &moves[h];
    if (curr->fs < next->fs) {
      int32_t fs = curr->fs + curr->delta; // maximum speed at end of current move
      if (fs < next->fs) {
        next->fs = fs;
        tailok = ph;
      }
    }
    if (next->fs == next->maxs)
      tailok = ph;
    ph = h;
  }
}

// -----------------------------------------------------------------------------
// Planner
// -----------------------------------------------------------------------------

void planner(int32_t h) {
  pprev = pcurr;
  pcurr = &moves[h];
  prevu[0] = curru[0];
  prevu[1] = curru[1];
  prevu[2] = curru[2];

  int32_t ac = 10000;
  int32_t fn = pcurr->fn;

  // 1. Hitung vektor unit (arah) untuk segmen ini
  pcurr->maxv = 63;
  float rcDis = 1.0f / pcurr->dis;
  for (int i = 0; i < 3; i++) {
    curru[i] = 0;
    if (pcurr->dx[i] != 0) {
      curru[i] = (mmdis[i] * rcDis);
      // Batasi akselerasi berdasarkan komponen axis terlemah
      float rCur = 1.0f / curru[i];
      ac = fmin(ac, abs(cfg[i].acc * rCur));

      // Limit kecepatan maksimal berdasarkan setting axis
      int32_t fx = abs(cfg[i].vel * rCur);
      // Hitung ulang maxv dalam unit scaled (untuk planner internal)
      if (fn > 0) {
        pcurr->maxv = fmin(pcurr->maxv, fx * 16 / fn);
      } else {
        pcurr->maxv = 16;
      }
      if (fx < fn) {
        fn = fx;
        pcurr->maxv = 16;
      }
    }
  }

  pcurr->ac = ac << 1;
  pcurr->delta = abs(pcurr->ac * pcurr->dis);
  pcurr->fn = sqr(fn);

  // ---------------------------------------------------------------------------
  // Cornering Logic (Junction Deviation)
  // ---------------------------------------------------------------------------
  pcurr->maxs = 0;
  if (bufflength > 1) {
    // Dot product vektor unit saat ini & sebelumnya = cos(sudut)
    float junc_cos =
        -prevu[0] * curru[0] - prevu[1] * curru[1] - prevu[2] * curru[2];

    if (junc_cos < 0.999) {    // Jika ada belokan (cos < 1)
      pcurr->maxs = pcurr->fn;

      if (junc_cos > -0.999) { // Jika bukan balik arah total (180 derajat)
        // Hitung kecepatan cornering maksimal berdasarkan gaya sentripetal
        float sin_theta_d2 = sqrt(0.5 * (1 - junc_cos));
        float R =
            (motionCfg.junctionDev * sin_theta_d2) / (1.0 - (sin_theta_d2));

        // Kecepatan max di corner ~ akar(Akselerasi * Radius)
        pcurr->maxs = fmin(pcurr->maxs, R * pcurr->ac);

        // Path smoothing heuristic untuk segmen sangat pendek yang lurus
        if (pcurr->dis < 0.5f && junc_cos > 0.9f) {
          pcurr->maxs *= 1.5f;
        }
      }
    }
  }
  // lastf = pcurr->fn;

  backforward(h);
}

// -----------------------------------------------------------------------------
// Hitung total jarak yang masih ada di buffer (dari tail ke head)
// -----------------------------------------------------------------------------
float getBufferedDistance() {
  float totalDist = 0;
  uint32_t idx, h;
  idx = tail;
  h = head;
  while (idx != h) {
    totalDist += moves[idx].dis;
    idx = nextbuff(idx);
  }
  return totalDist;
}

// -----------------------------------------------------------------------------
// Flush buffer jog, sisakan 1 segmen jika sedang berjalan
// -----------------------------------------------------------------------------
float MAX_JOG_BUFFER_DISTANCE = 50.0; // Maksimal 200mm jog di buffer

void flushJogBuffer() {
  if (buffempty)
    return;

  SYNC_START();

  if (m != 0) {
    // Segmen sedang jalan: buang antrean SESUDAH segmen aktif (tail+1),
    // jangan segmen aktifnya sendiri — memotongnya di tengah mencabut
    // data yang sedang dipakai ISR/stepgen.
    uint32_t nextIdx = nextbuff(tail);
    if (nextIdx != head) {
      head = nextIdx;
    }
  } else {
    head = tail;
  }

  SYNC_END();

  log("Jog buffer flushed\n");

  if (!buffempty) {
    backforward(head);
  }
}

// -----------------------------------------------------------------------------
// Tambah gerakan ke buffer
// -----------------------------------------------------------------------------

void addmove(float cf, float cx2, float cy2, float cz2, int toolPower, int g0,
             int rel, uint8_t jogFlags) {
  int32_t x2[3] = {0};
  if (rel) {
    cx2 += cx1;
    cy2 += cy1;
    cz2 += cz1;
  }

  // Hitung jarak baru setiap axis dalam mm
  mmdis[0] = (cx2 - cx1);
  mmdis[1] = (cy2 - cy1);
  mmdis[2] = (cz2 - cz1);
  float total_dis = sqrtf(sqr(mmdis[0]) + sqr(mmdis[1]) + sqr(mmdis[2]));

  // Hitung step dan cek arah (dibutuhkan oleh extension dan limit)
  x2[0] = (((int32_t)(cx2 * cfg[0].stepsPerMm) -
            (int32_t)(cx1 * cfg[0].stepsPerMm)));
  x2[1] = (((int32_t)(cy2 * cfg[1].stepsPerMm) -
            (int32_t)(cy1 * cfg[1].stepsPerMm)));
  x2[2] = (((int32_t)(cz2 * cfg[2].stepsPerMm) -
            (int32_t)(cz1 * cfg[2].stepsPerMm)));
  // Cek perubahan arah dan terapkan kompensasi backlash
  int32_t dd = 0;
  int faxis = 0;
  for (int i = 0; i < 3; i++) {
    int8_t currentDir = (mmdis[i] > 0) ? 1 : (mmdis[i] < 0) ? -1 : 0;
    if (lastMoveDir[i] != 0 && currentDir != 0 &&
        lastMoveDir[i] != currentDir) {
      // Arah berubah, tambahkan kompensasi backlash
      // on -100step/mm , 0.1mm backlash, change dir from + to -
      // backlash = 10step (abs steppermm * backlash in mm)
      // negative direction x2= -10
      // positive direction x2= +10
      x2[i] += x2[i] > 0 ? cfg[i].backlash : -cfg[i].backlash;
    }
    if (currentDir != 0)
      lastMoveDir[i] = currentDir;
    int32_t delta = abs(x2[i]);
    if (delta > dd) {
      dd = delta;
      faxis = i;
    }
  }

  if (dd > 0) {
    // Limit jarak jog
    if (!jobRunning && (jogFlags & JOG_FLAG_LIMIT)) {
      float bufferedDist = getBufferedDistance();
      if (bufferedDist > 1 &&
          bufferedDist + total_dis > MAX_JOG_BUFFER_DISTANCE) {
        return;
      }
      lastJogTime = millis();
    }

    // Catat Z tertinggi yang disuruh selama job (basis menu Go to Job
    // Start/Zero).
    if (jobRunning && cz2 > jobMaxZ)
      jobMaxZ = cz2;

    int32_t nhead = nextbuff(head);
    tmove *curr = &moves[nhead];
    curr->dx[0] = x2[0];
    curr->dx[1] = x2[1];
    curr->dx[2] = x2[2];
    curr->dis = total_dis;
    curr->status = g0 ? 8 : 0;
    curr->toolPower = toolPower;
    curr->fn = cf;
    curr->fs = 0;
    curr->status |= faxis << 4;

    // Plan the move in the next slot BEFORE incrementing head
    // This ensures Core 1 only sees fully initialized and planned moves
    planner(nhead);
    head = nhead;


    cx1 = cx2;
    cy1 = cy2;
    cz1 = cz2;
  }
}

// -----------------------------------------------------------------------------
// Mulai gerakan dari buffer
// -----------------------------------------------------------------------------
inline int32_t startmove() {
  int t = nextbuff(tail);
  m = &moves[t];
  if (m->dx[0] == 0 && m->dx[1] == 0 && m->dx[2] == 0) {
    tail = t;
    return 0;
  }

  fastaxis = (m->status >> 4) & 3;
  totalstep = labs(m->dx[fastaxis]);

  // Harus SEBELUM prepareramp() — cruise speed segmen pertama ikut kap ini.
  fixed_max_multiplier = m->maxv;
  prepareramp(t);

  // Skala Q8 + pembulatan 0.5: sisa bagi tak hilang (pasangan cacheTm di
  // stepgen yang menggeser <<6). Tanpa ini raster kecil-cepat truncasi ke 0.
  stepdiv2 =
      (uint32_t)((float)CLOCKCONSTANT * m->dis / (float)totalstep * 256.0f +
                 0.5f);
  m->status &= ~3;
  m->status |= 2;
  isG0 = m->status & 8;

  mcx[0] = mcx[1] = mcx[2] = 0;

  tail = t;

  // Unrolled loop untuk 3 axis - menghilangkan overhead loop
  if (m->dx[0] > 0) {
    bsdx[0] = m->dx[0];
    sx[0] = 1;
  } else {
    bsdx[0] = -m->dx[0];
    sx[0] = -1;
  }
  if (m->dx[1] > 0) {
    bsdx[1] = m->dx[1];
    sx[1] = 1;
  } else {
    bsdx[1] = -m->dx[1];
    sx[1] = -1;
  }
  if (m->dx[2] > 0) {
    bsdx[2] = m->dx[2];
    sx[2] = 1;
  } else {
    bsdx[2] = -m->dx[2];
    sx[2] = -1;
  }

  mctr = totalstep;
  mpower = (isG0 && toolCfg.mode == 1) ? 0 : m->toolPower;

  return 1;
}

// -----------------------------------------------------------------------------
// Loop gerakan utama
// -----------------------------------------------------------------------------
void motionloop() {
  // Proses homing state machine jika sedang homing
  if (isHoming)
    processHoming();

  bool canRun = !(buffempty || jobPaused || stopping);

  if (m && !jobPaused && !stopping) {
    // Generation DIHENTIKAN saat pause/stop — state segmen (sg/V/Sdest/mcx)
    // utuh → resume lanjut sisa segmen; decel ISR bekerja di sisa buffer.
    for (int i = 0; i < 15; i++) {
      segmentloop();
      if (mctr <= 0) {
        if (m)
          m->status = 0;
        m = pcurr = 0;
        return;
      }
      if (sebfull)
        break;
    }
  } else {
    if (canRun) {
      // Logic Delay: Jangan langsung start jika baru ada 1 move (memberi waktu
      // planner) Hanya berlaku jika bukan sedang menjalankan file job
      if (bufflength == 1 && !jobRunning) {
        if (bufferFillStartTime == 0)
          bufferFillStartTime = millis();
        if (millis() - bufferFillStartTime < 25)
          return;
      }

      bufferFillStartTime = 0; // Reset timer
      startmove();
    }
  }
}

// =============================================================================
// SECTION 6: JOB MANAGEMENT
// =============================================================================

// -----------------------------------------------------------------------------
// Reset Shapers
// -----------------------------------------------------------------------------
void resetShapers() {
  for (int i = 0; i < 3; i++) {
    shapers[i].signedAccum[0] = 0;
    shapers[i].signedAccum[1] = 0;
    shapers[i].accumulator = 0;
  }
  // SOA layout: shaperBuffer[tick][axis] - clear semua buffer
  if (shaperBuffer)
    memset(shaperBuffer, 0, SHAPER_BUFFER_SIZE * sizeof(*shaperBuffer));
  sebhead = 0;
  sebtail = 0;
  tokenHead = tokenTail = 0;
  globalShaperPushedSteps = globalShaperPoppedSteps = 0;
  lastPushedPwm = currentActivePwm = 0;
}
//-----------------------------------------------------------------------------
// Stop machine
//-----------------------------------------------------------------------------
void stopmachine() {
  jobPaused = false;
  stopping = false;
  // Matikan PWM saat job selesai
  currentActivePwm = 0;
  pwmValue = 0;
  pwmAccumulator = 0;
  head = tail;
  updateRealPos();
}

// -----------------------------------------------------------------------------
// Update Real Position
// -----------------------------------------------------------------------------
void updateRealPos() {
  cx1 = shapers[0].realStep * shapers[0].invStepsPerMm;
  cy1 = shapers[1].realStep * shapers[1].invStepsPerMm;
  cz1 = shapers[2].realStep * shapers[2].invStepsPerMm;
}
void autoDeleteJobFile(const char *jobName);
// -----------------------------------------------------------------------------
// Proses job utama
// -----------------------------------------------------------------------------
void processJob() {
  // // Sabuk pengaman ekor shaper: jangan stopmachine sebelum delay terpanjang
  // // (+2 tick) berlalu sejak event terakhir — energi antre tak terlihat dari
  // // buffer move/event.
  // uint32_t tailDelay =
  //     (uint32_t)(shapers[0].maxDelayTicks > shapers[1].maxDelayTicks
  //                    ? shapers[0].maxDelayTicks
  //                    : shapers[1].maxDelayTicks) +
  //     2;
  // bool tailDrained =
  //     (uint32_t)(absTime - lastEventTime) > tailDelay || !shaper_enabled;
  // if ((stopping || buffempty) && sebempty && shaperEmpty && tailDrained) {

  // DETECTFREEZE = !(stopping || buffempty) && (millis() - jobStartTime > 5000);
  if ((stopping || buffempty) && sebempty && shaperEmpty) {
    if (isPreviewMode) {
      isPreviewMode = false;
    }
    if (isParsingComplete || stopping) {
      stopmachine();
      if (jobRunning) {
        isParsingComplete = false;
        jobRunning = false;
        requestLcdReset(); // heal LCD setelah motor benar-benar diam
        lastJobDuration = millis() - jobStartTime;
        log("Job selesai X%f Y%f, durasi: %lu s\n", cx1, cy1,
            lastJobDuration / 1000);
        // Simpan ke logjob.txt
        {
          File logFile = LittleFS.open("/logjob.txt", "r+");
          if (!logFile)
            logFile = LittleFS.open("/logjob.txt", "w");
          if (logFile) {
            logFile.seek(logFile.size());
            time_t now =
                bootEpoch ? bootEpoch + (millis() / 1000) : millis() / 1000;
            logFile.printf("%ld,%s,%lu\n", (long)now,
                           currentJobFilename.c_str(), lastJobDuration / 1000);
            logFile.close();
          }
        }
        // Report ke karyacnc bahwa job sudah selesai
        autoDeleteJobFile(currentJobFilename.c_str());
        reportJobComplete(currentJobFilename.c_str(), true);
      }
    }
  }
  if (!stopping && jobRunning) {
    // Isi move buffer sampai penuh atau file habis.
    // Batas waktu 5ms sebagai safety untuk text G-code yang parsing-nya lambat.
    uint32_t parseStart = millis();
    int32_t sebFree = (int32_t)STEP_EVENT_BUFFER_SIZE - (int32_t)seblength;
    bool bufferSafe =
        (sebFree < (int32_t)(STEP_EVENT_BUFFER_SIZE >> 1)); // slot kosong <50%
    while (!bufffull && !isParsingComplete) {
      parseNextGCodeSegments();
      if (millis() - parseStart >= 5)
        break; // Jangan blocking >5ms
      if (!bufferSafe)
        break; // jika buffer tidak aman, hentikan parsing, cukup 1x
    }
  }
}

// -----------------------------------------------------------------------------
// Start Job
// -----------------------------------------------------------------------------
void startJob(String filename) {
  if (jobRunning) {
    return;
  }

  extern void syncJobSelection(String filename);
  syncJobSelection(filename);

  if (currentJobFile) {
    currentJobFile.close();
  }

  String filepath = "/" + filename;
  JobX = cx1;
  JobY = cy1;
  JobZ = cz1;
  jobMaxZ = cz1; // baseline = Z saat job start

  // Pending step Z direset 0 setiap job mulai (klik & lupakan per-job).
  resetZTrim();
  extern float previewMinX;
  extern float previewMaxX;
  extern float previewMinY;
  extern float previewMaxY;
  extern bool previewFirstMove;
  isPreviewMode = false;
  previewFirstMove = true;
  previewMinX = previewMaxX = cx1;
  previewMinY = previewMaxY = cy1;

  log("JobX: %f, JobY: %f, JobZ: %f\n", JobX, JobY, JobZ);

  // Try to open as GX format first
  if (openGxFile(filepath)) {
    log("Opening GX format file: %s\n", filename.c_str());
  } else {
    // Open as standard text G-code
    isGxMode = false;
    currentJobFile = LittleFS.open(filepath, "r");
    if (!currentJobFile) {
      return;
    }
  }

  jobFileSize = currentJobFile.size();
  jobBytesRead = 0;
  isParsingComplete = false;
  log("Mulai job X%f Y%f\n", JobX, JobY);

  currentJobFilename = filename;
  jobRunning = true;
  jobPaused = false;
  lastResumeMs =
      millis() - PAUSE_COOLDOWN_MS; // pause langsung tersedia di job baru
  isrRampMode = 0;                  // job baru mulai tanpa ramp
  jobStartTime = millis();

  extern uint32_t ldisplayTime;
  ldisplayTime = lastEventTime = lastRecordTimeMs = absTime = 0;

  G0Feed = 2000;
  G1Feed = 2000;

  currentToolPower = 0;
  mpower = 0;
  pwmValue = 0;
  pwmAccumulator = 0;

  resetShapers();

  // Reset position history
  positionHistoryIndex = 0;
}

// -----------------------------------------------------------------------------
// Stop Job
// -----------------------------------------------------------------------------
void stopJob() {
  stopping = true;
  pauseParkReset();
  // decel ISR — sisa event buffer dibuang sambil melambat mulus
  if (!sebempty) {
    isrRampMode = 1;
    rampStartTicks = absTime;
  }
  head = tail;
  if (m)
    m->status = 0;
  m = pcurr = 0;
}

// -----------------------------------------------------------------------------
// Toggle Job Pause
// -----------------------------------------------------------------------------
// Ditolak saat ramp masih berjalan (isrRampMode != 0). Berlaku untuk semua
// sumber perintah: remote IR, web, dan ESP-NOW.
static uint32_t lastMotionSeen = 0;

void toggleJobPause() {
  uint32_t now = millis();
  if (isrRampMode !=0)return;
  if (!jobPaused) {
    // PAUSE: decel ISR — event tersisa diberi delay aditif sehingga mesin
    // melambat halus, bukan berhenti dadakan dari cruise
    isrRampMode = 1;
    rampStartTicks = absTime;
    requestLcdReset(); // heal layar diminta saat pause saja (resume tak perlu)
    lastMotionSeen = millis();
  } else {
    // RESUME: selalu diizinkan (safety). Accel ditangani stepgen via FREEZING
    // → mode 2 (ramp ~73ms).
    FREEZING = true;
  }
  jobPaused = !jobPaused;
  log("Job %s\n", jobPaused ? "Paused" : "Resumed");
}

// -----------------------------------------------------------------------------
// Pause with Park: saat pause, catat Z, naik ke Z0 (spindle off), parkir.
// Saat resume (HANYA tombol Play): restore ke Z tersimpan lalu lanjut job.
// Tombol pause TIDAK toggle — tidak akan langsung jalan lagi walau terbaca
// 2x. Pending step trim tetap berfungsi saat parkir (Z0).
//
// PENTING: park/restore TIDAK memakai move buffer/planner (yang berisi sisa
// segmen job). Z digerakkan langsung oleh ISR (mekanisme yang sama dengan
// Z-trim) menuju target realStep. Jadi park jalan walau `m`/buffer masih
// berisi segmen tersisa — tidak macet menunggu buffer kosong.
// -----------------------------------------------------------------------------
volatile uint8_t pauseSeq = 0; // 0=idle,1=tunggu berhenti,2=park ke Z0,
                               // 3=parkir,4=restore ke Z tersimpan
volatile int32_t pausedRealStepZ = 0; // realStep Z tercatat saat pause
volatile bool resumeWanted = false;   // Play ditekan saat masih melambat
volatile int32_t parkTargetSteps = 0; // target realStep Z saat park/restore
volatile bool parkMoveDone = false;   // park/restore sudah sampai target
volatile uint32_t parkTimer = 0;      // counter tick antar step park
volatile uint32_t parkStepInterval = 0; // interval park terhitung (dipakai ISR)
volatile uint8_t savedPausePwm = 0;  // PWM asli saat pause (untuk restore spindle)

void remotePause() {
  if (!jobRunning || jobPaused || stopping || isHoming || pauseSeq != 0)
    return;
  toggleJobPause(); // mulai decel halus, jobPaused = true
  pauseSeq = 1;
  resumeWanted = false;
}

// Interval step Z untuk fraksi V max (mm/s, sesuai web config):
//   steps/detik = (frac × vel) × spm
//   interval tick = 62500 / steps/detik
// Dilindungi pembatas keras minimum interval agar tidak pernah overspeed.
#define PARK_MIN_TICKS 40 // 40 tick × 16µs = 0.64ms → max ~1562 step/detik
uint32_t speedFractionIntervalTicks(float velMmS, float spm, float frac) {
  if (velMmS < 1.0f || spm < 1.0f)
    return 2000;
  uint32_t iv = (uint32_t)(62500.0f / (frac * velMmS * spm));
  if (iv < PARK_MIN_TICKS)
    iv = PARK_MIN_TICKS;
  return iv;
}

// Park & restore: 1/2 V max Z.
uint32_t parkIntervalTicks() {
  return speedFractionIntervalTicks(fabsf(cfg[2].vel),
                                    fabsf(cfg[2].stepsPerMm), 0.5f);
}

void remoteResume() {
  if (!jobRunning || !jobPaused || stopping)
    return;
  if (pauseSeq == 1) { // masih melambat → lanjut resume setelah berhenti
    resumeWanted = true;
    return;
  }
  if (pauseSeq == 2 || pauseSeq == 3) {
    pauseSeq = 4; // mulai restore ke Z tersimpan
    parkTargetSteps = pausedRealStepZ;
    parkMoveDone = false;
    parkStepInterval = parkIntervalTicks();
    parkTimer = parkStepInterval;
    // Nyalakan spindle SEBELUM memulai gerakan turun menuju kedalaman
    // potong (mode router). ISR diizinkan mengeluarkan PWM saat fase restore.
    if (toolCfg.mode == 0 && savedPausePwm > 0) {
      currentToolPower = savedPausePwm;
      pwmValue = savedPausePwm;
    }
    return;
  }
  // fallback: resume langsung
  pauseSeq = 0;
  jobPaused = false;
  FREEZING = true;
}

void pauseParkReset() {
  pauseSeq = 0;
  resumeWanted = false;
  parkTargetSteps = 0;
  parkMoveDone = false;
  parkTimer = 0;
}

// Dipanggil tiap loop utama. Mengeksekusi sequence park/restore. Syarat
// "mesin diam" hanya event/shaper habis & ramp selesai — TIDAK menunggu
// buffer/m kosong (itu justru penyebab macet sebelumnya).
void processPauseSequence() {
  if (!jobRunning) {
    pauseParkReset();
    return;
  }
  bool stopped = (isrRampMode == 0) && sebempty && shaperEmpty;

  if (pauseSeq == 1) {
    if (!stopped)
      return;
    if (resumeWanted) {
      resumeWanted = false;
      pauseSeq = 0;
      jobPaused = false;
      FREEZING = true; // accel ramp
    } else {
      pausedRealStepZ = shapers[2].realStep;
      parkTargetSteps = 0; // Z0 = realStep 0 (referensi Set Zero)
      parkMoveDone = false;
      parkStepInterval = parkIntervalTicks();
      parkTimer = parkStepInterval;
      pauseSeq = 2;
    }
  } else if (pauseSeq == 2) {
    // Self-heal: kalau ISR tidak sempat men-set flag, deteksi langsung
    if (shapers[2].realStep == parkTargetSteps)
      parkMoveDone = true;
    if (!parkMoveDone)
      return;
    updateRealPos(); // sinkronkan cz1 dengan posisi fisik Z0
    // Catat PWM asli sebelum dimatikan — dipakai untuk menyalakan lagi
    // saat Play (sebelum turun menuju kedalaman potong).
    savedPausePwm = (uint8_t)pwmValue;
    pwmValue = 0;
    GPOClear(maskToolPwm);
    if (toolCfg.pinToolOn < 255)
      GPOClear(maskToolOn);
    pauseSeq = 3;
  } else if (pauseSeq == 4) {
    // Self-heal: sama seperti s2
    if (shapers[2].realStep == parkTargetSteps)
      parkMoveDone = true;
    if (!parkMoveDone)
      return;
    updateRealPos(); // sinkronkan cz1 dengan posisi fisik restore
    pauseSeq = 0;
    jobPaused = false;
    FREEZING = true; // lanjut job (accel ramp)
  }
  // pauseSeq == 3: parkir — tunggu tombol Play
}

// =============================================================================
// SECTION 7: MANUAL CONTROL
// =============================================================================

// -----------------------------------------------------------------------------
// Jog soft-limits (mesin tanpa endstop — cegah nabrak rail)
// Aturan: di DALAM window → bebas; keluar window → langkah besar dipotong ke
// batas; sudah DI LUAR window → hanya boleh langkah <= 1mm (pelan-pelan).
// -----------------------------------------------------------------------------
#define JOG_MIN_X -40.0f
#define JOG_MAX_X 1500.0f
#define JOG_MIN_Y -40.0f
#define JOG_MAX_Y 2600.0f

// Clamp delta jog 1 axis terhadap work window. Return delta yang diizinkan
// (boleh 0).
float clampJogAxis(float pos, float step, float lo, float hi) {
  // Fitur soft-limit bisa dimatikan dari Web UI / menu LCD "Jog Limit".
  // Saat OFF, jog bebas ke koordinat negatif / luar window (mis. butuh
  // zero di luar area kerja).
  if (!systemCfg.jogLimitEnabled)
    return step;
  if (step == 0)
    return 0;
  if (pos < lo || pos > hi) {
    // Sudah di luar window (mis. user zero di luar area): hanya boleh <= 1mm
    if (fabsf(step) > 1.0f)
      step = (step > 0) ? 1.0f : -1.0f;
    float t = pos + step;
    if ((step > 0 && t < pos) || (step < 0 && t > pos))
      return 0; // arah makin keluar → blokir
    return step;
  }
  float t = pos + step;
  if (t < lo || t > hi) {
    // Dari dalam menuju luar: potong tepat di batas
    // (HAPUS 2 baris di bawah jika ingin big-step boleh menembus batas sekali)
    if (fabsf(step) > 1.0f)
      step = (step > 0) ? 1.0f : -1.0f;
    t = pos + step;
    if (t < lo)
      t = lo;
    if (t > hi)
      t = hi;
  }
  return t - pos;
}

// -----------------------------------------------------------------------------
// Gerakkan multiple axis secara manual (multi-axis jog) - diagonal
// -----------------------------------------------------------------------------
void jogMulti(float distX, float distY, float distZ) {
  log("Jog multi-axis: X=%f, Y=%f, Z=%f\n", distX, distY, distZ);
  touchLcdActivity(); // web jog = aktivitas (anti idle-watchdog)

  // Tolak nilai tidak-sah dari jaringan (NaN/Inf meracuni planner)
  if (!isfinite(distX) || !isfinite(distY) || !isfinite(distZ))
    return;

  // Soft-limit X/Y (sama seperti jog remote IR)
  distX = clampJogAxis(cx1, distX, JOG_MIN_X, JOG_MAX_X);
  distY = clampJogAxis(cy1, distY, JOG_MIN_Y, JOG_MAX_Y);
  if (distX == 0 && distY == 0 && distZ == 0)
    return;

  float feed = 100;

  // JOG_FLAG_LIMIT: batasi jarak total di buffer (anti backlog saat hold)
  addmove(feed, distX, distY, distZ, currentToolPower, 1, 1,
          JOG_FLAG_LIMIT | JOG_FLAG_EXTEND);
}

// -----------------------------------------------------------------------------
// Z-Trim: hitung jeda tick antar step trim agar tidak melebihi kecepatan
// motor Z. Dipanggil saat boot & setelah config dimuat ulang.
// -----------------------------------------------------------------------------
void updateZTrimTiming() {
  // Trim Z memakai perhitungan yang sama dengan park: 1/2 V max Z (mm/s).
  zTrimIntervalTicks =
      speedFractionIntervalTicks(fabsf(cfg[2].vel),
                                 fabsf(cfg[2].stepsPerMm), 0.5f);
}

// -----------------------------------------------------------------------------
// Terapkan trim Z (mm). Target ditambah; ISR yang meng-apply (trimAppliedSteps)
// saat Z job diam & dengan kecepatan ≤ vel max Z.
// -----------------------------------------------------------------------------
void applyZTrim(float deltaMm) {
  if (isHoming)
    return; // jangan ganggu homing
  if (!isfinite(deltaMm))
    return;
  int32_t steps = (int32_t)(deltaMm * cfg[2].stepsPerMm);
  if (steps == 0)
    return;

  // Target bertambah/berkurang sebesar delta (sticky antar job).
  // Tanda steps mengikuti stepsPerMm config (positif/negatif) → arah +/−
  // konsisten dengan sistem.
  int32_t newTarget = trimTargetSteps + steps;
  int32_t maxT = (int32_t)(20.0f * fabsf(cfg[2].stepsPerMm)); // ±20mm
  if (newTarget > maxT)
    newTarget = maxT;
  if (newTarget < -maxT)
    newTarget = -maxT;
  trimTargetSteps = newTarget;

  // Jangan tembak di tengah gerak Z yang sedang berlangsung
  zTrimTimer = zTrimIntervalTicks + Z_TRIM_QUIET_TICKS;

  log("Z-Trim: target %+ld (%+.2fmm), applied %ld\n", (long)trimTargetSteps,
      (double)trimTargetSteps / cfg[2].stepsPerMm,
      (long)trimAppliedSteps);
}

// -----------------------------------------------------------------------------
// Reset trim Z (menu "Trim Reset", Set Zero, homing)
// -----------------------------------------------------------------------------
void resetZTrim() {
  trimTargetSteps = 0;
  trimAppliedSteps = 0;
  zTrimTimer = 0;
  log("Z-Trim reset\n");
}

// -----------------------------------------------------------------------------
// Reset semua posisi dan status
// -----------------------------------------------------------------------------
void resetAll() {
  if (jobRunning)
    return;

  resetZTrim(); // Set Zero/setel ulang acuan → trim ikut direset
  cx1 = 0;
  cy1 = 0;
  cz1 = 0;
  tailok = 0;
  // lastf = 0;

  shapers[0].realStep = 0;
  shapers[1].realStep = 0;
  shapers[2].realStep = 0;

  absTime = lastEventTime = lastRecordTimeMs = 0;
  m = 0;

  resetShapers();

  // Reset laser event buffer
  sebhead = 0;
  sebtail = 0;

  // PENTING: jangan sentuh /headpos.bin di sini — resetAll() juga dipanggil
  // setup() di SETIAP boot (menghapus file di sini = posisi rescue hilang
  // sebelum blok restore di setup sempat membacanya). Sinkronisasi file ke
  // zero dilakukan Set Zero (ui.cpp actionSetZero) via saveHeadPosition().
}

// -----------------------------------------------------------------------------
// Mulai proses homing (non-blocking)
// -----------------------------------------------------------------------------
void startHoming(int axis) {
  if (isHoming) {
    log("Sudah sedang homing, abaikan\n");
    return;
  }

  isHoming = true;
  homingQueueIdx = 0;
  homingState = HOMING_IDLE;

  if (axis == 3) {
    // Home semua axis
    for (int i = 0; i < 3; i++) {
      if (cfg[i].hasEndstop) {
        homingQueue[homingQueueIdx++] = i;
      }
    }
  } else if (axis >= 0 && axis < 3) {
    // Home single axis
    if (cfg[axis].hasEndstop) {
      homingQueue[homingQueueIdx++] = axis;
    }
  }

  if (homingQueueIdx > 0) {
    log("Memulai homing, %d axis di-queue\n", homingQueueIdx);
  } else {
    log("Tidak ada axis dengan endstop untuk di-home\n");
    isHoming = false;
  }
}

// -----------------------------------------------------------------------------
// Proses homing state machine (non-blocking, dipanggil dari motionloop)
// -----------------------------------------------------------------------------
void processHoming() {
  switch (homingState) {
  case HOMING_IDLE:
    if (homingQueueIdx > 0) {
      // Buffer penuh: jangan majukan antrean, coba lagi tick berikut
      if (bufffull)
        break;
      homingCurrentAxis = homingQueue[0];
      homingQueueIdx--;
      homingState = HOMING_SEARCH;

      float startPos = 0;
      switch (homingCurrentAxis) {
      case 0:
        startPos = cx1;
        break;
      case 1:
        startPos = cy1;
        break;
      case 2:
        startPos = cz1;
        break;
      }

      float searchDir = (homingCurrentAxis == 2) ? 1.0 : -1.0;
      float searchDist = 500.0;
      float targetPos = startPos + searchDir * searchDist;

      // Kirim gerakan ke endstop
      float targetX = (homingCurrentAxis == 0) ? targetPos : cx1;
      float targetY = (homingCurrentAxis == 1) ? targetPos : cy1;
      float targetZ = (homingCurrentAxis == 2) ? targetPos : cz1;

      addmove(homingCfg.homingSpeed, targetX, targetY, targetZ, 0, 1, 0, false);

      homingTimeout = millis() + 30000;
      log("Homing axis %d, mencari endstop...\n", homingCurrentAxis);
    } else {
      // Semua axis selesai
      isHoming = false;
      log("Homing selesai\n");
    }
    break;

  case HOMING_SEARCH:
    if (isEndstopTriggered()) {
      // Buffer penuh: tahan di state ini, kirim pullback tick berikut
      if (bufffull)
        break;
      log("Endstop axis %d terdeteksi, pullback...\n", homingCurrentAxis);
      homingState = HOMING_PULLBACK;

      // Set posisi axis yang di-home ke 0
      switch (homingCurrentAxis) {
      case 0:
        cx1 = 0;
        shapers[0].realStep = 0;
        break;
      case 1:
        cy1 = 0;
        shapers[1].realStep = 0;
        break;
      case 2:
        cz1 = 0;
        shapers[2].realStep = 0;
        break;
      }

      // Pullback dengan gerakan relatif
      float searchDir = (homingCurrentAxis == 2) ? 1.0 : -1.0;
      float pullbackDist = -searchDir * homingCfg.homingPullback;

      float targetX = (homingCurrentAxis == 0) ? pullbackDist : 0;
      float targetY = (homingCurrentAxis == 1) ? pullbackDist : 0;
      float targetZ = (homingCurrentAxis == 2) ? pullbackDist : 0;

      // Gunakan relatif mode (1) untuk pullback
      addmove(homingCfg.homingSpeed, targetX, targetY, targetZ, 0, 1, 1, false);

    } else if (millis() > homingTimeout) {
      log("Timeout homing axis %d\n", homingCurrentAxis);
      isHoming = false;
    }
    break;

  case HOMING_PULLBACK:
    // Tunggu gerakan pullback selesai (tidak ada move aktif)
    if (!m && !jobRunning) {
      log("Axis %d homed, posisi di-set ke 0\n", homingCurrentAxis);
      homingState = HOMING_IDLE;
      delay(100);
    }
    break;
  }
}

// -----------------------------------------------------------------------------
// Home specific axis (called from web UI)
// -----------------------------------------------------------------------------
void homeAxisFromUI(int axis) { startHoming(axis); }

// -----------------------------------------------------------------------------
// Cek apakah endstop aktif (NC = LOW saat tidak ditekan, HIGH saat ditekan)
// -----------------------------------------------------------------------------
bool isEndstopTriggered() { return digitalRead(homingCfg.pinEndstop) == HIGH; }

// -----------------------------------------------------------------------------
// Setup pin endstop
// -----------------------------------------------------------------------------
void setupEndstopPin() {
  if (homingCfg.pinEndstop < 255) {
    pinMode(homingCfg.pinEndstop, INPUT_PULLUP);
  }
}

// =============================================================================
// SECTION 8: TOOL CONTROL
// =============================================================================

// -----------------------------------------------------------------------------
// Set tool power (S value dari G-code)
// -----------------------------------------------------------------------------
void setToolPower(int power) {
  if (power < 0)
    power = 0;
  if (power > 255)
    power = 255;
  currentToolPower = power;
  if (!jobRunning && toolCfg.mode == 0) {
    SYNC_START();
    pwmValue = power;
    SYNC_END();
  }
}

// -----------------------------------------------------------------------------
// non blocking Test laser
// -----------------------------------------------------------------------------
void testLaser() {
  if (jobRunning)
    return;
  if (isTestLaser)
    return;
  testingLaser = absTime + (systemCfg.testLaserTime * 1000) / TICK_MICROS;
  isTestLaser = true;
}

// -----------------------------------------------------------------------------
// Save Power-Fail State ke LittleFS
// Dipanggil saat deteksi mati PLN dan job sedang running
// -----------------------------------------------------------------------------
void savePowerFailState() {
  if (stateSaved)
    return; // Hanya save sekali

  // Set forceStop flag - ISR akan berhenti dan bersihkan buffer
  forceStop = true;
  extern File currentJobFile;
  // Stop job jika sedang running
  if (jobRunning && !isParsingComplete) {
    stopping = true;
    jobPaused = false;
    // Tutup file job
    currentJobFile.close();
  }

  // Simpan binary: 3x int32_t (realStep X,Y,Z) + 3x float (jobX,Y,Z)
  // Total 24 byte - jauh lebih cepat dari JSON
  int32_t realSteps[3];
  SYNC_START();
  realSteps[0] = shapers[0].realStep;
  realSteps[1] = shapers[1].realStep;
  realSteps[2] = shapers[2].realStep;
  SYNC_END();
  float jobPos[3] = {JobX, JobY, JobZ};

  File f = LittleFS.open(POWER_FAIL_FILE, "w");
  if (f) {
    f.write((uint8_t *)realSteps, sizeof(realSteps));
    f.write((uint8_t *)jobPos, sizeof(jobPos));
    f.close();
    stateSaved = true;
    log("State saved! Steps: %ld, %ld, %ld | Job: %.2f, %.2f, %.2f\n",
        realSteps[0], realSteps[1], realSteps[2], JobX, JobY, JobZ);
  }
}

// -----------------------------------------------------------------------------
// Load Power-Fail State dari LittleFS
// Dipanggil saat boot untuk cek apakah ada state tersimpan
// -----------------------------------------------------------------------------
bool loadPowerFailState() {
  if (!LittleFS.exists(POWER_FAIL_FILE)) {
    log("Power fail state not found\n");
    return false;
  }

  File f = LittleFS.open(POWER_FAIL_FILE, "r");
  if (!f || f.size() < 24) { // 12 byte realSteps + 12 byte jobPos
    if (f)
      f.close();
    return false;
  }

  int32_t realSteps[3];
  float jobPos[3];

  // Read binary data
  if (f.read((uint8_t *)realSteps, sizeof(realSteps)) != sizeof(realSteps) ||
      f.read((uint8_t *)jobPos, sizeof(jobPos)) != sizeof(jobPos)) {
    f.close();
    return false;
  }
  f.close();

  // Load ke resumeState
  resumeState.realStepX = realSteps[0];
  resumeState.realStepY = realSteps[1];
  resumeState.realStepZ = realSteps[2];

  resumeState.jobX = jobPos[0];
  resumeState.jobY = jobPos[1];
  resumeState.jobZ = jobPos[2];

  resumeState.valid = true;

  // Set flag needResume dan hapus file
  needResume = true;
  LittleFS.remove(POWER_FAIL_FILE);

  log("State loaded! Steps: %ld, %ld, %ld | Job: %.2f, %.2f, %.2f\n",
      realSteps[0], realSteps[1], realSteps[2], jobPos[0], jobPos[1],
      jobPos[2]);
  return true;
}

// -----------------------------------------------------------------------------
// Clear Power-Fail State
// Dipanggil setelah berhasil resume atau user memilih ignore
// -----------------------------------------------------------------------------
void clearPowerFailState() {
  if (LittleFS.exists(POWER_FAIL_FILE)) {
    LittleFS.remove(POWER_FAIL_FILE);
    log("Power-fail state cleared\n");
  }
  resumeState.valid = false;
}

// -----------------------------------------------------------------------------
// Simpan posisi head (realStep X/Y/Z) ke LittleFS.
// Tujuan: kalau ESP8266 terpaksa direstart saat kepala di tengah area kerja,
// posisi tidak dianggap 0,0,0 lagi saat boot — mudah jog pulang.
// Dipanggil saat motor diam di konsumen heal LCD; ditimpa saat user Set Zero.
// -----------------------------------------------------------------------------
void saveHeadPosition() {
  int32_t realSteps[3];
  SYNC_START();
  realSteps[0] = shapers[0].realStep;
  realSteps[1] = shapers[1].realStep;
  realSteps[2] = shapers[2].realStep;
  SYNC_END();
  File f = LittleFS.open(HEADPOS_FILE, "w");
  if (f) {
    f.write((uint8_t *)realSteps, sizeof(realSteps));
    f.close();
  }
}

// =============================================================================
// SECTION 9: CONFIGURATION
// =============================================================================

// -----------------------------------------------------------------------------
// Helper Functions for JSON Optimization
// -----------------------------------------------------------------------------
// Round float ke 3 desimal untuk menghemat ruang JSON
inline float round3(float value) { return roundf(value * 1000.0f) / 1000.0f; }

// -----------------------------------------------------------------------------
// Load Axis Config from JSON
// -----------------------------------------------------------------------------
void loadAxisConfig(JsonObject &axis, int axisIdx) {
  cfg[axisIdx].vel = axis["vel"];
  cfg[axisIdx].acc = axis["acc"];
  cfg[axisIdx].stepsPerMm = axis["stepsPerMm"];
  cfg[axisIdx].freq = axis["freq"];
  cfg[axisIdx].damp = axis["damp"];
  cfg[axisIdx].backlash = axis["backlash"];
  cfg[axisIdx].backlash *= abs(cfg[axisIdx].stepsPerMm);
  cfg[axisIdx].shaperType = axis["shaperType"];
  cfg[axisIdx].pinStep = axis["pinStep"];
  cfg[axisIdx].pinDir = axis["pinDir"];
  cfg[axisIdx].hasEndstop = axis["hasEndstop"] | 0;
}

// -----------------------------------------------------------------------------
// Write Default Config JSON
// -----------------------------------------------------------------------------
void writeConfigJson() {
  File f = LittleFS.open("/config.json", "w");
  if (!f) {
    log("Gagal membuka file config.json untuk write\n");
    return;
  }
  String configStr;
#if defined(ESP8266)
  configStr = FPSTR(configJson);
#else
  configStr = configJson;
#endif

  // Remove trailing '}' and inject cpuType
  if (configStr.endsWith("}")) {
    configStr.remove(configStr.length() - 1);
  }
  configStr += ",\"cpuType\":\"";
  configStr += cpuType;
  configStr += "\"}";

  f.print(configStr);

  f.close();
  log("Membuat file config.json untuk %s\n", cpuType);
}

// -----------------------------------------------------------------------------
// Set Global Speed
// -----------------------------------------------------------------------------
bool setGlobalSpeed(int speed) {
  if (speed < 25 || speed > 300) {
    return false;
  }
  globalSpeed = speed * 16 / 100;
  return true;
}

// -----------------------------------------------------------------------------
// Get Global Speed
// -----------------------------------------------------------------------------
int getGlobalSpeed() { return globalSpeed * 100 / 16; }

// -----------------------------------------------------------------------------
// Load All Config
// -----------------------------------------------------------------------------
bool loadAllConfig() {
  if (jobRunning) {
    log("Cannot load config while job is running\n");
    return false;
  }

  File configFile = LittleFS.open("/config.json", "r");
  if (!configFile) {
    log("Gagal membuka file konfigurasi /config.json\n");
    writeConfigJson();
    configFile = LittleFS.open("/config.json", "r");
  }

  size_t fileSize = configFile.size();
  log("Membuka config.json, ukuran: %d bytes\n", fileSize);
  if (fileSize == 0) {
    configFile.close();
    log("File config.json kosong, menulis default config\n");
    writeConfigJson();
    configFile = LittleFS.open("/config.json", "r");
  }
  if (!configFile) {
    log("Gagal membuka config.json setelah write\n");
    return false;
  }

  // Dokumen JSON hidup di heap selama parse saja — tak menetap di BSS.
  // ESP8266: jangan di stack (2 KB rawan overflow).
  DynamicJsonDocument doc(CONFIG_DOC_SIZE);
  DeserializationError error = deserializeJson(doc, configFile);
  configFile.close();

  if (error) {
    log("Gagal parsing konfigurasi: %s\n", error.c_str());
    return false;
  }

  log("Konfigurasi berhasil dimuat\n");

  // Load konfigurasi WiFi
  if (doc.containsKey("wifi")) {
    JsonObject wifi = doc["wifi"];
    wifiSSID = wifi["ssid"] | "";
    wifiPassword = wifi["password"] | "";
    staticIPLastOctet = wifi["lastOctet"] | 0;
    useStaticIP = wifi["useStatic"] | false;

    if (useStaticIP && wifiConnected) {
      IPAddress currentGW = WiFi.gatewayIP();
      staticIPPrefix = String(currentGW[0]) + "." + String(currentGW[1]) + "." +
                       String(currentGW[2]);
    } else {
      staticIPPrefix = "192.168.1";
    }

    log("WiFi config loaded: %s, %s, %s, %d, %s\n", wifiSSID.c_str(),
        wifiPassword.c_str(), staticIPPrefix.c_str(), staticIPLastOctet,
        useStaticIP ? "true" : "false");
  }

  // Load konfigurasi Motion
  if (doc.containsKey("motion")) {
    JsonObject motion = doc["motion"];
    motionCfg.junctionDev = motion["junctionDev"] | 0.5;
    motionCfg.minPlannerSpeed = motion["minPlannerSpeed"] | 0.0;
    motionCfg.enableInputShaping = motion["enableInputShaping"] | false;

    // Tutup jalur shaped selama reconfig agar ISR tak baca buffer
    // setengah-jadi; syncShaperBuffer() menerbitkan ulang di akhir.
    shaper_enabled = false;
  }

  // Load konfigurasi Axis
  for (int i = 0; i < 3; i++) {
    const char *axisNames[] = {"X", "Y", "Z"};
    if (doc.containsKey(axisNames[i])) {
      JsonObject axisObj = doc[axisNames[i]];
      loadAxisConfig(axisObj, i);
    }
  }

  // Load konfigurasi Tool
  if (doc.containsKey("tool")) {
    JsonObject tool = doc["tool"];
    toolCfg.pinPwm = tool["pinPwm"];
    toolCfg.pinToolOn = tool["pinToolOn"] | 255; // 255 = NC (Not Connected)
    toolCfg.pinZeroCross = tool["pinZeroCross"] | 22; // Default GPIO22
    toolCfg.scale = tool["scale"];
    toolCfg.scale *= 10;
    toolCfg.mode = tool["mode"];
    toolCfg.pwmMode = tool["pwmMode"] | 0;
    toolCfg.useZeroCross = tool["useZeroCross"] | false;
    toolCfg.invertToolOn = tool["invertToolOn"] | false;
  }

  // Load konfigurasi Homing
  if (doc.containsKey("homing")) {
    JsonObject homing = doc["homing"];
    homingCfg.pinEndstop = homing["pinEndstop"] | D0;
    homingCfg.homingSpeed = homing["homingSpeed"] | 100.0;
    homingCfg.homingPullback = homing["homingPullback"] | 2.0;
  }

  // Load konfigurasi LCD
  if (doc.containsKey("lcd")) {
    JsonObject lcd = doc["lcd"];
    lcd_sda = lcd["sda"];
    lcd_sda2 = lcd["sda2"];
    lcd_rst = lcd["rst"];
    lcd_contrast = lcd["contrast"];
    lcd_scl = lcd["scl"];
    lcd_addr = lcd["addr"];
    lcd_kind = lcd["kind"];
    lcd_cs = lcd["cs"];
    // check if TX or RX used then canUseSerial must be false
    canUseSerial = true;
    if (lcd_scl == TX || lcd_sda == TX || lcd_sda2 == TX || lcd_cs == TX ||
        lcd_scl == RX || lcd_sda == RX || lcd_sda2 == RX || lcd_cs == RX) {
      canUseSerial = false;
    }

    // Check Axes
    for (int i = 0; i < 3; i++) {
      if (cfg[i].pinStep == TX || cfg[i].pinStep == RX || cfg[i].pinDir == TX ||
          cfg[i].pinDir == RX) {
        canUseSerial = false;
      }
    }

    // Check tool
    if (toolCfg.pinPwm == TX || toolCfg.pinPwm == RX ||
        toolCfg.pinToolOn == TX || toolCfg.pinToolOn == RX ||
        toolCfg.pinZeroCross == TX || toolCfg.pinZeroCross == RX) {
      canUseSerial = false;
    }

    // Check homing
    if (homingCfg.pinEndstop == TX || homingCfg.pinEndstop == RX) {
      canUseSerial = false;
    }
  }

  // Load konfigurasi System
  if (doc.containsKey("system")) {
    JsonObject system = doc["system"];
    systemCfg.broadcastPositionWhenJobRunning =
        system["broadcastPositionWhenJobRunning"] | false;
    systemCfg.testLaserPower = system["testLaserPower"] | 255;
    systemCfg.testLaserTime = system["testLaserTime"] | 1;
    systemCfg.jogLimitEnabled = system["jogLimitEnabled"] | true;
  }

  // Load konfigurasi Karyacnc
  if (doc.containsKey("karyacnc")) {
    JsonObject karyacnc = doc["karyacnc"];
    karyacncIP = karyacnc["ip"] | "";
    karyacncToken = karyacnc["token"] | "";
    karyacncReportId = karyacnc["reportId"] | "";
  }

  // Load konfigurasi CNC Info
  if (doc.containsKey("cncInfo")) {
    JsonObject cncInfo = doc["cncInfo"];
    strncpy(cncName, cncInfo["name"] | "CNC Machine 1", 31);
    cncName[31] = '\0';
  } else {
    strncpy(cncName, "CNC Machine 1", 31);
    cncName[31] = '\0';
  }

  if (canUseSerial) {
    Serial.begin(115200);
  } else
    Serial.end();

  // Update pin global sesuai konfigurasi
  pinStep[0] = cfg[0].pinStep;
  pinDir[0] = cfg[0].pinDir;
  pinStep[1] = cfg[1].pinStep;
  pinDir[1] = cfg[1].pinDir;
  pinStep[2] = cfg[2].pinStep;
  pinDir[2] = cfg[2].pinDir;

  // Inisialisasi ulang shaper dan update pin
  initShaper(&shapers[0], &cfg[0], 0);
  initShaper(&shapers[1], &cfg[1], 1);
  initShaper(&shapers[2], &cfg[2], 2);

  syncShaperBuffer();

  // Update mode pin untuk pin baru dan hitung masker
  setupAxisPins();

  // Setup TOOLON pin jika diaktifkan
  log("Tool ON pin: %d\n", toolCfg.pinToolOn);
  if (toolCfg.pinPwm < 255) {
    pinMode(toolCfg.pinPwm, OUTPUT);
  }
  if (toolCfg.pinToolOn < 255) {
    pinMode(toolCfg.pinToolOn, OUTPUT);
    digitalWrite(toolCfg.pinToolOn,
                 toolCfg.invertToolOn ? HIGH : LOW); // Default off
  }

  // Setup Zero Cross pin jika diaktifkan
  if (toolCfg.useZeroCross && toolCfg.pinZeroCross < 255 &&
      toolCfg.pinZeroCross >= 0) {
    pinMode(toolCfg.pinZeroCross, INPUT_PULLUP);
#if defined(ESP8266)
    attachInterrupt(digitalPinToInterrupt(toolCfg.pinZeroCross), zeroCrossISR,
                    RISING);
#elif defined(ESP32)
    attachInterrupt(digitalPinToInterrupt(toolCfg.pinZeroCross), zeroCrossISR,
                    RISING);
#endif
    log("Zero Cross detection enabled on GPIO%d\n", toolCfg.pinZeroCross);
  }

  log("Init LCD\n");
  initUI();
  return true;
}

// -----------------------------------------------------------------------------
// Inisialisasi input shaper
// -----------------------------------------------------------------------------
void initShaper(AxisShaper *s, AxisConfig *cfg, int axis) {
  s->stepsPerMm = cfg->stepsPerMm;
  s->invStepsPerMm = 1.0f / cfg->stepsPerMm;
  s->numImpulses = 0;
  s->realStep = 0;
  s->stepPin = cfg->pinStep;
  s->stepMask = (s->stepPin < 255) ? (1 << s->stepPin) : 0;

  // Clear all impulses
  for (int i = 0; i < 4; i++) {
    s->impulses[i].weight = 0;
    s->impulses[i].delay = 0;
  }

  float wn = 2 * PI * cfg->freq;
  float dampSq = cfg->damp * cfg->damp;
  float wd = wn * sqrtf(1 - dampSq);
  float sqrtTerm = sqrtf(1 - dampSq);
  float K = exp((-cfg->damp * PI) / sqrtTerm);
  int16_t td_ticks = (int16_t)((PI / wd * 1000000) / TICK_MICROS);

  if (cfg->shaperType == 0) {
    // None - Tanpa input shaper
    s->numImpulses = 1;
    s->impulses[0].weight = SHAPER_SCALE;
    s->impulses[0].delay = 0;
  } else if (cfg->shaperType == 1) {
    // ZVD - Zero Vibration and Derivative
    float denom = (1 + K) * (1 + K) * (1 + K);
    float floatImpulses[4] = {1.0 / denom, (3 * K) / denom, (3 * K * K) / denom,
                              (K * K * K) / denom};

    s->impulses[0].weight = (int16_t)(floatImpulses[0] * SHAPER_SCALE);
    s->impulses[0].delay = 0;
    s->impulses[1].weight = (int16_t)(floatImpulses[1] * SHAPER_SCALE);
    s->impulses[1].delay = td_ticks;
    s->impulses[2].weight = (int16_t)(floatImpulses[2] * SHAPER_SCALE);
    s->impulses[2].delay = 2 * td_ticks;
    s->impulses[3].weight = (int16_t)(floatImpulses[3] * SHAPER_SCALE);
    s->impulses[3].delay = 3 * td_ticks;
    s->numImpulses = 4;
  } else if (cfg->shaperType == 2) {
    // MZV - Modified Zero Vibration
    float a = 1.0 / (4 * K * K + 1);
    float b = (2 * K) * a;
    float c = (2 * K * K) * a;

    s->impulses[0].weight = (int16_t)(a * SHAPER_SCALE);
    s->impulses[0].delay = 0;
    s->impulses[1].weight = (int16_t)(b * SHAPER_SCALE);
    s->impulses[1].delay = td_ticks;
    s->impulses[2].weight = (int16_t)(c * SHAPER_SCALE);
    s->impulses[2].delay = 2 * td_ticks;
    s->impulses[3].weight = (int16_t)(c * SHAPER_SCALE);
    s->impulses[3].delay = 3 * td_ticks;
    s->numImpulses = 4;
  } else if (cfg->shaperType == 3) {
    // EI - Extra-Insensitive
    float A = 1.0 - exp(-cfg->damp * PI / sqrtTerm);
    float denom = 1 +
                  2 * K * exp(-cfg->damp * PI * (2 * PI - 1) / (2 * sqrtTerm)) +
                  K * K;
    float floatImpulses[4] = {
        A / denom,
        (A * K * exp(-cfg->damp * PI * (2 * PI - 1) / (2 * sqrtTerm))) / denom,
        (A * K * exp(-cfg->damp * PI * (2 * PI - 1) / (2 * sqrtTerm))) / denom,
        (A * K * K) / denom};

    s->impulses[0].weight = (int16_t)(floatImpulses[0] * SHAPER_SCALE);
    s->impulses[0].delay = 0;
    s->impulses[1].weight = (int16_t)(floatImpulses[1] * SHAPER_SCALE);
    s->impulses[1].delay = td_ticks;
    s->impulses[2].weight = (int16_t)(floatImpulses[2] * SHAPER_SCALE);
    s->impulses[2].delay = 2 * td_ticks;
    s->impulses[3].weight = (int16_t)(floatImpulses[3] * SHAPER_SCALE);
    s->impulses[3].delay = 3 * td_ticks;
    s->numImpulses = 4;
  }

  // Normalize weights to sum to SHAPER_SCALE
  int sum = 0;
  int nsum = 0;
  for (int i = 0; i < s->numImpulses; i++) {
    sum += s->impulses[i].weight;
  }
  if (sum == 0)
    sum = 1;
  for (int i = 0; i < s->numImpulses; i++) {
    s->impulses[i].weight = (s->impulses[i].weight * SHAPER_SCALE) / sum;
    nsum += s->impulses[i].weight;
  }
  s->impulses[0].weight += SHAPER_SCALE - nsum;

  // Check total delay against buffer size
  uint32_t totalDelay = s->impulses[s->numImpulses - 1].delay;
  uint32_t n = SHAPER_BUFFER_SIZE - 3;
  log("Inisialisasi shaper axis %d, tipe %d\n", axis, cfg->shaperType);
  log("Freq %f damp %f\n", cfg->freq, cfg->damp);
  if (totalDelay > n) {
    log("Resizing shaper delay from %d to %d\n", totalDelay, n);
    for (int i = 0; i < s->numImpulses; i++) {
      s->impulses[i].delay =
          (int16_t)((float)s->impulses[i].delay * n / totalDelay);
    }
  } else
    n = totalDelay;

  s->maxDelayTicks = n;

  for (int i = 0; i < s->numImpulses; i++) {
    log("Impulse %d weight %d, delay %d\n", i, s->impulses[i].weight,
        s->impulses[i].delay);
  }
}

// -----------------------------------------------------------------------------
// Setup Axis Pins
// -----------------------------------------------------------------------------
void setupAxisPins() {
  log("Setup axis pins\n");
  for (int i = 0; i < 3; i++) {
    if (cfg[i].pinStep < 255)
      pinMode(cfg[i].pinStep, OUTPUT);
    if (cfg[i].pinDir < 255)
      pinMode(cfg[i].pinDir, OUTPUT);
  }

  maskDir[0] = (pinDir[0] < 255) ? (1 << pinDir[0]) : 0;
  maskDir[1] = (pinDir[1] < 255) ? (1 << pinDir[1]) : 0;
  maskDir[2] = (pinDir[2] < 255) ? (1 << pinDir[2]) : 0;

  if (toolCfg.pinPwm < 255) {
    maskToolPwm = (1 << toolCfg.pinPwm);
  } else {
    maskToolPwm = 0;
  }

  // Setup mask untuk TOOLON pin jika diaktifkan
  if (toolCfg.pinToolOn < 255) {
    maskToolOn = (1 << toolCfg.pinToolOn);
  } else {
    maskToolOn = 0;
  }
  allStepMask = shapers[0].stepMask | shapers[1].stepMask | shapers[2].stepMask;  
}

// =============================================================================
// SECTION 10: NETWORK COMMUNICATION
// =============================================================================

// -----------------------------------------------------------------------------
// URL Encode Helper
// -----------------------------------------------------------------------------
String urlEncode(const String &str) {
  String encoded = "";
  char c;
  for (size_t i = 0; i < str.length(); i++) {
    c = str.charAt(i);
    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      encoded += c;
    } else if (c == ' ') {
      encoded += '+';
    } else {
      char buf[5];
      sprintf(buf, "%%%02X", (unsigned char)c);
      encoded += buf;
    }
  }
  return encoded;
}
// -----------------------------------------------------------------------------
// Auto Delete/Rename Job File
// Jika nama file diawali dengan "_", kurangi satu "_" saat job selesai.
// Jika setelah dikurangi tidak ada "_" lagi, hapus file.
// -----------------------------------------------------------------------------
void autoDeleteJobFile(const char *jobName) {
  String filename = String(jobName);

  // Cek apakah nama file diawali dengan "_"
  if (!filename.startsWith("_")) {
    return;
  }

  String oldPath = "/" + filename;

  // Hitung berapa banyak "_" di awal
  int underscoreCount = 0;
  for (int i = 0; i < filename.length(); i++) {
    if (filename.charAt(i) == '_') {
      underscoreCount++;
    } else {
      break;
    }
  }

  // Kurangi satu "_" dari awal
  String newFilename = filename.substring(1);
  String newPath = "/" + newFilename;

  // Cek apakah masih ada "_" di awal nama baru
  if (newFilename.startsWith("_")) {
    // Masih ada "_", rename file
    if (LittleFS.rename(oldPath, newPath)) {
      log("Auto-rename: %s -> %s\n", filename.c_str(), newFilename.c_str());
    } else {
      log("Auto-rename gagal: %s\n", filename.c_str());
    }
  } else {
    // Tidak ada "_" lagi, hapus file
    if (LittleFS.remove(oldPath)) {
      log("Auto-delete: %s (dihapus setelah selesai)\n", filename.c_str());
    } else {
      log("Auto-delete gagal: %s\n", filename.c_str());
    }
  }
}

// -----------------------------------------------------------------------------
// Report Job Completion
// -----------------------------------------------------------------------------
void reportJobComplete(const char *jobName, bool success) {
  if (karyacncIP.length() == 0) {
    return;
  }

  HTTPClient http;
  String url = "http://" + karyacncIP + ":8888/t";

  String message = String(cncName) + " - finish job " + String(jobName);

  String encodedBot = urlEncode(karyacncToken);
  String encodedId = urlEncode(karyacncReportId);
  String encodedMsg = urlEncode(message);

  url += "?bot=" + encodedBot + "&id=" + encodedId + "&t=" + encodedMsg;

  http.begin(karyacncClient, url);
  int httpCode = http.GET();
  http.end();
}

// -----------------------------------------------------------------------------
// Connect to WiFi
// -----------------------------------------------------------------------------
extern void d_text(int x, int y, String s);
extern void d_show();
extern void d_clear();

bool connectToWiFi() {
  if (wifiSSID.length() == 0) {
    return false;
  }

  log("Menghubungkan ke WiFi: %s %s\n", wifiSSID.c_str(), wifiPassword.c_str());

  if (useStaticIP && staticIPPrefix.length() > 0 && staticIPLastOctet > 0) {
    IPAddress staticIP;

    int a, b, c;
    if (sscanf(staticIPPrefix.c_str(), "%d.%d.%d", &a, &b, &c) == 3) {
      staticIP = IPAddress(a, b, c, staticIPLastOctet);
      WiFi.config(staticIP, gateway, subnet, dns);
      log("Menggunakan IP statis: %d.%d.%d.%d\n", a, b, c, staticIPLastOctet);
    }
  }

  WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
#ifdef ESP8266
  WiFi.setSleepMode(WIFI_LIGHT_SLEEP);
#elif defined(ESP32)
  // Disable WiFi power saving on ESP32 for better network performance
  WiFi.setSleep(false);
#endif
  int attempts = 0;
  d_clear();
  d_text(0, 0, "Connecting to WiFi...");
  d_text(0, 1, wifiSSID);
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    d_text(attempts, 2, ".");
    d_show();
    delay(500);
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    log("\nWiFi Terhubung!\n");
    log("IP: %s\n", WiFi.localIP().toString().c_str());
    wifiConnected = true;
    isAccessPoint = false;
    return true;
  } else {
    log("\nKoneksi WiFi gagal\n");
    wifiConnected = false;
    return false;
  }
}

// -----------------------------------------------------------------------------
// Chunked File Streaming — isi step buffer di antara tiap chunk
// Mencegah step event buffer habis saat serve file besar (index.html ~97KB)
// -----------------------------------------------------------------------------
// Deklarasikan __yield bawaan sistem core ESP8266
extern "C" void __yield();
void streamFileChunked(File &file, const String &contentType,
                       bool isGzip = false) {
  const size_t CHUNK_SIZE =
      512; // byte per chunk — cukup kecil agar motionloop bisa masuk
  static uint8_t chunkBuf[CHUNK_SIZE];

  size_t fileSize = file.size();

  // Siapkan response header manual
  WiFiClient client = server.client();
  client.print(F("HTTP/1.1 200 OK\r\n"));
  client.print(F("Content-Type: "));
  client.print(contentType);
  client.print(F("\r\n"));
  client.print(F("Content-Length: "));
  client.print(fileSize);
  client.print(F("\r\n"));
  if (isGzip)
    client.print(F("Content-Encoding: gzip\r\n"));
  client.print(F("Connection: close\r\n"));
  client.print(F("\r\n"));

  // Stream chunk per chunk, isi step buffer di antara tiap chunk
  while (file.available()) {
    size_t toRead = min((size_t)file.available(), CHUNK_SIZE);
    size_t bytesRead = file.read(chunkBuf, toRead);
    if (bytesRead == 0)
      break;
    client.write(chunkBuf, bytesRead);

    // Isi step event buffer agar ISR stepper tidak kehabisan

    processJob();
    yield();
  }
}

// -----------------------------------------------------------------------------
// Setup Web Server Routes
// -----------------------------------------------------------------------------

// Toggle + persist soft-limit jog (dipakai Web UI dan menu LCD "Jog Limit")
bool setJogLimitEnabled(bool en) {
  systemCfg.jogLimitEnabled = en;
  // Read-modify-write /config.json agar field lain tidak hilang
  DynamicJsonDocument doc(CONFIG_DOC_SIZE);
  File f = LittleFS.open("/config.json", "r");
  if (f) {
    deserializeJson(doc, f);
    f.close();
  }
  doc["system"]["jogLimitEnabled"] = en;
  File w = LittleFS.open("/config.json", "w");
  if (!w)
    return false;
  serializeJson(doc, w);
  w.close();
  log("Jog limit %s (tersimpan)\n", en ? "ON" : "OFF");
  return true;
}

void setupRoutes() {
  server.onNotFound([]() {
    String path = server.uri();
    if (path == "/")
      path = "/index.html";
    String gzPath = path + ".gz";
    if (LittleFS.exists(path + ".html.gz")) {
      gzPath = path + ".html.gz";
      path = path + ".html";
    } else if (LittleFS.exists(path + ".html")) {
      gzPath = path + ".html.gz";
      path = path + ".html";
    }
    if (LittleFS.exists(gzPath) && LittleFS.exists(path)) {
      LittleFS.remove(path);
    }
    if (LittleFS.exists(gzPath) || LittleFS.exists(path)) {
      bool isGzip = LittleFS.exists(gzPath);
      File file = LittleFS.open(isGzip ? gzPath : path, "r");

      String contentType = "text/plain";
      if (path.endsWith(".html"))
        contentType = "text/html";
      else if (path.endsWith(".js"))
        contentType = "application/javascript";
      else if (path.endsWith(".css"))
        contentType = "text/css";

      // Gunakan chunked streaming untuk file besar (index.html ~97KB)
      // agar motionloop() bisa berjalan di antara chunk dan step buffer tidak
      // habis.
      streamFileChunked(file, contentType, isGzip);
      file.close();

    } else {
      server.send(404, "text/plain", "Not Found");
    }
  });

  // GET status jog limit
  server.on("/api/joglimit", HTTP_GET, []() {
    String out = "{\"jogLimitEnabled\":" +
                 String(systemCfg.jogLimitEnabled ? "true" : "false") + "}";
    server.send(200, "application/json", out);
  });

  // POST {"jogLimitEnabled":true|false}
  server.on("/api/joglimit", HTTP_POST, []() {
    if (jobRunning) {
      server.send(423, "text/plain", "Job sedang berjalan");
      return;
    }
    if (!server.hasArg("plain")) {
      server.send(400, "text/plain", "No JSON data");
      return;
    }
    DynamicJsonDocument doc(256);
    if (deserializeJson(doc, server.arg("plain"))) {
      server.send(400, "text/plain", "JSON tidak valid");
      return;
    }
    bool en = doc["jogLimitEnabled"] | !systemCfg.jogLimitEnabled;
    if (setJogLimitEnabled(en))
      server.send(200, "application/json",
                  "{\"jogLimitEnabled\":" +
                      String(systemCfg.jogLimitEnabled ? "true" : "false") +
                      "}");
    else
      server.send(500, "text/plain", "Failed to save config");
  });

  // GET konfigurasi
  server.on("/api/config", HTTP_GET, []() {
    File configFile = LittleFS.open("/config.json", "r");
    if (configFile) {
      streamFileChunked(configFile, "application/json");
      configFile.close();
    } else {
      server.send(404, "text/plain", "404: Not Found");
    }
  });

  // POST konfigurasi
  server.on("/api/config", HTTP_POST, []() {
    if (jobRunning) {
      server.send(423, "text/plain", "Job sedang berjalan");
      return;
    }
    log("Simpan konfigurasi mulai\n");
    if (server.hasArg("plain")) {
      File configFile = LittleFS.open("/config.json", "w");
      if (configFile) {
        configFile.print(server.arg("plain"));
        configFile.close();
        log("Konfigurasi disimpan ke file\n");
        loadAllConfig();
        updateZTrimTiming();
        server.send(200, "text/plain", "OK");
      } else {
        server.send(500, "text/plain", "Failed to save config");
      }
    } else {
      server.send(400, "text/plain", "No JSON data");
    }
    log("Simpan konfigurasi selesai\n");
  });

  // GET daftar file
  server.on("/api/files", HTTP_GET, []() {
    String output = "";

#if defined(ESP8266)
    Dir dir = LittleFS.openDir("/");
    while (dir.next()) {
      if (!dir.isDirectory()) {
        String filename = dir.fileName();
        if (filename.endsWith(".gcode") || filename.endsWith(".nc") ||
            filename.endsWith(".txt") || filename.endsWith(".gx")) {
          output += filename + "," + String(dir.fileSize()) + "\n";
        }
      }
    }
#elif defined(ESP32)
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while(file){
      if (!file.isDirectory()) {
        String filename = file.name();
        if (filename.endsWith(".gcode") || filename.endsWith(".nc") || filename.endsWith(".txt") || filename.endsWith(".gx")) {
          output += filename + "," + String(file.size()) + "\n";
        }
      }
      file = root.openNextFile();
    }
#endif

    server.send(200, "text/plain", output);
  });

  // Halaman upload
  server.on("/upload", HTTP_GET,
            []() { server.send(200, "text/html", uploadHtml); });

  // Upload file
  server.on(
      "/upload", HTTP_POST,
      []() {
        bool noReport =
            server.hasArg("noReport") && server.arg("noReport") == "1";

        if (!noReport) {
          IPAddress clientIP = server.client().remoteIP();
          char newIP[16];
          snprintf(newIP, sizeof(newIP), "%d.%d.%d.%d", clientIP[0],
                   clientIP[1], clientIP[2], clientIP[3]);

          if (strcmp(newIP, karyacncIP.c_str()) != 0 &&
              strcmp(newIP, "0.0.0.0") != 0) {
            karyacncIP = String(newIP);
            log("[AUTO] Karyacnc IP set to: %s\n", newIP);
          }
        } else {
          log("[AUTO] Skipping karyacnc IP update (noReport flag set)\n");
        }

        server.send(200, "text/plain", "File berhasil diupload");
      },
      []() {
        HTTPUpload &upload = server.upload();

        if (upload.status == UPLOAD_FILE_START) {
          if (jobRunning) {
            log("Upload ditolak: job sedang berjalan\n");
            server.send(423, "text/plain", "Job sedang berjalan");
            return;
          }
          log("Upload dimulai: %s\n", upload.filename.c_str());
          fileU = LittleFS.open("/" + upload.filename, "w");
        } else if (upload.status == UPLOAD_FILE_WRITE) {
          if (fileU) {
            fileU.write(upload.buf, upload.currentSize);
          }
        } else if (upload.status == UPLOAD_FILE_END) {
          fileU.close();
          log("Upload selesai: %s\n", upload.filename.c_str());
        }
      });

  // Delete file
  server.on("/api/delete", HTTP_POST, []() {
    if (server.hasArg("file")) {
      String filename = server.arg("file");
      String filepath = "/" + filename;
      LittleFS.remove(filepath);
      server.send(200, "text/plain", "OK");
    }
  });

  // Mulai job
  server.on("/api/job/start", HTTP_POST, []() {
    if (server.hasArg("file") && !jobRunning) {
      touchLcdActivity(); // aktivitas web (anti idle-watchdog)
      startJob(server.arg("file"));
      server.send(200, "text/plain", "Job dimulai");
    } else {
      server.send(400, "text/plain", "Parameter file tidak ada");
    }
  });

  server.on("/startjob", HTTP_GET, []() {
    touchLcdActivity(); // aktivitas web (anti idle-watchdog)
    startJob(server.arg("jobname"));
    server.send(200, "text/html", "Start Ok");
  });

  server.on("/getfree", HTTP_GET, []() {
#if defined(ESP32)
    size_t tBytes = LittleFS.totalBytes();
    size_t uBytes = LittleFS.usedBytes();
#else
      FSInfo fs_info;
      LittleFS.info(fs_info);
      size_t tBytes = fs_info.totalBytes;
      size_t uBytes = fs_info.usedBytes;
#endif
    server.send(200, "text/plain",
                "[" + String(tBytes) + "," + String(uBytes) + "]");
  });

  // Preview job
  server.on("/api/job/preview", HTTP_POST, []() {
    if (server.hasArg("file")) {
      server.send(200, "text/plain", "Preview dimulai");
      previewJob(server.arg("file"), true);
    } else {
      server.send(400, "text/plain", "Parameter file tidak ada");
    }
  });
}

// -----------------------------------------------------------------------------
// Broadcast Position History via WebSocket (BINARY FORMAT)
// -----------------------------------------------------------------------------
void broadcastPositionHistory() {
  if (systemCfg.broadcastPositionWhenJobRunning) {

    uint8_t buffer[POSITION_HISTORY_SIZE * 16 + 10];
    uint16_t idx = 0;

    uint32_t count = positionHistoryIndex;

    buffer[idx++] = 0x01;
    buffer[idx++] =
        systemCfg.broadcastPositionWhenJobRunning ? (uint8_t)count : 0;

    if (systemCfg.broadcastPositionWhenJobRunning) {
      for (int i = 0; i < count; i++) {
        int32_t x = positionHistory[i].X;
        int32_t y = positionHistory[i].Y;
        int32_t z = positionHistory[i].Z;
        int32_t l = positionHistory[i].l;

        buffer[idx++] = x & 0xFF;
        buffer[idx++] = (x >> 8) & 0xFF;
        buffer[idx++] = (x >> 16) & 0xFF;
        buffer[idx++] = (x >> 24) & 0xFF;

        buffer[idx++] = y & 0xFF;
        buffer[idx++] = (y >> 8) & 0xFF;
        buffer[idx++] = (y >> 16) & 0xFF;
        buffer[idx++] = (y >> 24) & 0xFF;

        buffer[idx++] = z & 0xFF;
        buffer[idx++] = (z >> 8) & 0xFF;
        buffer[idx++] = (z >> 16) & 0xFF;
        buffer[idx++] = (z >> 24) & 0xFF;

        buffer[idx++] = l & 0xFF;
        buffer[idx++] = (l >> 8) & 0xFF;
        buffer[idx++] = (l >> 16) & 0xFF;
        buffer[idx++] = (l >> 24) & 0xFF;
      }
    }
    buffer[idx++] = jobRunning ? 1 : 0;
    buffer[idx++] = jobPaused ? 1 : 0;

    if (jobFileSize > 0) {
      progressPercent = (int)((jobBytesRead * 100) / jobFileSize);
      if (progressPercent > 100)
        progressPercent = 100;
    } else {
      progressPercent = 0;
    }

    buffer[idx++] = (uint8_t)progressPercent;

    uint8_t nameLen = (uint8_t)strlen(currentJobFilename.c_str());
    buffer[idx++] = nameLen;
    if (nameLen > 0) {
      memcpy(buffer + idx, currentJobFilename.c_str(), nameLen);
      idx += nameLen;
    }

    // === Buffer Health ===
    // sebHealth: seberapa penuh step event buffer (100% = penuh = sehat)
    {
      uint32_t used = seblength; // dihitung on-demand dari (sebhead - sebtail)
      uint8_t sebHealth = (uint8_t)((used * 100) / STEP_EVENT_BUFFER_SIZE);
      buffer[idx++] = sebHealth;
    }
    // moveHealth: seberapa penuh move buffer (100% = penuh = sehat)
    {
      uint32_t used =
          bufflength; // makro: (head-tail+NUMBUFFER) & (NUMBUFFER-1)
      uint8_t moveHealth = (uint8_t)((used * 100) / NUMBUFFER);
      buffer[idx++] = moveHealth;
    }
    if (idx > 0) {
      webSocket.broadcastBIN(buffer, idx);
    }
  }
  positionHistoryIndex = 0; // Reset early to allow Core 1 to continue filling
}

// -----------------------------------------------------------------------------
// Record Current Position to History
// -----------------------------------------------------------------------------
void recordPosition() {
  if (positionHistoryIndex < POSITION_HISTORY_SIZE) {
    if (systemCfg.broadcastPositionWhenJobRunning) {
      positionHistory[positionHistoryIndex].X =
          (int32_t)(shapers[0].realStep * 100.0 * shapers[0].invStepsPerMm);
      positionHistory[positionHistoryIndex].Y =
          (int32_t)(shapers[1].realStep * 100.0 * shapers[1].invStepsPerMm);
      positionHistory[positionHistoryIndex].Z =
          (int32_t)(shapers[2].realStep * 100.0 * shapers[2].invStepsPerMm);
      positionHistory[positionHistoryIndex].l = pwmValue;
    }
    positionHistoryIndex++;
  }
}

// -----------------------------------------------------------------------------
// WebSocket Event Handler
// -----------------------------------------------------------------------------
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload,
                    size_t length) {
  switch (type) {
  case WStype_DISCONNECTED:
    break;
  case WStype_CONNECTED:
    break;
  case WStype_TEXT:
    break;
  case WStype_BIN:
    touchLcdActivity(); // perintah web apapun = aktivitas (anti idle-watchdog)
    if (length >= 1) {
      uint8_t cmd = payload[0];
      switch (cmd) {
      case 0x10: {
        if (length >= 13) {
          float distX, distY, distZ;
          memcpy(&distX, payload + 1, 4);
          memcpy(&distY, payload + 5, 4);
          memcpy(&distZ, payload + 9, 4);
          if (!jobRunning) {
            jogMulti(distX, distY, distZ);
          }
        }
        break;
      }
      case 0x11:
        remotePause();
        break;
      case 0x12:
        remoteResume();
        break;
      case 0x13:
        stopJob();
        break;
      case 0x18:
        // Jog berhenti (tombol dilepas) — buang sisa antrean jog agar mesin
        // berhenti responsif, tidak "jalan sendiri" dari backlog.
        if (!jobRunning)
          flushJogBuffer();
        break;
      case 0x14: {
        if (length >= 3) {
          int16_t speed;
          memcpy(&speed, payload + 1, 2);
          setGlobalSpeed(speed);
        }
        break;
      }
      case 0x15: {
        if (!jobRunning) {
          if (toolCfg.mode == 0) {
            if (length >= 2) {
              uint8_t power = payload[1];
              setToolPower(power);
            } else {
              setToolPower(pwmValue > 0 ? 0 : 255);
            }
          } else {
            testLaser();
          }
        }
        break;
      }
      case 0x16:
        resetAll();
        break;
      case 0x17: {
        if (length >= 2 && !isHoming) {
          uint8_t axis = payload[1];
          if (axis <= 3) {
            startHoming(axis);
          }
        }
        break;
      }
      case 0x20: {
        uint8_t response[3];
        response[0] = 0xA0;
        int16_t speed = getGlobalSpeed();
        memcpy(response + 1, &speed, 2);
        webSocket.sendBIN(num, response, 3);
        break;
      }
      case 0x21: {
        if (jobFileSize > 0) {
          progressPercent = (int)((jobBytesRead * 100) / jobFileSize);
          if (progressPercent > 100)
            progressPercent = 100;
        } else {
          progressPercent = 0;
        }

        uint8_t nameLen = (uint8_t)strlen(currentJobFilename.c_str());
        uint8_t response[7 + nameLen];
        uint16_t idx = 0;
        response[idx++] = 0xA1;
        response[idx++] = jobRunning ? 1 : 0;
        response[idx++] = jobPaused ? 1 : 0;
        memcpy(response + idx, currentJobFilename.c_str(), nameLen);
        idx += nameLen;
        response[idx++] = nameLen;
        memcpy(response + idx, &jobBytesRead, 4);
        idx += 4;
        memcpy(response + idx, &jobFileSize, 4);
        idx += 4;
        response[idx++] = progressPercent;
        webSocket.sendBIN(num, response, idx);
        break;
      }
      }
    }
    break;
  }
}

// =============================================================================
// SECTION 12: SETUP & LOOP
// =============================================================================

#if defined(ESP32)
// -----------------------------------------------------------------------------
// Task untuk Core 0 - Menangani Network, Parser, dan UI
// -----------------------------------------------------------------------------
void webTask(void *pvParameters) {
  for (;;) {

    // Handle Network
    server.handleClient();
    webSocket.loop();
    ArduinoOTA.handle();
    if (pendingPositionBroadcast) {
      broadcastPositionHistory();
      pendingPositionBroadcast = false;
    }
    vTaskDelay(1);
  }
}
#endif

// -----------------------------------------------------------------------------
// Setup awal
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  // canUseSerial ditentukan loadAllConfig() berdasarkan pemakaian pin TX/RX
  // (LCD memakai GPIO1/GPIO3 → Serial.end() dipanggil). Default false mencegah
  // log()/log0() menulis ke TX sebelum config termuat.

  currentToolPower = 0;

  log("Memulai filesystem...\n");
#if defined(ESP8266)
  if (!LittleFS.begin()) {
    log("Gagal memulai LittleFS, mencoba format...\n");
    LittleFS.format();
    LittleFS.begin();
  }
#elif defined(ESP32)
  if (!LittleFS.begin(true)) {
    log("Gagal memulai LittleFS, mencoba format...\n");
    LittleFS.format();
    LittleFS.begin(true);
  }
#endif

  log("Load config\n");
  loadAllConfig();
  updateZTrimTiming(); // interval drain trim mengikuti config Z

  setupEndstopPin();
  log("Endstop pin: %d\n", homingCfg.pinEndstop);

  log("Configured SSID: %s\n", wifiSSID.c_str());
  log("Use Static IP: %s\n", useStaticIP ? "Yes" : "No");

  if (connectToWiFi()) {
    log("Mode: WiFi Station\n");
    log("Station IP: %s\n", WiFi.localIP().toString().c_str());

    IPAddress ip = WiFi.localIP();
    sprintf(cncIP, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
  } else {
    log("WiFi connection failed, starting AP mode\n");
    WiFi.mode(WIFI_AP);
#if defined(ESP8266)
    WiFi.softAP(cncName, apPassword);
#elif defined(ESP32)
    WiFi.softAP(cncName, apPassword);
#endif
    isAccessPoint = true;
    log("AP Dimulai: %s\n", cncName);
    log("AP IP: %s\n", WiFi.softAPIP().toString().c_str());

    IPAddress ip = WiFi.softAPIP();
    sprintf(cncIP, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
  }

  // Sinkronisasi NTP untuk timestamp logjob
  if (!isAccessPoint) {
    if (bootEpoch == 0)
      log("NTP sync gagal, pakai millis()\n");
  }

  log("CNC IP: %s, Name: %s\n", cncIP, cncName);

  setupRoutes();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Setup OTA
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {
      type = "filesystem";
    }
    log("Start updating %s\n", type.c_str());
  });
  ArduinoOTA.onEnd([]() { log("\nEnd"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {});
  ArduinoOTA.onError([](ota_error_t error) {
    log("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      log("Auth Failed\n");
    } else if (error == OTA_BEGIN_ERROR) {
      log("Begin Failed\n");
    } else if (error == OTA_CONNECT_ERROR) {
      log("Connect Failed\n");
    } else if (error == OTA_RECEIVE_ERROR) {
      log("Receive Failed\n");
    } else if (error == OTA_END_ERROR) {
      log("End Failed\n");
    }
  });
  ArduinoOTA.begin();

  resetAll();
  initUI();
  server.begin();
  log("HTTP Server Dimulai on port 80\n");

#if defined(ESP32)
  // Buat task untuk Core 0 sebelum timer dimulai
  xTaskCreatePinnedToCore(webTask, "SystemTask", 8192, NULL, 1, NULL,
                          0 // Core 0
  );
#endif

#if defined(ESP8266)
  timer1_attachInterrupt(shaperISR);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
  timer1_write(TICK_MICROS * 5);
#elif defined(ESP32)
  hw_timer_t *timer = NULL;
  timer = timerBegin(1000000 / TICK_MICROS);
  timerAttachInterrupt(timer, &shaperISR);
  timerAlarm(timer, 1, true, 0);
#endif
  log("Timer started\n");
  // Cek apakah ada saved state dari power-fail
  if (loadPowerFailState()) {
    log("!!! POWER-FAIL STATE DETECTED - Auto Resume !!!\n");
    log("Saved steps: X=%ld Y=%ld Z=%ld\n", resumeState.realStepX,
        resumeState.realStepY, resumeState.realStepZ);
  }

  // Apply resume state jika perlu
  bool powerfailRestored =
      false; // headpos jangan menimpa restore powerfail
  if (needResume) {
    powerfailRestored = true;
    log("Auto-resuming position...\n");
    // Set posisi mesin ke step count tersimpan (tanpa gerakan fisik)
    shapers[0].realStep = resumeState.realStepX;
    shapers[1].realStep = resumeState.realStepY;
    shapers[2].realStep = resumeState.realStepZ;
    updateRealPos();

    // Update job start position juga
    JobX = resumeState.jobX;
    JobY = resumeState.jobY;
    JobZ = resumeState.jobZ;

    // Catatan: trim Z TIDAK disimpan saat mati PLN (klik & lupakan) —
    // setelah restart trim kembali 0. Sisa step tidak dipulihkan.
    log("Position restored: X=%.3f Y=%.3f Z=%.3f\n", cx1, cy1, cz1);

    // Reset flag
    needResume = false;
    resumeState.valid = false;
  }

  // Rescue posisi head setelah restart paksa: kalau ada /headpos.bin,
  // posisi boot = posisi tersimpan — tidak dianggap 0,0,0 lagi, jadi mudah
  // jog pulang. File DIPERTAHANKAN; hapus/timpa hanya via Set Zero atau
  // save berikutnya.
  if (!powerfailRestored && LittleFS.exists(HEADPOS_FILE)) {
    File f = LittleFS.open(HEADPOS_FILE, "r");
    if (f && f.size() >= 12) {
      int32_t realSteps[3];
      if (f.read((uint8_t *)realSteps, sizeof(realSteps)) ==
          sizeof(realSteps)) {
        shapers[0].realStep = realSteps[0];
        shapers[1].realStep = realSteps[1];
        shapers[2].realStep = realSteps[2];
        updateRealPos();
        log("Head position restored: X=%.3f Y=%.3f Z=%.3f\n", cx1, cy1, cz1);
      }
    }
    if (f)
      f.close();
  }
}

// -----------------------------------------------------------------------------
// Loop utama
// -----------------------------------------------------------------------------
void coreloop() {
  static uint32_t lastWiFiYield = 0;
  static uint32_t lastWiFiHandle = 0;
  static uint32_t lastHeadPosSaveMs =
      0; // throttle simpan posisi head periodik

#if defined(ESP8266)
  if (!jobRunning)
    ArduinoOTA.handle();
  uint32_t now = millis();
  if (now - lastWiFiHandle >= 20) {
    lastWiFiHandle = now;
    uint32_t sebFree = STEP_EVENT_BUFFER_SIZE - seblength;
    // Hanya handle client saat buffer step event aman:
    // sebFree < 25% = hanya 25% slot kosong = 75% terisi event = tersedia ~24ms
    // cadangan Jika sedang tidak running / paused, selalu handle (tidak ada
    // stepper yang butuh event)
    bool bufferSafe = (sebFree < (STEP_EVENT_BUFFER_SIZE >> 2));
    if (bufferSafe || !jobRunning || jobPaused || stopping) {
      // Isi buffer semaksimal mungkin SEBELUM handleClient blocking
      yield();
      server.handleClient();
      webSocket.loop();
      if (pendingPositionBroadcast) {
        broadcastPositionHistory();
        pendingPositionBroadcast = false;
      }
    }
  }
#endif

  // Record position periodically (pakai millis() - absTime bisa kelaparan
  // saat step rate tinggi sehingga ui_loop() tidak pernah dipanggil)
  uint32_t nowMs = millis();
  if (nowMs - lastRecordTimeMs > 50) {
    lastRecordTimeMs = nowMs;
    recordPosition();
    ui_loop();
    if (positionHistoryIndex >= POSITION_HISTORY_SIZE) {
      pendingPositionBroadcast = true;
    }
  }

  /* MONITORING PWM QUEUE - Uncomment jika perlu tracking
  if (pwmQueueOverflow) {
    pwmQueueOverflow = false;
    log("PWM OVERFLOW! Count: %lu\n", pwmOverflowCount);
  }

  // Tampilkan statistik setiap 10 detik
  static uint32_t lastPwmStatsTime = 0;
  if (millis() - lastPwmStatsTime > 10000) {
    lastPwmStatsTime = millis();
    if (pwmMaxUsage > 0 || pwmOverflowCount > 0) {
      log("PWM Stats - Max: %d/%d (%.1f%%), Overflow: %lu\n",
          pwmMaxUsage, PWM_TOKEN_QUEUE_SIZE,
          (pwmMaxUsage * 100.0) / PWM_TOKEN_QUEUE_SIZE,
          pwmOverflowCount);
      pwmMaxUsage = 0;  // Reset untuk session berikutnya
    }
  }
  */

  {
    // Konsumen reset LCD deferred — eksekusi HANYA saat:
    // (1) flag pending, (2) tidak sedang homing (transfer SPI bisa mengganggu
    // state machine), (3) motor diam tenang >= LCD_HEAL_QUIET_MS (spindle AC
    // baru berhenti total beberapa detik pasca-motion; getaran mengganggu
    // heal). Transfer bit-bang SPI full-frame memblokir ISR puluhan ms —
    // bila dilakukan saat buffer berisi, motor freeze mendadak lalu menghentak.
    if (UNLIKELY(lcdResetPending) && !isHoming &&
        (nowMs - lastMotionSeen) >= LCD_HEAL_QUIET_MS) {
      lcdResetPending = false;
      lastMotionSeen = millis();
      lcdForceFullRedraw();
      saveHeadPosition(); // catat posisi head saat pasti diam (rescue setelah
                          // restart)
    }
  }
  // Simpan posisi head periodik selama job — kalau ESP freeze & direstart di
  // tengah job, posisi terakhir diketahui. (saveHeadPosition dinonaktifkan)
  // if (jobRunning && nowMs - lastHeadPosSaveMs > HEADPOS_SAVE_MS) {
  //   lastHeadPosSaveMs = nowMs;
  //   saveHeadPosition();
  // }

  processPauseSequence(); // park/restore saat pause (Z → Z0, dst.)
  motionloop();
}

void loop() {
  // Deteksi mati PLN - cek jika zero-cross tidak terdeteksi dalam waktu
  // tertentu
  if (toolCfg.useZeroCross && !powerFailDetected && !stateSaved) {
    int32_t elapsedSinceZC = millis() - lastZeroCrossTimeMs;

    if (elapsedSinceZC > POWER_FAIL_THRESHOLD_MS && lastZeroCrossTimeMs > 0) {
      powerFailDetected = true;
      log("POWER FAIL DETECTED! Elapsed since ZC: %d ms\n", elapsedSinceZC);

      // Simpan state sebelum mati
      savePowerFailState();
    }
  }

  processJob();
  coreloop();

#if defined(ESP32)
  vTaskDelay(1); // Berikan kesempatan IDLE task Core 1
#endif
}

// Timpa (override) fungsi yield() default
void yield() {
  // Mencegah rekursi tak terbatas
  static bool in_yield = false;

  if (!in_yield) {
    in_yield = true;

    // Isi buffer step event sampai >= 75% penuh.
    // PENTING untuk M4 laser raster: segment sangat pendek (<0.5mm),
    // tiap segment hanya menghasilkan sedikit event, sehingga harus
    // loop melewati BANYAK segment (banyak startmove) dalam satu yield().
    //
    // Buffer 2048 @ 16us/tick = 32ms total.
    // Target: jaga sebFree <= 25% (= 512 slot kosong) = 1536 event = ~24ms
    // cadangan.
    if (jobRunning && !jobPaused && !stopping) {
      const uint32_t TARGET_FREE = STEP_EVENT_BUFFER_SIZE >> 2; // 25% = 512

      // Loop outer: terus isi buffer melewati banyak segment
      while (true) {
        int32_t sebFree = (int32_t)STEP_EVENT_BUFFER_SIZE - (int32_t)seblength;

        if (sebFree <= (int32_t)TARGET_FREE)
          break; // Buffer sudah cukup terisi
        if (sebfull)
          break; // Buffer penuh total

        if (m != nullptr) {
          // Ada segment aktif — isi event dari segment ini
          segmentloop();

          if (mctr <= 0) {
            // Segment selesai — tandai dan reset pointer
            // ISR mungkin masih akses m, jadi update atomik
            if (m)
              m->status = 0;
            m = pcurr = 0;
            // Lanjut ke iterasi berikutnya (akan startmove jika buffempty
            // false)
          }
        } else {
          // Tidak ada segment aktif — ambil segment berikutnya dari move buffer
          if (!buffempty) {
            int32_t res = startmove();
            if (res == 0 || m == nullptr) {
              // startmove() skip (zero-length move) — coba lagi di iterasi
              // berikutnya tapi jika buffer sudah habis juga setelah ini,
              // keluar
              if (buffempty)
                break;
              // Ada move lain, lanjut
              continue;
            }
            // startmove() sukses, m sudah di-set, lanjut loop untuk isi event
          } else {
            // Tidak ada move di buffer — tunggu processJob() mengisi
            break;
          }
        }
      }
    }

    in_yield = false;
  }

  // WAJIB: Teruskan ke sistem (WiFi & Network stack butuh waktu)
  __yield();
}
