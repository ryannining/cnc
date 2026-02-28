// UI PART - Menu LCD untuk CNC Controller
// Refactored: Struktur menu terpusat, mudah dimaintain

#include <Arduino.h>
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ESP32)
#include <WiFi.h>
#endif
#include "karyalcd.h"
#include "structures.h"
#include <LittleFS.h>
#include <stdint.h>

// ============================================
// LCD CONFIGURATION
// ============================================
#ifdef ESP8266
int lcd_rst = D3;
int lcd_sda = D3;
int lcd_sda2 = D1;
int lcd_contrast = 6;
int lcd_scl = D2;
int lcd_cs = D4;
int lcd_addr = 0x3c;
int lcd_kind = KIND_ST7565;
int lIR_KEY = D3;
#else
int lcd_rst = IO26;
int lcd_sda = IO26;
int lcd_sda2 = IO25;
int lcd_contrast = 6;
int lcd_scl = IO27;
int lcd_cs = IO5;
int lcd_addr = 0x3c;
int lcd_kind = KIND_ST7565;
int lIR_KEY = 3;
#endif
bool IR_ok;
float LCD_Y = 13.2;
float LCD_X = 6.4;
int YOFS = 0;
int XOFS = 0;
int LINES = 5;
bool oledready = false;
KaryaLCD xdisplay(KIND_ST7565);

// ============================================
// IR KEY DEFINITIONS
// ============================================
#define IRK_1 0x45
#define IRK_2 0x46
#define IRK_3 0x47
#define IRK_4 0x44
#define IRK_5 0x40
#define IRK_6 0x43
#define IRK_7 0x7
#define IRK_8 0x15
#define IRK_9 0x9
#define IRK_0 0x19
#define IRK_X 0x16
#define IRK_H 0xD
#define IRK_UP 0x18
#define IRK_LF 0x8
#define IRK_RG 0x5A
#define IRK_DN 0x52
#define IRK_OK 0x1C

#define RM4(n) (n + 1024)
#define IRK4_I_II RM4(10)
#define IRK4_INPUT RM4(48)
#define IRK4_SHAPING RM4(11)

#define IRK4_QMENU RM4(69)
#define IRK4_REVEAL RM4(42)

#define IRK4_OK RM4(68)
#define IRK4_PAUSE RM4(186)
#define IRK4_PLAY RM4(176)
#define IRK4_SLOWDOWN RM4(143)
#define IRK4_FASTER RM4(142)
#define IRK4_JOBINDEX RM4(112)
#define IRK4_EXIT RM4(91)
#define IRK4_SETTINGS RM4(67)
#define IRK4_ZUP RM4(0)
#define IRK4_ZDN RM4(1)
#define IRK4_SPINDLEUP RM4(2)
#define IRK4_SPINDLEDN RM4(3)
#define IRK4_BACK2 RM4(40)
#define IRK4_BACK RM4(7)
#define IRK4_UP RM4(64)
#define IRK4_LF RM4(7)
#define IRK4_RG RM4(6)
#define IRK4_DN RM4(65)
#define IRK4_X RM4(83)
#define IRK4_H RM4(26)
#define IRK4_1 RM4(17)
#define IRK4_2 RM4(18)
#define IRK4_3 RM4(19)
#define IRK4_4 RM4(20)
#define IRK4_5 RM4(21)
#define IRK4_6 RM4(22)
#define IRK4_7 RM4(23)
#define IRK4_8 RM4(24)
#define IRK4_9 RM4(25)
#define IRK4_0 RM4(16)
#define IRK4_STOP RM4(177)
#define IRK4_POWER RM4(8)
#define IRK4_DOT1 1138
#define IRK4_DOT2 1137
#define IRK4_DOT3 1123
#define IRK4_DOT4 1121

// ============================================
// EXTERN VARIABLES DARI cnc.ino
// ============================================
extern bool IR_ok;
extern int lIR_KEY;
extern float cx1, cy1, cz1;
extern volatile int pwmValue;
extern void applyZTrim(float deltaMm);
extern int globalSpeed;
extern bool jobRunning;
extern bool jobPaused;
extern String currentJobFilename;
extern int progressPercent;
extern volatile uint32_t absTime;
extern uint32_t jobStartTime;
extern uint32_t lastJobDuration;
float lastJobSize[2];

extern int positionHistoryIndex;
extern struct PositionRecord positionHistory[];
extern void addmove(float cf, float cx2, float cy2, float cz2, int toolPower,
                    int g0 = 1, int rel = 0, uint8_t jogFlags = 0);

extern void setToolPower(int power);
extern void testLaser();
extern void startJob(String filename);
extern void stopJob();
extern void toggleJobPause();
extern void remotePause();
extern void remoteResume();
extern volatile uint8_t pauseSeq;
extern void resetAll();
extern int getGlobalSpeed();
extern bool shaper_enabled;
extern AxisShaper shapers[3];
extern char cncIP[];
extern float JobX, JobY, JobZ;
extern float jobMaxZ;   // FIX 2026-08-30: Z tertinggi saat job, dipakai lift aman traverse
extern float clampJogAxis(float pos, float step, float lo, float hi);   // soft-limit jog (cnc.ino)
extern bool setJogLimitEnabled(bool en); // toggle + persist jog limit (cnc.ino)
extern String jobsize;
extern void previewJob(String filename, bool perform);

// ============================================
// MENU SYSTEM - TYPE DEFINITIONS
// ============================================
typedef void (*MenuHandler)(int key);

struct MenuItem {
  const char *label;
  MenuHandler action;
};

struct Menu {
  int id;
  const char *title;
  MenuItem *items;
  int itemCount;
  int columns;
  MenuHandler onKey;
  bool dynamic;
};

// ============================================
// MENU STATE
// ============================================
#define MENU_HOME -1
#define MENU_MAIN 0
#define MENU_JOBLIST 2
#define MENU_CONFIRM 80

int menuId = MENU_HOME;
int menuPos = 0;
int menuPage = 0;
int menuPageCount = 0;
String confirmTitle;
MenuHandler confirmCallback = nullptr;
String selectedJobName;
bool useJobSize = false;
int jogMul = 100;
int savedMainMenuPos = 0;
int savedJobListPos = 0;
int savedJobListPage = 0;

// ============================================
// FORWARD DECLARATIONS
// ============================================
void actionSetZero(int key);
void actionOpenJobs(int key);
void actionGoToZero(int key);
void actionGoToJobStart(int key);
void actionJogLimit(int key);
void updateJogLimitLabel();
void jobListOnKey(int key);
void confirmOnKey(int key);
void homeOnKey(int key);
void mainMenuOnKey(int key);
void refreshJobList();
void drawMenu();
void resetLCD();
void initUI();
void loadJobListWithSavedPosition();

// ============================================
// MENU ITEM DEFINITIONS
// ============================================
// Label dinamis menu "Jog Limit" (menampilkan status ON/OFF)
char jogLimitLabel[24] = "Jog Limit: ON";
void updateJogLimitLabel() {
  snprintf(jogLimitLabel, sizeof(jogLimitLabel), "Jog Limit: %s",
           systemCfg.jogLimitEnabled ? "ON" : "OFF");
}

MenuItem mainItems[] = {
    {"Set Zero", actionSetZero},
    {"My Jobs", actionOpenJobs},
    {"Go to zero", actionGoToZero},
    {"Go to job start", actionGoToJobStart},
    {jogLimitLabel, actionJogLimit},
};

MenuItem confirmItems[] = {
    {"YES", nullptr},
    {"NO", nullptr},
};

#define NUMJOBS 30
String jobListData[(NUMJOBS + 1) * 2];
MenuItem *jobListItems = nullptr;
int jobListCount = 0;

// ============================================
// MENU REGISTRY
// ============================================
Menu menus[] = {
    {MENU_MAIN, "Main Menu", mainItems, 5, 1, mainMenuOnKey, false},
    {MENU_JOBLIST, "File", nullptr, 0, 1, jobListOnKey, true},
    {MENU_CONFIRM, "", confirmItems, 2, 1, confirmOnKey, false},
};
int menuCount = sizeof(menus) / sizeof(menus[0]);

Menu *currentMenu = nullptr;

// ============================================
// GLOBAL KEY BINDINGS
// ============================================
struct GlobalKey {
  int key;
  void (*handler)();
  bool requireNoJob;
};

void handlePause();
void handleResume();
void handleStop();
void handleSpeedUp();
void handleSpeedDown();
void handleShaperToggle();
void handleExit();
void handlePowerReset();
void handleSpindleUp();
void handleSpindleDown();

GlobalKey globalKeys[] = {
    {IRK4_PAUSE, handlePause, false},
    {IRK4_PLAY, handleResume, false},
    {IRK4_STOP, handleStop, false},
    {IRK4_FASTER, handleSpeedUp, false},
    {IRK4_SLOWDOWN, handleSpeedDown, false},
    {IRK4_SHAPING, handleShaperToggle, true},
    {IRK4_EXIT, handleExit, false},
    {IRK4_POWER, handlePowerReset, true},
    {IRK4_SPINDLEUP, handleSpindleUp, true},
    {IRK4_SPINDLEDN, handleSpindleDown, true},
};
int globalKeyCount = sizeof(globalKeys) / sizeof(globalKeys[0]);

// ============================================
// JOG KEY BINDINGS
// ============================================
struct JogKey {
  int key;
  float dx, dy, dz;
};

JogKey jogKeys[] = {
    {IRK_UP, 0, -1, 0},    {IRK_DN, 0, 1, 0},      {IRK_LF, -1, 0, 0},
    {IRK_RG, 1, 0, 0},     {IRK_1, 0, 0, 1},       {IRK_7, 0, 0, -1},
    {IRK_3, 0, 0, 0.5},    {IRK_9, 0, 0, -0.5},    {IRK_2, 0, -10, 0},
    {IRK_4, -10, 0, 0},    {IRK_6, 10, 0, 0},      {IRK_8, 0, 10, 0},
    {IRK4_UP, 0, -1, 0},   {IRK4_DN, 0, 1, 0},     {IRK4_LF, -1, 0, 0},
    {IRK4_RG, 1, 0, 0},    {IRK4_1, 0, 0, 1},      {IRK4_7, 0, 0, -1},
    {IRK4_3, 0, 0, 0.5},   {IRK4_9, 0, 0, -0.5},   {IRK4_2, 0, -10, 0},
    {IRK4_4, -10, 0, 0},   {IRK4_6, 10, 0, 0},     {IRK4_8, 0, 10, 0},
    {IRK4_ZUP, 0, 0, 0.1}, {IRK4_ZDN, 0, 0, -0.1},
};
int jogKeyCount = sizeof(jogKeys) / sizeof(jogKeys[0]);

// ============================================
// JOG MULTIPLIER PRESETS
// ============================================
int jogMulPresets[] = {25, 100, 250, 500};
int jogMulPresetCount = 4;

// ============================================
// DEBUG KEY MODE
// ============================================
bool debugKeyMode = false;
int lastDebugKey = 0;

// ============================================
// IR FUNCTIONS
// ============================================
void IR_setup() { IR_ok = IRBegin(lIR_KEY); }

void IR_end() {
  IREnd(lIR_KEY);
  IR_ok = false;
}

bool canrepeat(int n) {
  return (n == IRK4_OK || n == IRK4_UP || n == IRK4_DN || n == IRK4_LF ||
          n == IRK4_RG);
}

int lok, lad, rep;

int getRemoteKey() {
  if (IRAvailable()) {
    auto data = IRRead();
    int ok = data.command & 255;
    int ad = data.address & 255;
    if (ok == 0 && ad == 255) {
      rep++;
    } else {
      rep = 1;
      lok = ok;
      lad = ad;
    }
    if (ok == 0 && ad == 0)
      return 0; // frame korup dibuang (validasi inverse NEC gagal)
    return (lad * 256) + lok;
  }
  return 0;
}

// ============================================
// LCD IDLE WATCHDOG (anti listrik statis/ESD) — FIX 2026-08-30
// Jika tidak ada aktivitas (remote/web/refresh layar) > 2 menit dan tidak
// sedang job, paksa re-init controller + full redraw -> layar yang blank
// kena statis saat mesin ditinggal otomatis menyala kembali.
// ============================================
void d_show();  // forward decl (definisi di bawah)
static uint32_t lastLcdActivity = 0;
void touchLcdActivity() { lastLcdActivity = millis(); }

// FIX 2026-08-31: reset LCD kini DEFERRED via flag. Transfer full-frame bit-bang
// SPI memblokir ISR motion → motor freeze mendadak + menghentak bila dilakukan
// saat step buffer masih berisi (mis. baru saja pause). Semua pemicu reset hanya
// menandai flag; eksekusi sesungguhnya oleh coreloop() (cnc.ino) HANYA saat step
// buffer kosong DAN shaper selesai (motor diam fisik).
volatile bool lcdResetPending = false;
void requestLcdReset() { lcdResetPending = true; }

void lcdIdleCheck() {
  if (jobRunning) { touchLcdActivity(); return; }  // saat job layar refresh rutin
  if (millis() - lastLcdActivity > 120000UL) {
    requestLcdReset();  // deferred — aman juga saat jog (buffer tidak dipotong)
  }
}

// ============================================
// LCD HELPER FUNCTIONS
// ============================================
void d_clear() { xdisplay.clear(); }

void d_show() {
  IR_end();
  touchLcdActivity();   // setiap refresh layar = aktivitas
  xdisplay.display();
  IR_setup();
}

// FIX 2026-08-30: re-init controller + full frame sekaligus, untuk dipanggil
// dari cnc.ino saat transisi job (selesai/stop/pause) — heal segera kalau
// ESD ke-reset controller LCD di tengah job. IR di-pause agar tidak konflik
// dengan ISR remote selama transfer SPI.
void lcdForceFullRedraw() {
  IR_end();
  touchLcdActivity();
  xdisplay.forceFullRedraw();
  xdisplay.display();
  IR_setup();
}

void d_setcolor(uint8_t c) { xdisplay.setColor((OLEDDISPLAY_COLOR)c); }

void d_text(int x, int y, String s) {
  xdisplay.drawString((x)*LCD_X + XOFS, (y)*LCD_Y + YOFS, s);
}

int d_w() {
  switch (lcd_kind) {
  case 0:
    return 162;
  case 1:
    return 96;
  case 2:
    return 128;
  case 3:
    return 128;
  case 4:
    return 128;
  case 5:
    return 160;
  case 6:
    return 160;
  default:
    return 128;
  }
}

void d_rect(int a, int b, int c, int d) { xdisplay.drawRect(a, b, c, d); }
void d_frect(int a, int b, int c, int d) { xdisplay.fillRect(a, b, c, d); }

// ============================================
// TIME FORMAT HELPERS
// ============================================
String formatTimeShort(long ms) {
  long secs = ms / 1000;
  long mins = secs / 60;
  long hours = mins / 60;
  secs = secs % 60;
  mins = mins % 60;

  String result = "";
  if (hours > 0) {
    result += String(hours) + ":";
    if (mins < 10)
      result += "0";
  }
  result += String(mins) + ":";
  if (secs < 10)
    result += "0";
  result += String(secs);
  return result;
}

// ============================================
// MENU FUNCTIONS
// ============================================
void saveCurrentMenuPosition() {
  if (menuId == MENU_MAIN) {
    savedMainMenuPos = menuPos;
  } else if (menuId == MENU_JOBLIST) {
    savedJobListPos = menuPos;
    savedJobListPage = menuPage;
  }
}

void loadMenu(int id) {
  saveCurrentMenuPosition();

  for (int i = 0; i < menuCount; i++) {
    if (menus[i].id == id) {
      currentMenu = &menus[i];
      menuId = id;

      if (id == MENU_MAIN) {
        menuPos = savedMainMenuPos;
        menuPage = 0;
      } else if (id == MENU_JOBLIST) {
        menuPos = savedJobListPos;
        menuPage = savedJobListPage;
      } else {
        menuPos = 0;
        menuPage = 0;
      }

      if (currentMenu->dynamic && id == MENU_JOBLIST) {
        refreshJobList();
        int totalItems = jobListCount / 2 - 1;
        if (totalItems < 0)
          totalItems = 0;
        if (menuPos > totalItems)
          menuPos = totalItems;
        if (menuPage > menuPos)
          menuPage = max(0, menuPos - (LINES - 2));
      }

      if (id == MENU_CONFIRM) {
        currentMenu->title = confirmTitle.c_str();
      }

      if (id == MENU_MAIN) {
        updateJogLimitLabel(); // refresh status ON/OFF di label menu
      }

      menuPageCount = max(0, currentMenu->itemCount - (LINES - 1));
      drawMenu();
      return;
    }
  }
}

void loadHome() {
  saveCurrentMenuPosition();
  currentMenu = nullptr;
  menuId = MENU_HOME;
  menuPos = 0;
  menuPage = 0;
}

void loadJobListWithSavedPosition() { loadMenu(MENU_JOBLIST); }

// ============================================
// JOB LIST FUNCTIONS
// ============================================
void refreshJobList() {
  jobListCount = 0;
  jobListData[0] = "File";
  jobListData[1] = "Kb";

  int i = 0;

#if defined(ESP8266)
  Dir dir = LittleFS.openDir("/");
  while (dir.next() && i < NUMJOBS) {
    String filename = dir.fileName();
    if (filename.endsWith(".gcode")) {
      filename.remove(filename.length() - 6);
      jobListData[i * 2 + 2] = filename;
      jobListData[i * 2 + 3] = String(dir.fileSize() / 1024);
      i++;
    }
  }
#elif defined(ESP32)
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  while (file && i < NUMJOBS) {
    String filename = file.name();
    if (filename.endsWith(".gcode")) {
      filename.remove(filename.length() - 6);
      jobListData[i * 2 + 2] = filename;
      jobListData[i * 2 + 3] = String(file.size() / 1024);
      i++;
    }
    file = root.openNextFile();
  }
#endif

  jobListCount = i * 2 + 2;
}

String getJobNameAt(int pos) {
  if (pos < 0 || pos >= jobListCount / 2)
    return "";
  return jobListData[pos * 2 + 2] + ".gcode";
}

void syncJobSelection(String filename) {
  // Clean up filename (remove path and extension)
  String target = filename;
  if (target.startsWith("/"))
    target.remove(0, 1);
  if (target.endsWith(".gcode"))
    target.remove(target.length() - 6);

  refreshJobList();

  int items = (jobListCount - 2) / 2;
  for (int i = 0; i < items; i++) {
    if (jobListData[i * 2 + 2] == target) {
      menuPos = i;
      selectedJobName = target + ".gcode";
      savedJobListPos = menuPos;

      // Calculate menuPage to make it visible
      if (menuPos >= (LINES - 1)) {
        menuPage = menuPos - (LINES - 2);
      } else {
        menuPage = 0;
      }
      savedJobListPage = menuPage;
      break;
    }
  }
}

// ============================================
// DRAW FUNCTIONS
// ============================================
float lastx = 0, lasty = 0, lastz = 0, lastV = 0;
uint32_t lcalcV = 0;

void drawHome() {
  d_clear();
  d_setcolor(WHITE);

  d_text(0, 0, cncIP);

  if (shaper_enabled) {
    d_text(10, 0, "iS");
  }

  char buf[32];

  if (debugKeyMode && lastDebugKey != 0) {
    sprintf(buf, "K:%d", lastDebugKey);
    d_text(12, 0, buf);
  } else {
    sprintf(buf, "V%2.1f", lastV);
    d_text(13, 0, buf);
  }

  sprintf(buf, "Fr%d", getGlobalSpeed());
  d_text(13, 2, buf);

  float x = shapers[0].realStep * shapers[0].invStepsPerMm;
  float y = shapers[1].realStep * shapers[1].invStepsPerMm;
  float z = shapers[2].realStep * shapers[2].invStepsPerMm;
  // Serial.println(shapers[2].invStepsPerMm);

  if (absTime - lcalcV > 0) {
    lastx -= x;
    lasty -= y;
    lastz -= z;
    lastV = sqrt(lastx * lastx + lasty * lasty + lastz * lastz) /
            ((float)(absTime - lcalcV) * TICK_MICROS * 10.0 / 10000000);
    lastx = x;
    lasty = y;
    lastz = z;
    lcalcV = absTime;
  }

  sprintf(buf, "X%2.1f", x);
  d_text(1, 1, buf);

  sprintf(buf, "Y%2.1f", y);
  d_text(8, 1, buf);

  sprintf(buf, "Z%2.1f", z);
  d_text(15, 1, buf);

  int power = (pwmValue * 100) / 256;
  d_text(0, 2, "S " + String(power) + "%");

  if (jobRunning) {
    String status = jobPaused
                        ? ("Paused s" + String((int)pauseSeq))
                        : currentJobFilename;
    status += " " + String(progressPercent) + "%";
    d_text(0, 3, status);
  } else {
    if (useJobSize) {
      d_text(0, 3, "Jog by jobsize");
    } else {
      d_text(0, 3, "Jog: " + String(jogMul));
    }
  }

  if (jobRunning)
    lastJobDuration = (millis() - jobStartTime);
  String jobTimeStr = "J" + formatTimeShort(lastJobDuration);
  d_text(0, 4, jobTimeStr);

  String upStr = "U" + formatTimeShort(millis());
  int upX = 20 - upStr.length();
  if (upX < 10)
    upX = 10;
  d_text(upX, 4, upStr);

  d_show();
}

void drawMenu() {
  if (!currentMenu)
    return;

  d_clear();
  d_setcolor(WHITE);

  const char *title = currentMenu->title;
  if (menuId == MENU_JOBLIST) {
    title = jobListData[0].c_str();
  }
  d_text(0, 0, title);
  d_rect(0, LCD_Y - 2, d_w(), 1);

  d_frect(0, LCD_Y * (1 + menuPos - menuPage) - 1, d_w() + 2, LCD_Y);

  int startPos = menuPage;
  int visibleLines = LINES - 1;

  for (int i = 0; i < visibleLines; i++) {
    int itemIdx = startPos + i;
    int totalItems = currentMenu->itemCount;

    if (menuId == MENU_JOBLIST) {
      totalItems = jobListCount / 2 - 1;
    }

    if (itemIdx >= totalItems)
      break;

    d_setcolor(itemIdx == menuPos ? 0 : 1);

    String label;
    if (menuId == MENU_JOBLIST) {
      String name = jobListData[(itemIdx + 1) * 2];
      String size = jobListData[(itemIdx + 1) * 2 + 1];
      label = name + "-" + size + "kb";
    } else if (menuId == MENU_CONFIRM) {
      label = currentMenu->items[itemIdx].label;
    } else {
      label = currentMenu->items[itemIdx].label;
    }

    d_text(1, i + 1, label);
  }

  d_show();
}

// ============================================
// GLOBAL KEY HANDLERS
// ============================================
void handlePause() {
  // Hanya PAUSE (tidak toggle) — resume harus lewat tombol Play.
  remotePause();
}

void handleResume() {
  remoteResume();
}

void handleStop() {
  if (jobRunning) {
    stopJob();
  }
}

void handleSpeedUp() {
  if (menuId == MENU_HOME && globalSpeed < 48) {
    globalSpeed += 2;
  }
}

void handleSpeedDown() {
  if (menuId == MENU_HOME && globalSpeed > 2) {
    globalSpeed -= 1;
  }
}

void handleShaperToggle() {
  if (!jobRunning) {
    motionCfg.enableInputShaping = !motionCfg.enableInputShaping;
    syncShaperBuffer();
  }
}

void handleExit() { loadHome(); }

void handlePowerReset() {
  // FIX (rev 7): jobPaused MASIH jobRunning=true — dulu tombol reset LCD mati
  // total saat pause (guard !jobRunning menolaknya). Saat pause main loop
  // relatif kosong → reset LCD aman dilakukan.
  if (!jobRunning || jobPaused) {
    resetLCD();
  }
}

void handleSpindleUp() {
  if (!jobRunning) {
    if (toolCfg.mode == 1) {
      testLaser();
    } else {
      setToolPower(pwmValue + 25);
    }
  }
}

void handleSpindleDown() {
  if (!jobRunning) {
    if (toolCfg.mode == 1) {
      testLaser();
    } else {
      setToolPower(pwmValue - 25);
    }
  }
}

// ============================================
// MENU ACTION HANDLERS
// ============================================
void actionSetZero(int key) {
  resetAll();
  extern void saveHeadPosition();  // FIX 2026-09-01: file rescue disinkronkan ke zero baru
  saveHeadPosition();              // (timpa dengan 0,0,0 — resetAll kini tidak menyentuh file)
  loadHome();
}

void actionOpenJobs(int key) {
  if (!jobRunning) {
    loadMenu(MENU_JOBLIST);
  } else {
    confirmTitle = "Stop running job ?";
    confirmCallback = [](int confirmed) {
      if (confirmed) {
        stopJob();
      }
      loadMenu(MENU_JOBLIST);
    };
    currentMenu->title = confirmTitle.c_str();
    loadMenu(MENU_CONFIRM);
  }
}

// FIX 2026-08-30: lift Z aman = max(Z saat ini, Z tertinggi job) tapi tidak
// melebihi jarak dari titik start job (jobMaxZ adalah Z absolut tertinggi
// yang pernah disuruh mesin → terbukti aman mekanis).
// Gerakan lift dipakai RELATIF (addmove rel=1) seperti kode lama, jadi
// dihitung sebagai delta terhadap cz1 saat menu dipilih.
static float safeLiftRel() {
  float zTarget = jobMaxZ;
  if (cz1 > zTarget) zTarget = cz1;      // sudah lebih tinggi dari jobMaxZ → tidak turun, lift minimum 1mm
  float rel = zTarget - cz1;
  if (rel < 1.0f) rel = 1.0f;            // minimal 1mm clear
  return rel;
}

// Toggle soft-limit jog dari menu utama LCD (tersimpan ke /config.json)
void actionJogLimit(int key) {
  if (key == IRK_OK || key == IRK4_OK) {
    if (jobRunning) {
      log0("Jog Limit: tidak bisa diubah saat job berjalan\n");
      return;
    }
    setJogLimitEnabled(!systemCfg.jogLimitEnabled);
    updateJogLimitLabel();
    drawMenu();
  }
}

void actionGoToZero(int key) {
  if (!jobRunning) {
    // Butuh 3 slot beruntun: tolak sekalian bila kurang agar urutan tak bolong
    if (bufflength + 3 >= NUMBUFFER) {
      log0("Buffer penuh, GoTo Zero dibatalkan\n");
      return;
    }
    addmove(100, 0, 0, safeLiftRel(), 0, 1, 1, false);
    addmove(100, 0, 0, cz1, 0, 1, 0, false);
    addmove(100, 0, 0, -15, 0, 1, 1, false);
    loadHome();
  }
}

void actionGoToJobStart(int key) {
  if (!jobRunning) {
    // Butuh 3 slot beruntun: tolak sekalian bila kurang agar urutan tak bolong
    if (bufflength + 3 >= NUMBUFFER) {
      log0("Buffer penuh, GoTo JobStart dibatalkan\n");
      return;
    }
    addmove(100, 0, 0, safeLiftRel(), 0, 1, 1, false);
    addmove(100, JobX, JobY, cz1, 0, 1, 0, false);
    addmove(100, JobX, JobY, JobZ, 0, 1, 0, false);
    loadHome();
  }
}

// ============================================
// MENU KEY HANDLERS
// ============================================
void mainMenuOnKey(int key) {
  if (key == IRK_LF || key == IRK4_LF || key == IRK4_BACK ||
      key == IRK4_BACK2) {
    loadHome();
    return;
  }

  if (key == IRK_OK || key == IRK4_OK) {
    if (currentMenu->items && menuPos >= 0 &&
        menuPos < currentMenu->itemCount) {
      if (currentMenu->items[menuPos].action) {
        currentMenu->items[menuPos].action(key);
      }
    }
  }
}

void confirmOnKey(int key) {
  if (key == IRK_OK || key == IRK_LF || key == IRK4_OK || key == IRK4_LF ||
      key == IRK4_BACK2) {
    bool confirmed = (key == IRK_OK || key == IRK4_OK) && (menuPos == 0);
    if (confirmCallback) {
      confirmCallback(confirmed ? 1 : 0);
    }
  }
}

void jobListOnKey(int key) {
  static int lastKey = 0;

  if (key == IRK_LF || key == IRK4_LF || key == IRK4_BACK ||
      key == IRK4_BACK2) {
    loadMenu(MENU_MAIN);
    return;
  }

  if (key == IRK4_STOP) {
    selectedJobName = getJobNameAt(menuPos);
    if (selectedJobName.length() > 0) {
      savedJobListPos = menuPos;
      savedJobListPage = menuPage;
      confirmTitle = "Delete " + selectedJobName + "?";
      confirmCallback = [](int confirmed) {
        if (confirmed) {
          LittleFS.remove("/" + selectedJobName);
          refreshJobList();
        }
        loadJobListWithSavedPosition();
      };
      loadMenu(MENU_CONFIRM);
    }
    return;
  }

  if (key == IRK_OK || key == IRK4_OK) {
    selectedJobName = getJobNameAt(menuPos);
    savedJobListPos = menuPos;
    savedJobListPage = menuPage;
    confirmTitle = "Run " + selectedJobName + "?";
    confirmCallback = [](int confirmed) {
      if (confirmed) {
        startJob(selectedJobName);
        loadHome();
      } else {
        loadJobListWithSavedPosition();
      }
    };
    loadMenu(MENU_CONFIRM);
    return;
  }

  if (key == IRK4_QMENU || (key == IRK4_REVEAL && rep == 1)) {
    selectedJobName = getJobNameAt(menuPos);
    previewJob(selectedJobName, key == IRK4_REVEAL);
    if (key != IRK4_REVEAL) {
      jobListData[0] = jobsize;
    }
    drawMenu();
  }

  lastKey = key;

  float x = 0, y = 0, z = 0;
  for (int i = 0; i < jogKeyCount; i++) {
    if (key == jogKeys[i].key) {
      x = jogKeys[i].dx;
      y = jogKeys[i].dy;
      z = jogKeys[i].dz;
      break;
    }
  }

  if (x != 0 || y != 0 || z != 0) {
    // FIX 2026-08-30: soft-limit X/Y (mesin tanpa endstop)
    x = clampJogAxis(cx1, x, -50.0f, 1300.0f);
    y = clampJogAxis(cy1, y, -50.0f, 2500.0f);
    if (x != 0 || y != 0 || z != 0) {
      addmove(100, x, y, z, 0, 1, 1, JOG_FLAG_LIMIT | JOG_FLAG_EXTEND);
    }
  }
}

void homeOnKey(int key) {
  if (key == IRK_OK || key == IRK4_OK) {
    loadMenu(MENU_MAIN);
    return;
  }

  if (key == IRK_5 || key == IRK4_5) {
    jogMul = (jogMul == 100) ? 500 : 100;
    return;
  }

  if (key == IRK4_REVEAL) {
    if (lastJobSize[0] > 0 && lastJobSize[1] > 0) {
      useJobSize = !useJobSize;
      jogMul = 100;
    }
    return;
  }

  if (key == IRK4_DOT1) {
    jogMul = 25;
    return;
  }
  if (key == IRK4_DOT2) {
    jogMul = 100;
    return;
  }
  if (key == IRK4_DOT3) {
    jogMul = 250;
    return;
  }
  if (key == IRK4_DOT4) {
    jogMul = 500;
    return;
  }

  if (key == IRK_X || key == IRK4_X) {
    debugKeyMode = !debugKeyMode;
    return;
  }

  float x = 0, y = 0, z = 0;
  for (int i = 0; i < jogKeyCount; i++) {
    if (key == jogKeys[i].key) {
      x = jogKeys[i].dx;
      y = jogKeys[i].dy;
      z = jogKeys[i].dz;
      break;
    }
  }

  // Saat job jalan: tombol Z jadi TRIM (kenop virtual) — 0.5mm per tekan.
  // Step diproses ISR hanya saat motor Z diam & ≤ kecepatan max Z.
  if (jobRunning && x == 0 && y == 0 && z != 0) {
    applyZTrim(z > 0 ? 0.5f : -0.5f);
    return;
  }

  if (x != 0 || y != 0 || z != 0) {
    if (useJobSize) {
      if (abs(x) > 1)
        x = (x > 0) ? lastJobSize[0] : -lastJobSize[0];
      if (abs(y) > 1)
        y = (y > 0) ? lastJobSize[1] : -lastJobSize[1];
    }
    // FIX 2026-08-30: soft-limit X/Y (mesin tanpa endstop) — delta final setelah multiplier
    float dx = clampJogAxis(cx1, x * jogMul * 0.01, -50.0f, 1300.0f);
    float dy = clampJogAxis(cy1, y * jogMul * 0.01, -50.0f, 2500.0f);
    if (dx != 0 || dy != 0 || z != 0) {
      addmove(100, dx, dy, z, 0, 1, 1, JOG_FLAG_LIMIT | JOG_FLAG_EXTEND);
    }
  }
}

// ============================================
// MAIN UI LOOP
// ============================================
uint32_t ldisplayTime = 0;

void handleMenuNavigation(int key) {
  int y = 0;

  if (key == IRK_UP || key == IRK4_UP)
    y = -1;
  if (key == IRK_DN || key == IRK4_DN)
    y = +1;

  if (y != 0) {
    int totalItems = currentMenu->itemCount;
    if (menuId == MENU_JOBLIST) {
      totalItems = jobListCount / 2 - 1;
    }

    menuPos += y;

    int maxPos = totalItems - 1;
    if (maxPos < 0)
      maxPos = 0;

    if (menuPos < 0) {
      menuPos = maxPos;
      menuPage = max(0, maxPos - (LINES - 2));
    }
    if (menuPos > maxPos) {
      menuPos = 0;
      menuPage = 0;
    }
    if (menuPos > menuPage + (LINES - 2))
      menuPage++;
    if (menuPos < menuPage)
      menuPage--;

    drawMenu();
  } else if (currentMenu->onKey) {
    currentMenu->onKey(key);
  }
}

int ir_oled_loop(int icommand) {
  if (icommand != 0) touchLcdActivity();  // tombol remote apapun = aktivitas (anti-static watchdog)
  // Gunakan millis() bukan absTime agar display selalu update walau
  // timer shaper ISR kelaparan saat motor step rate tinggi.
  // OPTIM 2026-08-30: saat job berjalan, redraw periodik cukup 250ms
  // (angka progress tidak butuh 20fps) -> hemat 3-5ms CPU tiap tick utk parser/WiFi.
  static uint32_t lastAutoRedraw = 0;
  uint32_t redrawInterval = jobRunning ? 250 : 100;
  // FIX lag-jog: ldisplayTime di-update ISR IR tiap edge pulsa → saat tombol
  // ditahan (repeat ~75ms) redraw tidak pernah tercapai → layar beku.
  // Timer redraw kini milik ui.cpp saja; ldisplayTime hanya dipakai ISR
  // sebagai penanda aktivitas remote (touchLcdActivity di atas yang pegang).
  if (icommand == 0 && millis() - lastAutoRedraw > redrawInterval) {
    lastAutoRedraw = millis();
    if (menuId == MENU_HOME) {
      drawHome();
    }
  }

  if (icommand == 0)
    return 0;

  lastDebugKey = icommand;

  for (int i = 0; i < globalKeyCount; i++) {
    if (icommand == globalKeys[i].key) {
      if (!globalKeys[i].requireNoJob || !jobRunning) {
        globalKeys[i].handler();
      }
    }
  }

  if (menuId == MENU_HOME) {
    homeOnKey(icommand);
    return 0;
  }

  handleMenuNavigation(icommand);
  return 1;
}

void ui_loop() {
  lcdIdleCheck();   // FIX 2026-08-30: watchdog anti listrik statis saat idle
  int xcmd = getRemoteKey();
  if (xcmd) {
    ir_oled_loop(xcmd);
  } else {
    ir_oled_loop(0);
  }
}

// ============================================
// LCD INITIALIZATION
// ============================================
#include "fonts.h"

void resetLCD() {
  log0("reset lcd!\n");
  initUI();
  loadHome();
}

void initUI() {
  IR_end();
  lIR_KEY = lcd_sda;
  xdisplay.setparam(lcd_kind, lcd_addr, lcd_sda, lcd_sda2, lcd_scl, lcd_cs,
                    lcd_rst);
  xdisplay.setFont(
      lcd_kind == KIND_ST7735 || lcd_kind == KIND_NK1661 ? BIGFONT : BASICFONT);

  if (lcd_kind == KIND_NK1202) {
    LCD_Y = 14;
    LCD_X = 4.8;
    YOFS = 1;
    XOFS = 1;
    LINES = 5;
  }
  if (lcd_kind == KIND_ST7565) {
    LCD_Y = 13.2;
    LCD_X = 6.4;
    YOFS = 0;
    XOFS = 0;
    LINES = 5;
  }
  if (lcd_kind == KIND_NK6100) {
    LCD_Y = 20;
    LCD_X = 6.2;
    YOFS = 1;
    XOFS = 1;
    LINES = 6;
  }
  if (lcd_kind == KIND_ST7735) {
    LCD_Y = 21;
    LCD_X = 7.75;
    YOFS = 1;
    XOFS = 1;
    LINES = 6;
  }
  if (lcd_kind == KIND_NK1661) {
    LCD_Y = 21;
    LCD_X = 7.75;
    YOFS = 1;
    XOFS = 1;
    LINES = 6;
  }
  if (lcd_kind == KIND_OLED1306) {
    LCD_Y = 16;
    LCD_X = 6.5;
    YOFS = 0;
    XOFS = 0;
    LINES = 4;
  }

  xdisplay.init();
  xdisplay.clear();

  oledready = true;
  loadHome();
  IR_setup();
}

// ============================================
// END UI PART
// ============================================
