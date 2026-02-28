#include <Arduino.h>
#include "gcode.h"
#include "structures.h"

File currentJobFile;
size_t jobFileSize = 0;
size_t jobBytesRead = 0;
bool isParsingComplete = false;
bool isGxMode = false;
GXParserState gxState;

volatile bool isPreviewMode = false;
float previewMinX = 0;
float previewMaxX = 0;
float previewMinY = 0;
float previewMaxY = 0;
bool previewFirstMove = true;

bool openGxFile(String filepath) {
  isGxMode = false;
  currentJobFile = LittleFS.open(filepath, "r");

  if (!currentJobFile) {
    return false;
  }

  if (isGxFormat(currentJobFile)) {
    isGxMode = true;
    char hdr[3];
    currentJobFile.read((uint8_t*)&hdr, 3);
    int version = hdr[2];

    initGxParser(version);

    return true;
  }

  return false;
}

extern void setToolPower(int power);
extern void addmove(float cf, float cx2, float cy2, float cz2, int toolPower, int g0, int rel, uint8_t jogFlags);
extern void startHoming(int axis);
extern bool jobRunning;
extern bool jobPaused;
extern bool isHoming;
extern void testLaser();
extern float cx1, cy1, cz1;
extern float G0Feed, G1Feed;
extern int currentToolPower;

#define datasize(n) ((1 << (n*8-1))-2)

bool isGxFormat(File& file) {
  if (!file) return false;

  char hdr[3];
  long pos = file.position();
  file.read((uint8_t*)&hdr, 3);
  file.seek(pos);

  if (hdr[0] == 'G' && hdr[1] == 'X') {
    return true;
  }
  return false;
}

void initGxParser(int version) {
  gxState.version = version;
  gxState.initialized = true;
  gxState.AX = cx1;
  gxState.AY = cy1;
  gxState.AZ = cz1;

  gxState.fScale = 2;


  if (version == 1) {
    gxState.xyScale = 1.0 / 0.005;
    gxState.zScale = 1.0 / 0.1;
    gxState.eScale = 1.0 / 0.02;
    gxState.xySize = 2;
    gxState.zSize = 1;
    gxState.eSize = 1;
    gxState.xyLimit = datasize(2);
    gxState.zLimit = datasize(1);
    gxState.eLimit = datasize(1);
  } else {
    gxState.initialized = false;
  }
}
extern float lastJobSize[2];
void completeParse(){
  float width = (previewMaxX-previewMinX);
  float height = (previewMaxY-previewMinY);
  lastJobSize[0]=width+1;
  lastJobSize[1]=height+1;
}
void readGxLine() {
  if (!currentJobFile.available()) {
    isParsingComplete = true;
    completeParse();
    currentJobFile.close();
    return;
  }

  uint32_t qpos = currentJobFile.position();
  byte h;
  currentJobFile.read(&h, 1);

  struct GCodeCmd {
    byte seen_G = 0;
    byte seen_M = 0;
    int G = 0;
    int M = 0;
    byte seen_X = 0, seen_Y = 0, seen_Z = 0, seen_E = 0, seen_F = 0, seen_S = 0;
    float feed;
    int power;
    float targetX;
    float targetY;
    float targetZ;
  } cmd;

  // Initialize targets with current state
  cmd.targetX = gxState.AX;
  cmd.targetY = gxState.AY;
  cmd.targetZ = gxState.AZ;
  cmd.feed = G1Feed;
  cmd.power = currentToolPower;

  if (h & 1) {
    cmd.seen_M = 1;
    switch ((h >> 1) & 3) {
      case 0: cmd.M = 3; break;
      case 1: cmd.M = 109; break; // 104 is ignored
      case 2: break; // Bitmap parameter
    }
  } else {
    cmd.seen_G = 1;
    switch ((h >> 1) & 3) {
      case 0: cmd.G = 0; break;
      case 1: cmd.G = 1; break;
      case 2: cmd.G = 28; break;
      case 3: cmd.G = 92; break;
    }
  }

  if (h & (1 << 3)) { // F or S
    byte s = 0;
    currentJobFile.read(&s, 1);
    if (cmd.seen_M) {
      cmd.seen_S = 1;
      cmd.power = s;
    } else {
      cmd.seen_F = 1;
      cmd.feed = s * gxState.fScale;
      if (cmd.G == 1) {
        G1Feed = cmd.feed;
      } else {
        G0Feed = cmd.feed;
      }
    }
  }

  int32_t x = 0;
  float d;

  if (h & (1 << 4)) { // X
    x = 0;
    currentJobFile.read((uint8_t*)&x, gxState.xySize);
    cmd.seen_X = 1;
    if (cmd.seen_M) {
      // M-code parameter, consume but do not update path state
      // float val = float(x - gxState.xyLimit) * 100 / gxState.xyScale;
    } else {
      d = float(x - gxState.xyLimit) / gxState.xyScale;
      gxState.AX += d;
      cmd.targetX = gxState.AX;
    }
  }

  if (h & (1 << 5)) { // Y
    x = 0;
    currentJobFile.read((uint8_t*)&x, gxState.xySize);
    cmd.seen_Y = 1;
    if (cmd.seen_M) {
      // M-code parameter, consume but do not update path state
      // float val = float(x - gxState.xyLimit) * 100 / gxState.xyScale;
    } else {
      d = float(x - gxState.xyLimit) / gxState.xyScale;
      gxState.AY += d;
      cmd.targetY = gxState.AY;
    }
  }

  if (h & (1 << 6)) { // Z
    x = 0;
    currentJobFile.read((uint8_t*)&x, gxState.zSize);
    cmd.seen_Z = 1;
    d = float(x - gxState.zLimit) / gxState.zScale;

    if (cmd.G == 28) {
      // G28 parameter usage depends on implementation, but in ref it does NOT update path accumulator
      cmd.targetZ = d;
    } else {
      gxState.AZ += d;
      cmd.targetZ = gxState.AZ;
    }
  }

  if (h & (1 << 7)) { // E
    x = 0;
    currentJobFile.read((uint8_t*)&x, gxState.eSize);
    cmd.seen_E = 1;
    // Consume E bytes but ignore functionality for now
  }

  if (cmd.seen_S) {
    setToolPower(cmd.power);
  }

  if (cmd.seen_G && (cmd.G == 0 || cmd.G == 1)) {
    if (previewFirstMove) {
      previewMinX = previewMaxX = cmd.targetX;
      previewMinY = previewMaxY = cmd.targetY;
      previewFirstMove = false;
    } else {
      if (cmd.targetX < previewMinX) previewMinX = cmd.targetX;
      if (cmd.targetX > previewMaxX) previewMaxX = cmd.targetX;
      if (cmd.targetY < previewMinY) previewMinY = cmd.targetY;
      if (cmd.targetY > previewMaxY) previewMaxY = cmd.targetY;
    }
    if (!isPreviewMode) {
      float feed = cmd.G == 0 ? G0Feed : G1Feed;
      int isG0 = cmd.G == 0 ? 1 : 0;
      // log0("G%d X%f Y%f Z%f\n", cmd.G, gxState.AX, gxState.AY, gxState.AZ);
      addmove(feed, gxState.AX, gxState.AY, gxState.AZ, currentToolPower, isG0, 0, false);
      // log0("G%d X%f Y%f Z%f\n", cmd.G, cmd.targetX, cmd.targetY, cmd.targetZ);
      // addmove(feed, cmd.targetX, cmd.targetY, cmd.targetZ, currentToolPower, isG0, 0);
    }
  } else if (cmd.G == 28) {
    bool homeX = cmd.seen_X;
    bool homeY = cmd.seen_Y;
    bool homeZ = cmd.seen_Z;

    if (!homeX && !homeY && !homeZ) {
      startHoming(3);
    } else if (!isPreviewMode) {
       // Support specific axis homing if G28 has params?
       // Currently generic code just does startHoming(3) for G28 without params.
       // Ref does: next_target.target.axis[nZ] = d; -> implying it moves TO that position?
       // But usually G28 is "Go to Home".
       // If G28 is used with coordinates, it might mean "Move to intermediate point then home".
       // For now, keep existing behavior for no-param G28.
    }
  }

  jobBytesRead = currentJobFile.position();
}

void readTextGCodeLine() {
  if (!currentJobFile.available()) {
    isParsingComplete = true;
    completeParse();
    currentJobFile.close();
    return;
  }

  String line = currentJobFile.readStringUntil('\n');
  jobBytesRead += line.length() + 1;

  line.trim();
  line.toUpperCase();

  if (line.length() == 0 || line.startsWith(";")) return;

  if (line.startsWith("M3")) {
    int newPower = 0;
    for (int i = 0; i < line.length(); i++) {
      char c = line.charAt(i);
      if (c == 'S') {
        int start = i + 1;
        int end = start;
        bool hasDigit = false;
        while (end < line.length() && (isdigit(line.charAt(end)) || line.charAt(end) == '-')) {
          if (isdigit(line.charAt(end))) hasDigit = true;
          end++;
        }
        if (hasDigit) {
          newPower = line.substring(start, end).toInt();
        }
        break;
      }
    }
    if (newPower >= 0 && newPower <= 255) {
      setToolPower(newPower);
    }
    return;
  }

  static int gcodeMode = 0;

  if (line.startsWith("G90")) {
    gcodeMode = 0;
    return;
  }

  if (line.startsWith("G91")) {
    gcodeMode = 1;
    return;
  }

  if (line.startsWith("G28")) {
    bool homeX = false;
    bool homeY = false;
    bool homeZ = false;

    for (int i = 0; i < line.length(); i++) {
      char c = line.charAt(i);
      if (c == 'X') homeX = true;
      else if (c == 'Y') homeY = true;
      else if (c == 'Z') homeZ = true;
    }

    if (!homeX && !homeY && !homeZ) {
      startHoming(3);
    } else {
      if (homeX) startHoming(0);
      if (homeY) startHoming(1);
      if (homeZ) startHoming(2);
    }
    return;
  }

  if (line.startsWith("G0") || line.startsWith("G1")) {
    float targetX = cx1;
    float targetY = cy1;
    float targetZ = cz1;
    int isG0 = line.startsWith("G0") ? 1 : 0;
    float feed = isG0 ? G0Feed : G1Feed;
    bool hasF = false;
    bool hasS = false;
    int power = currentToolPower;

    for (int i = 0; i < line.length(); i++) {
      char c = line.charAt(i);
      if (c == 'X' || c == 'Y' || c == 'Z' || c == 'F' || c == 'S') {
        int start = i + 1;
        int end = start;
        bool hasDigit = false;
        while (end < line.length() && (isdigit(line.charAt(end)) || line.charAt(end) == '.' || line.charAt(end) == '-')) {
          if (isdigit(line.charAt(end))) hasDigit = true;
          end++;
        }

        if (hasDigit) {
          if (c == 'X') targetX = line.substring(start, end).toFloat();
          else if (c == 'Y') targetY = line.substring(start, end).toFloat();
          else if (c == 'Z') targetZ = line.substring(start, end).toFloat();
          else if (c == 'F') {
            feed = line.substring(start, end).toFloat() / 60.0;
            hasF = true;
          } else if (c == 'S') {
            power = line.substring(start, end).toInt();
            hasS = true;
          }
        }
      }
    }

    if (hasS) {
      setToolPower(power);
    }

    if (hasF) {
      if (isG0) {
        G0Feed = feed;
      } else {
        G1Feed = feed;
      }
    }

    if (previewFirstMove) {
      previewMinX = previewMaxX = targetX;
      previewMinY = previewMaxY = targetY;
      previewFirstMove = false;
    } else {
      if (targetX < previewMinX) previewMinX = targetX;
      if (targetX > previewMaxX) previewMaxX = targetX;
      if (targetY < previewMinY) previewMinY = targetY;
      if (targetY > previewMaxY) previewMaxY = targetY;
    }
    if (!isPreviewMode) {
      addmove(feed, targetX, targetY, targetZ, currentToolPower, isG0, gcodeMode, false);
    }
  }
}

void parseNextGCodeSegments() {
  if (!jobRunning || jobPaused || isParsingComplete) return;
  if (bufffull || isHoming) return;
  // int ctr=2;
  // while (ctr--) {
    if (isGxMode) {
      readGxLine();
    } else {
      readTextGCodeLine();
    }
  //   if (bufffull || isParsingComplete) break;
  // }
}
String lastJobName="";
float lastPx,lastPy,parseCx1,parseCy1;
String jobsize;
void previewJob(String filename,bool perform) {
  if (jobRunning) return;
  if (lastJobName!=filename) {
    String filepath = "/" + filename;
    currentJobFile = LittleFS.open(filepath, "r");
    if (!currentJobFile) {
      return;
    }

    // Open shapes file for recording during preview

    isPreviewMode = true;
    previewFirstMove = true;
    parseCx1 = cx1;
    parseCy1 = cy1;
    lastPx = previewMinX = previewMaxX = cx1;
    lastPy = previewMinY = previewMaxY = cy1;

    if (isGxFormat(currentJobFile)) {
      isGxMode = true;
      char hdr[3];
      currentJobFile.read((uint8_t*)&hdr, 3);
      int version = hdr[2];
      initGxParser(version);

      while (currentJobFile.available() && !isParsingComplete) {
        readGxLine();
        yield(); // Mencegah Watchdog Reset (WDT) saat memproses file besar
      }
    } else {
      isGxMode = false;
      currentJobFile.seek(0);

      while (currentJobFile.available() && !isParsingComplete) {
        readTextGCodeLine();
        yield(); // Mencegah Watchdog Reset (WDT) saat memproses file besar
      }
    }

    isParsingComplete = false;
    currentJobFile.close();
    completeParse();
    lastJobName=filename;
    jobsize = "Size " + String(lastJobSize[0], 1) + "x" + String(lastJobSize[1], 1);
  }

  // Update offset references to current machine position and ensure preview mode is active
  isPreviewMode = true;
  lastPx = cx1;
  lastPy = cy1;

  if (perform) {
    float xasal = cx1;
    float yasal = cy1;
    float currZ = cz1;
    float testD=0.1;
    float znaik=systemCfg.testLaserTime*cfg[2].vel*0.001; // hitung z naik memakai kecepatan axis Z dan jarak menjadi systemCfg.testLaserTime
    if (toolCfg.mode==0) znaik=10; // 0 : router
    log0("Z:%f\n",znaik)
    int p=systemCfg.testLaserPower;
    int G1Feed=100;
    int G0Feed=100;
    // log0("%f %f\n",previewMinX,previewMinY);
    float offsetX = cx1 - parseCx1;
    float offsetY = cy1 - parseCy1;
    float x1=(previewMinX+offsetX)-lastPx;
    float y1=(previewMinY+offsetY)-lastPy;
    float width=lastJobSize[0];
    float height=lastJobSize[1];
    addmove(G0Feed, x1, y1, znaik, 0, 1, 1, false);
    addmove(G1Feed, testD, 0,   0, p, 0, 1, false);
    addmove(G0Feed, width, 0,   0, 0, 1, 1,false);

    if (toolCfg.mode==1) addmove(G1Feed, 0, testD, -znaik, p, 0, 1, false);
    addmove(G0Feed, 0, height, 0, 0, 1, 1,false);

    if (toolCfg.mode==1) addmove(G1Feed, -testD, 0, znaik, p, 0, 1, false);
    addmove(G0Feed, -width, 0, 0, 0, 1, 1,false);

    if (toolCfg.mode==1) addmove(G1Feed, 0, -testD, -znaik, p, 0, 1, false);
    addmove(G0Feed, 0, -height, 0, 0, 1, 1,false);

    addmove(G0Feed, xasal, yasal, currZ, 0, 1, 0, false);
    // extern void LOADINFO();
    // LOADINFO();
  }
}

int getShapeCount() {
  File f = LittleFS.open("/shapes.dat", "r");
  if (!f) return 0;
  int count = f.size() / sizeof(ShapeRecord);
  f.close();
  return count;
}

bool getShape(int index, ShapeRecord &rec) {
  File f = LittleFS.open("/shapes.dat", "r");
  if (!f) return false;
  if (index * sizeof(ShapeRecord) >= f.size()) {
    f.close();
    return false;
  }
  f.seek(index * sizeof(ShapeRecord));
  f.read((uint8_t*)&rec, sizeof(ShapeRecord));
  f.close();
  return true;
}
