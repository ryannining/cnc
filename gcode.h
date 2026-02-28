#ifndef GCODE_H
#define GCODE_H

#include <Arduino.h>

#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <LittleFS.h>
#elif defined(ESP32)
  #include <WiFi.h>
  #include <LittleFS.h>
#endif

struct GXParserState {
  int version;
  int xySize, zSize, eSize;
  int xyLimit, zLimit, eLimit;
  float xyScale, zScale, eScale, fScale;
  float AX, AY, AZ, AE;
  bool initialized;
};

struct ShapeRecord {
  uint32_t filePos;
  float x, y, z;
};

static bool canUseSerial = false;
extern File currentJobFile;
extern size_t jobFileSize;
extern size_t jobBytesRead;
extern bool isParsingComplete;
extern float G0Feed;
extern float G1Feed;
extern float cx1, cy1, cz1;
extern int currentToolPower;

extern bool isGxMode;
extern GXParserState gxState;

bool openGxFile(String filepath);
bool isGxFormat(File& file);
void initGxParser(int version);
void readGxLine();
void readTextGCodeLine();
void parseNextGCodeSegments();
void processHoming();
int32_t startmove();
void previewJob(String filename,bool perform);
int scanShapes(String filename);
int getShapeCount();
bool getShape(int index, ShapeRecord &rec);
#endif
