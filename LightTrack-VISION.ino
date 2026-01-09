// LightTrack VISION with rtpMIDI + OSC Support
// Sends radar tracking data as MIDI CC and/or OSC messages over the network

#include <Arduino.h>
#include <LD2450.h>
#include <FastLED.h>
#include <EEPROM.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <time.h>
#include <ArduinoOTA.h>
#include <stdlib.h>
#include "esp_wifi.h"
#include <math.h>
#include <utility>

// --- rtpMIDI (AppleMIDI) ---
#include <AppleMIDI.h>

// --- OSC ---
#include <OSCMessage.h>
#include <OSCBundle.h>

// --- PIN CONFIGURATION ---
const int RX1_PIN = D7;
const int TX1_PIN = D6;
const int LED_PIN = D10;

// --- RADAR INSTANCE ---
LD2450 radar;

// --- LED STRIP CONFIGURATION ---
#define MAX_LED_DENSITY 60
#define STRIP_LENGTH    3
#define MAX_NUM_LEDS    (MAX_LED_DENSITY * STRIP_LENGTH)
#define LED_TYPE        WS2812B
#define COLOR_ORDER     GRB
CRGB leds[MAX_NUM_LEDS];

// --- CURRENT STRIP VARIABLES ---
int currentLedDensity = 60;
int currentNumLeds = (currentLedDensity * STRIP_LENGTH);

// --- TARGET TRACKING CONFIGURATION ---
#define MAX_TARGETS 3
#define TARGET_TIMEOUT 2000
#define GHOST_FILTER_TIMEOUT 5000 
#define GHOST_REMOVAL_TIMEOUT 10000 
#define MERGE_DISTANCE 500 

// --- RADAR PROCESSING RANGE ---
#define MIN_PROCESS_DISTANCE  150
#define MAX_PROCESS_DISTANCE  8000

// --- CALIBRATION TABLES ---
#define CALIBRATION_POINTS 16
const int calibration30[CALIBRATION_POINTS][2] = {
  {150,   5}, {250,   8}, {500,  15}, {750,  23}, {1000,  30}, {1500,  45},
  {2000,  60}, {2500,  75}, {3000,  90}, {4000, 120}, {5000, 150}, {6000, 180},
  {7000, 210}, {8000, 240}, {8500, 255}, {9000, 270}
};
const int calibration60[CALIBRATION_POINTS][2] = {
  {150,   10}, {250,  16}, {500,  30}, {750,  46}, {1000,  60}, {1500,  90},
  {2000, 120}, {2500, 150}, {3000, 180}, {4000, 240}, {5000, 300}, {6000, 360},
  {7000, 420}, {8000, 480}, {8500, 540}, {9000, 599}
};

// --- INACTIVE ZONES ---
#define MAX_INACTIVE_ZONES 12
struct Point { int x; int y; };
struct InactiveZone { bool enabled; Point corners[4]; };
InactiveZone inactiveZones[MAX_INACTIVE_ZONES];

// --- Individual Target Settings Structure ---
struct TargetSetting { bool enabled; CRGB color; };

// --- NETWORK CONFIGURATION ---
struct NetworkConfig {
  bool enabled;
  char wifiSSID[33];
  char wifiPassword[65];
};

// --- MIDI CONFIGURATION ---
struct MidiConfig {
  bool enabled;
  char sessionName[33];
  uint8_t midiChannel;
  uint8_t ccX;
  uint8_t ccY;
  uint8_t ccDistance;
  uint8_t ccSpeed;
  bool sendSmoothed;        // Send smoothed values instead of raw
  float smoothingFactor;    // 0.0-1.0, lower = more smoothing
  int16_t xMin, xMax;
  int16_t yMin, yMax;
  int16_t distMin, distMax;
  int16_t speedMin, speedMax;
  uint8_t sendInterval;
};

// --- OSC CONFIGURATION ---
struct OscConfig {
  bool enabled;
  char targetHost[64];      // IP or hostname to send OSC to
  uint16_t targetPort;      // UDP port (default 8000)
  uint16_t localPort;       // Local receive port (optional, for bidirectional)
  char addressPrefix[32];   // OSC address prefix, e.g., "/radar" or "/lighttrack"
  bool sendBundle;          // Send as bundle (all values together) or individual messages
  bool normalizeValues;     // Send normalized 0.0-1.0 or raw mm values
  bool sendSmoothed;        // Send Kalman-filtered/smoothed values instead of raw
  float smoothingFactor;    // 0.0-1.0, lower = more smoothing (default 0.15)
  int16_t xMin, xMax;
  int16_t yMin, yMax;
  int16_t distMin, distMax;
  int16_t speedMin, speedMax;
  uint8_t sendInterval;
};

NetworkConfig networkConfig = {
  .enabled = false,
  .wifiSSID = "",
  .wifiPassword = ""
};

MidiConfig midiConfig = {
  .enabled = false,
  .sessionName = "LightTrack",
  .midiChannel = 1,
  .ccX = 1,
  .ccY = 2,
  .ccDistance = 3,
  .ccSpeed = 4,
  .sendSmoothed = true,
  .smoothingFactor = 0.15f,
  .xMin = -4000,
  .xMax = 4000,
  .yMin = 0,
  .yMax = 6000,
  .distMin = 0,
  .distMax = 6000,
  .speedMin = -500,
  .speedMax = 500,
  .sendInterval = 50
};

OscConfig oscConfig = {
  .enabled = false,
  .targetHost = "192.168.1.100",
  .targetPort = 8000,
  .localPort = 9000,
  .addressPrefix = "/radar",
  .sendBundle = true,
  .normalizeValues = true,
  .sendSmoothed = true,        // Default to smoothed values
  .smoothingFactor = 0.15f,    // Lower = smoother (0.1-0.3 recommended)
  .xMin = -4000,
  .xMax = 4000,
  .yMin = 0,
  .yMax = 6000,
  .distMin = 0,
  .distMax = 6000,
  .speedMin = -500,
  .speedMax = 500,
  .sendInterval = 20
};

// --- MIDI STATE ---
APPLEMIDI_CREATE_DEFAULTSESSION_INSTANCE();
bool midiConnected = false;
bool wifiSTAConnected = false;
unsigned long lastMidiSend = 0;
uint8_t lastCCValues[MAX_TARGETS][4] = {{255, 255, 255, 255}, {255, 255, 255, 255}, {255, 255, 255, 255}};

// --- OSC STATE ---
WiFiUDP oscUdp;
unsigned long lastOscSend = 0;
IPAddress oscTargetIP;
bool oscTargetResolved = false;

// --- GLOBAL SETTINGS ---
const float MAX_BRIGHTNESS_SCALER = 0.7f;
int ledUpdateInterval = 20;
int movementSensitivity = 1;
int wifiTimeoutMinutes = 0;
float movingIntensity = 0.5f * MAX_BRIGHTNESS_SCALER;
float stationaryIntensity = 0.04f * MAX_BRIGHTNESS_SCALER;
int movingLength = 20;
int gradientSoftness = 4;
int ledOffDelay = 5;
int centerShift = 0;
int additionalLEDs = 75;
bool backgroundModeActive = false;
int maxActiveTargets = 1;
TargetSetting targetSettings[MAX_TARGETS] = { 
  {true, CRGB(255, 200, 50)},   // Target 1: warm white
  {true, CRGB(50, 200, 255)},   // Target 2: cyan
  {true, CRGB(255, 50, 200)}    // Target 3: magenta
};

// --- EEPROM ---
#define EEPROM_SIZE 2560
#define EEPROM_VERSION 0xB5  // Bumped for radar view debugging

// --- TARGET STRUCTURE ---
struct Target {
  bool present;
  int x, y, distance, speed;  // Raw sensor values
  unsigned long lastSeenTime, lastMovementTime;
  float smoothedLEDPosition, velocityLED;
  float p_pos, p_vel;
  float currentBrightness;
  int lastMovementDirection;
  bool isInitialized;
  // Smoothed values for OSC/MIDI output
  float smoothedX, smoothedY, smoothedDistance, smoothedSpeed;
  bool smoothedInitialized;
};
Target targets[MAX_TARGETS];

// --- WORKING VARIABLES ---
bool stripInitialized = false;
int startHour = 20, startMinute = 0;
int endHour = 8, endMinute = 0;
bool lightOn = true;
unsigned long lastTimeCheck = 0;
volatile bool smarthomeOverride = false;
volatile int clientTimezoneOffsetMinutes = 0;
volatile bool isTimeOffsetSet = false;
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
TaskHandle_t sensorTaskHandle = NULL, ledTaskHandle = NULL, webServerTaskHandle = NULL, networkTaskHandle = NULL;
unsigned long wifiStartTime = 0;
bool wifiActive = true;

// --- FUNCTION PROTOTYPES ---
void sensorTask(void* parameter);
void ledTask(void* parameter);
void webServerTask(void* parameter);
void networkTask(void* parameter);
void loadSettings();
void saveSettings();
bool initializeStrip();
int mapDistanceToLED(int rawDistance);
bool isTargetInInactiveZone(int targetX, int targetY);
void setupWiFi();
void setupOTA();
void setupMIDI();
void setupOSC();
void sendMidiCC();
void sendOscData();
uint8_t mapToMidi(int value, int minVal, int maxVal);
float mapToNormalized(int value, int minVal, int maxVal);
void handleRoot();
void handleNetworkSettings();
void handleSetNetworkConfig();
void handleGetNetworkStatus();
void handleSetMovingIntensity();
void handleSetStationaryIntensity();
void handleSetMovingLength();
void handleSetAdditionalLEDs();
void handleSetCenterShift();
void handleSetLedDensity();
void handleSetGradientSoftness();
void handleSetLedOffDelay();
void handleSetTime();
void handleSetSchedule();
void handleNotFound();
void handleSmartHomeOn();
void handleSmartHomeOff();
void handleSmartHomeClear();
void handleToggleBackgroundMode();
void handleGetCurrentTime();
void handleRadarView();
void handleSetTargetSettings();
void webSocketEvent(uint8_t, WStype_t, uint8_t*, size_t);
void broadcastRadarData();
void updateTime();
void checkWiFiTimeout();

// ------------------------- Setup -------------------------
void setup() {
  setCpuFrequencyMhz(80);  // Keep at 80MHz for stable serial timing with LD2450
  
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n--- setup() begins ---");
  Serial.println("LightTrack VISION + rtpMIDI + OSC");
  Serial.printf("CPU Frequency: %d MHz\n", getCpuFrequencyMhz());
  Serial.println("=======================================================");

  randomSeed(ESP.getEfuseMac());

  for (auto & target : targets) {
    target.present = false;
    target.isInitialized = false;
    target.smoothedLEDPosition = 0;
    target.velocityLED = 0.0f;
    target.p_pos = 1000.0f;
    target.p_vel = 1000.0f;
    target.lastMovementTime = 0;
    target.currentBrightness = 0.0f;
    target.lastMovementDirection = 0;
    // Initialize smoothed output values
    target.smoothedX = 0.0f;
    target.smoothedY = 0.0f;
    target.smoothedDistance = 0.0f;
    target.smoothedSpeed = 0.0f;
    target.smoothedInitialized = false;
  }

  for (auto & zone : inactiveZones) {
    zone.enabled = false;
  }
  
  if (!SPIFFS.begin(true)) { Serial.println("!!! SPIFFS mount failed"); }

  loadSettings();

  Serial.println("Initializing LD2450 Radar...");
  
  // Make sure Serial1 is clean before starting
  Serial1.end();
  delay(100);
  
  // Initialize Serial1 for LD2450 (256000 baud required)
  Serial1.begin(256000, SERIAL_8N1, RX1_PIN, TX1_PIN);
  delay(500);
  
  // Flush any garbage data
  while (Serial1.available()) {
    Serial1.read();
  }
  delay(200);
  
  // Initialize radar library
  radar.begin(Serial1);
  delay(500);
  
  // Flush again after begin
  while (Serial1.available()) {
    Serial1.read();
  }
  
  Serial.println("Radar initialized. Waiting for first frames...");
  
  // Try to get a few frames to confirm it's working
  int testFrames = 0;
  for (int i = 0; i < 50 && testFrames < 3; i++) {
    if (radar.read() > 0) {
      testFrames++;
      Serial.printf("  Got frame %d\n", testFrames);
    }
    delay(100);
  }
  if (testFrames > 0) {
    Serial.printf("Radar OK - received %d test frames\n", testFrames);
  } else {
    Serial.println("WARNING: No radar frames received during init test!");
    Serial.println("  Check wiring: LD2450 TX -> D7 (RX), LD2450 RX -> D6 (TX)");
  }

  setupWiFi();
  wifiStartTime = millis();
  configTzTime("UTC0", "pool.ntp.org", "time.nist.gov");
  setupOTA();
  stripInitialized = initializeStrip();

  Serial.println("Starting RTOS tasks...");
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 5, &sensorTaskHandle, 0);
  xTaskCreatePinnedToCore(ledTask, "LEDTask", 8192, NULL, 4, &ledTaskHandle, 1);
  xTaskCreatePinnedToCore(webServerTask, "WebServerTask", 8192, NULL, 3, &webServerTaskHandle, 0);
  xTaskCreatePinnedToCore(networkTask, "NetworkTask", 6144, NULL, 4, &networkTaskHandle, 0);

  Serial.println("--------------------------------------");
  Serial.println("Setup complete. System is running.");
  Serial.print("Config AP: http://"); Serial.println(WiFi.softAPIP());
  if (wifiSTAConnected) {
    Serial.print("Network IP: http://"); Serial.println(WiFi.localIP());
  }
  Serial.println("--------------------------------------");
}

void loop() {
  updateTime();
  checkWiFiTimeout();
  vTaskDelay(pdMS_TO_TICKS(5000));
}

// --- MIDI FUNCTIONS ---
uint8_t mapToMidi(int value, int minVal, int maxVal) {
  if (minVal == maxVal) return 64;
  value = constrain(value, minVal, maxVal);
  return (uint8_t)map(value, minVal, maxVal, 0, 127);
}

float mapToNormalized(int value, int minVal, int maxVal) {
  if (minVal == maxVal) return 0.5f;
  float normalized = (float)(value - minVal) / (float)(maxVal - minVal);
  return constrain(normalized, 0.0f, 1.0f);
}

void setupNetworkConnection() {
  if (!networkConfig.enabled || strlen(networkConfig.wifiSSID) == 0) {
    Serial.println("[Network] Disabled or no WiFi configured");
    return;
  }

  Serial.printf("[Network] Connecting to WiFi: %s\n", networkConfig.wifiSSID);
  
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(networkConfig.wifiSSID, networkConfig.wifiPassword);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiSTAConnected = true;
    Serial.println();
    Serial.printf("[Network] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[Network] WiFi connection failed!");
    wifiSTAConnected = false;
  }
}

void setupMIDI() {
  if (!midiConfig.enabled || !wifiSTAConnected) {
    Serial.println("[MIDI] Disabled or no network");
    return;
  }

  MIDI.begin(MIDI_CHANNEL_OMNI);
  AppleMIDI.setName(midiConfig.sessionName);
  
  AppleMIDI.setHandleConnected([](const APPLEMIDI_NAMESPACE::ssrc_t & ssrc, const char* name) {
    midiConnected = true;
    Serial.printf("[rtpMIDI] Connected to session: %s\n", name);
  });
  
  AppleMIDI.setHandleDisconnected([](const APPLEMIDI_NAMESPACE::ssrc_t & ssrc) {
    midiConnected = false;
    Serial.println("[rtpMIDI] Disconnected");
  });
  
  Serial.printf("[MIDI] rtpMIDI session '%s' started on port 5004\n", midiConfig.sessionName);
}

void setupOSC() {
  if (!oscConfig.enabled || !wifiSTAConnected) {
    Serial.println("[OSC] Disabled or no network");
    return;
  }

  // Resolve target host
  if (WiFi.hostByName(oscConfig.targetHost, oscTargetIP)) {
    oscTargetResolved = true;
    Serial.printf("[OSC] Target resolved: %s -> %s:%d\n", 
                  oscConfig.targetHost, oscTargetIP.toString().c_str(), oscConfig.targetPort);
  } else {
    // Try parsing as IP directly
    if (oscTargetIP.fromString(oscConfig.targetHost)) {
      oscTargetResolved = true;
      Serial.printf("[OSC] Target IP: %s:%d\n", oscTargetIP.toString().c_str(), oscConfig.targetPort);
    } else {
      oscTargetResolved = false;
      Serial.printf("[OSC] Failed to resolve: %s\n", oscConfig.targetHost);
    }
  }

  // Start UDP for receiving (optional)
  if (oscConfig.localPort > 0) {
    oscUdp.begin(oscConfig.localPort);
    Serial.printf("[OSC] Listening on port %d\n", oscConfig.localPort);
  }
}

void sendMidiCC() {
  if (!midiConfig.enabled || !wifiSTAConnected) return;
  
  unsigned long now = millis();
  if (now - lastMidiSend < midiConfig.sendInterval) return;
  lastMidiSend = now;
  
  // Send data for each target on consecutive MIDI channels
  for (int t = 0; t < MAX_TARGETS; t++) {
    if (!targets[t].present) continue;
    
    // Each target gets its own channel: base, base+1, base+2 (wrapping at 16)
    uint8_t channel = ((midiConfig.midiChannel - 1 + t) % 16) + 1;
    
    // Choose between raw and smoothed values
    int valX_raw, valY_raw, valDist_raw, valSpeed_raw;
    if (midiConfig.sendSmoothed && targets[t].smoothedInitialized) {
      valX_raw = (int)targets[t].smoothedX;
      valY_raw = (int)targets[t].smoothedY;
      valDist_raw = (int)targets[t].smoothedDistance;
      valSpeed_raw = (int)targets[t].smoothedSpeed;
    } else {
      valX_raw = targets[t].x;
      valY_raw = targets[t].y;
      valDist_raw = targets[t].distance;
      valSpeed_raw = targets[t].speed;
    }
    
    uint8_t valX = mapToMidi(valX_raw, midiConfig.xMin, midiConfig.xMax);
    uint8_t valY = mapToMidi(valY_raw, midiConfig.yMin, midiConfig.yMax);
    uint8_t valDist = mapToMidi(valDist_raw, midiConfig.distMin, midiConfig.distMax);
    uint8_t valSpeed = mapToMidi(valSpeed_raw, midiConfig.speedMin, midiConfig.speedMax);
    
    if (valX != lastCCValues[t][0]) {
      MIDI.sendControlChange(midiConfig.ccX, valX, channel);
      lastCCValues[t][0] = valX;
    }
    if (valY != lastCCValues[t][1]) {
      MIDI.sendControlChange(midiConfig.ccY, valY, channel);
      lastCCValues[t][1] = valY;
    }
    if (valDist != lastCCValues[t][2]) {
      MIDI.sendControlChange(midiConfig.ccDistance, valDist, channel);
      lastCCValues[t][2] = valDist;
    }
    if (valSpeed != lastCCValues[t][3]) {
      MIDI.sendControlChange(midiConfig.ccSpeed, valSpeed, channel);
      lastCCValues[t][3] = valSpeed;
    }
  }
}

void sendOscData() {
  if (!oscConfig.enabled || !wifiSTAConnected || !oscTargetResolved) return;
  
  unsigned long now = millis();
  if (now - lastOscSend < oscConfig.sendInterval) return;
  lastOscSend = now;

  // Count present targets
  int targetCount = 0;
  for (int t = 0; t < MAX_TARGETS; t++) {
    if (targets[t].present) targetCount++;
  }

  // Debug output every second
  static unsigned long lastDebug = 0;
  if (now - lastDebug > 1000) {
    lastDebug = now;
    Serial.printf("[OSC DEBUG] targets:%d smoothed:%d", targetCount, oscConfig.sendSmoothed);
    for (int t = 0; t < MAX_TARGETS; t++) {
      if (targets[t].present) {
        float valX = oscConfig.sendSmoothed ? targets[t].smoothedX : (float)targets[t].x;
        float valY = oscConfig.sendSmoothed ? targets[t].smoothedY : (float)targets[t].y;
        Serial.printf(" | T%d X:%.0f Y:%.0f", t+1, valX, valY);
      }
    }
    Serial.println();
  }

  // Build address strings for count
  char addrCount[64];
  snprintf(addrCount, sizeof(addrCount), "%s/count", oscConfig.addressPrefix);

  if (oscConfig.sendBundle) {
    // Send as OSC bundle (all targets in one packet)
    OSCBundle bundle;
    
    // Send target count
    bundle.add(addrCount).add((int32_t)targetCount);
    
    // Send data for each target
    for (int t = 0; t < MAX_TARGETS; t++) {
      char addrPresent[64], addrX[64], addrY[64], addrDist[64], addrSpeed[64], addrAll[64];
      snprintf(addrPresent, sizeof(addrPresent), "%s/%d/present", oscConfig.addressPrefix, t+1);
      snprintf(addrX, sizeof(addrX), "%s/%d/x", oscConfig.addressPrefix, t+1);
      snprintf(addrY, sizeof(addrY), "%s/%d/y", oscConfig.addressPrefix, t+1);
      snprintf(addrDist, sizeof(addrDist), "%s/%d/distance", oscConfig.addressPrefix, t+1);
      snprintf(addrSpeed, sizeof(addrSpeed), "%s/%d/speed", oscConfig.addressPrefix, t+1);
      snprintf(addrAll, sizeof(addrAll), "%s/%d/all", oscConfig.addressPrefix, t+1);
      
      bundle.add(addrPresent).add((int32_t)(targets[t].present ? 1 : 0));
      
      if (targets[t].present) {
        float valX, valY, valDist, valSpeed;
        if (oscConfig.sendSmoothed && targets[t].smoothedInitialized) {
          valX = targets[t].smoothedX;
          valY = targets[t].smoothedY;
          valDist = targets[t].smoothedDistance;
          valSpeed = targets[t].smoothedSpeed;
        } else {
          valX = (float)targets[t].x;
          valY = (float)targets[t].y;
          valDist = (float)targets[t].distance;
          valSpeed = (float)targets[t].speed;
        }
        
        if (oscConfig.normalizeValues) {
          float normX = mapToNormalized((int)valX, oscConfig.xMin, oscConfig.xMax);
          float normY = mapToNormalized((int)valY, oscConfig.yMin, oscConfig.yMax);
          float normDist = mapToNormalized((int)valDist, oscConfig.distMin, oscConfig.distMax);
          float normSpeed = mapToNormalized((int)valSpeed, oscConfig.speedMin, oscConfig.speedMax);
          
          bundle.add(addrX).add(normX);
          bundle.add(addrY).add(normY);
          bundle.add(addrDist).add(normDist);
          bundle.add(addrSpeed).add(normSpeed);
          bundle.add(addrAll).add(normX).add(normY).add(normDist).add(normSpeed);
        } else {
          bundle.add(addrX).add(valX);
          bundle.add(addrY).add(valY);
          bundle.add(addrDist).add(valDist);
          bundle.add(addrSpeed).add(valSpeed);
          bundle.add(addrAll).add(valX).add(valY).add(valDist).add(valSpeed);
        }
      }
    }
    
    oscUdp.beginPacket(oscTargetIP, oscConfig.targetPort);
    bundle.send(oscUdp);
    oscUdp.endPacket();
    bundle.empty();
    
  } else {
    // Send individual messages
    OSCMessage msgCount(addrCount);
    msgCount.add((int32_t)targetCount);
    oscUdp.beginPacket(oscTargetIP, oscConfig.targetPort);
    msgCount.send(oscUdp);
    oscUdp.endPacket();
    msgCount.empty();
    
    for (int t = 0; t < MAX_TARGETS; t++) {
      char addrPresent[64], addrX[64], addrY[64], addrDist[64], addrSpeed[64];
      snprintf(addrPresent, sizeof(addrPresent), "%s/%d/present", oscConfig.addressPrefix, t+1);
      snprintf(addrX, sizeof(addrX), "%s/%d/x", oscConfig.addressPrefix, t+1);
      snprintf(addrY, sizeof(addrY), "%s/%d/y", oscConfig.addressPrefix, t+1);
      snprintf(addrDist, sizeof(addrDist), "%s/%d/distance", oscConfig.addressPrefix, t+1);
      snprintf(addrSpeed, sizeof(addrSpeed), "%s/%d/speed", oscConfig.addressPrefix, t+1);
      
      OSCMessage msgPresent(addrPresent);
      msgPresent.add((int32_t)(targets[t].present ? 1 : 0));
      oscUdp.beginPacket(oscTargetIP, oscConfig.targetPort);
      msgPresent.send(oscUdp);
      oscUdp.endPacket();
      msgPresent.empty();
      
      if (targets[t].present) {
        float valX, valY, valDist, valSpeed;
        if (oscConfig.sendSmoothed && targets[t].smoothedInitialized) {
          valX = targets[t].smoothedX;
          valY = targets[t].smoothedY;
          valDist = targets[t].smoothedDistance;
          valSpeed = targets[t].smoothedSpeed;
        } else {
          valX = (float)targets[t].x;
          valY = (float)targets[t].y;
          valDist = (float)targets[t].distance;
          valSpeed = (float)targets[t].speed;
        }
        
        if (oscConfig.normalizeValues) {
          OSCMessage msgX(addrX); msgX.add((float)mapToNormalized((int)valX, oscConfig.xMin, oscConfig.xMax));
          oscUdp.beginPacket(oscTargetIP, oscConfig.targetPort); msgX.send(oscUdp); oscUdp.endPacket(); msgX.empty();
          
          OSCMessage msgY(addrY); msgY.add((float)mapToNormalized((int)valY, oscConfig.yMin, oscConfig.yMax));
          oscUdp.beginPacket(oscTargetIP, oscConfig.targetPort); msgY.send(oscUdp); oscUdp.endPacket(); msgY.empty();
          
          OSCMessage msgDist(addrDist); msgDist.add((float)mapToNormalized((int)valDist, oscConfig.distMin, oscConfig.distMax));
          oscUdp.beginPacket(oscTargetIP, oscConfig.targetPort); msgDist.send(oscUdp); oscUdp.endPacket(); msgDist.empty();
          
          OSCMessage msgSpeed(addrSpeed); msgSpeed.add((float)mapToNormalized((int)valSpeed, oscConfig.speedMin, oscConfig.speedMax));
          oscUdp.beginPacket(oscTargetIP, oscConfig.targetPort); msgSpeed.send(oscUdp); oscUdp.endPacket(); msgSpeed.empty();
        } else {
          OSCMessage msgX(addrX); msgX.add(valX);
          oscUdp.beginPacket(oscTargetIP, oscConfig.targetPort); msgX.send(oscUdp); oscUdp.endPacket(); msgX.empty();
          
          OSCMessage msgY(addrY); msgY.add(valY);
          oscUdp.beginPacket(oscTargetIP, oscConfig.targetPort); msgY.send(oscUdp); oscUdp.endPacket(); msgY.empty();
          
          OSCMessage msgDist(addrDist); msgDist.add(valDist);
          oscUdp.beginPacket(oscTargetIP, oscConfig.targetPort); msgDist.send(oscUdp); oscUdp.endPacket(); msgDist.empty();
          
          OSCMessage msgSpeed(addrSpeed); msgSpeed.add(valSpeed);
          oscUdp.beginPacket(oscTargetIP, oscConfig.targetPort); msgSpeed.send(oscUdp); oscUdp.endPacket(); msgSpeed.empty();
        }
      }
    }
  }
}

void networkTask(void* parameter) {
  Serial.println("[Network Task] Started");
  
  // Initial setup
  if (networkConfig.enabled) {
    setupNetworkConnection();
    
    if (wifiSTAConnected) {
      if (midiConfig.enabled) setupMIDI();
      if (oscConfig.enabled) setupOSC();
    }
  }
  
  for (;;) {
    if (networkConfig.enabled && wifiSTAConnected) {
      // Process MIDI
      if (midiConfig.enabled) {
        MIDI.read();
        sendMidiCC();
      }
      
      // Process OSC
      if (oscConfig.enabled) {
        sendOscData();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// --- CORE LOGIC FUNCTIONS ---
int mapDistanceToLED(int rawDistance) {
  const int (*selectedCalibration)[2] = (currentLedDensity == 60) ? calibration60 : calibration30;
  rawDistance = constrain(rawDistance, MIN_PROCESS_DISTANCE, MAX_PROCESS_DISTANCE);
  for (int i = 0; i < CALIBRATION_POINTS - 1; i++) {
    if (rawDistance >= selectedCalibration[i][0] && rawDistance <= selectedCalibration[i+1][0]) {
      if (selectedCalibration[i+1][0] == selectedCalibration[i][0]) return selectedCalibration[i][1];
      float proportion = (float)(rawDistance - selectedCalibration[i][0]) / (float)(selectedCalibration[i+1][0] - selectedCalibration[i][0]);
      return constrain(selectedCalibration[i][1] + round(proportion * (selectedCalibration[i+1][1] - selectedCalibration[i][1])), 0, currentNumLeds - 1);
    }
  }
  if (rawDistance >= selectedCalibration[CALIBRATION_POINTS-1][0]) return constrain(selectedCalibration[CALIBRATION_POINTS-1][1], 0, currentNumLeds - 1);
  return constrain(selectedCalibration[0][1], 0, currentNumLeds - 1);
}

bool isTargetInInactiveZone(int targetX, int targetY) {
  for (int i = 0; i < MAX_INACTIVE_ZONES; ++i) {
    if (!inactiveZones[i].enabled) continue;
    const Point* corners = inactiveZones[i].corners;
    bool has_neg = false, has_pos = false;
    for (int j = 0; j < 4; ++j) {
      const Point& p1 = corners[j];
      const Point& p2 = corners[(j + 1) % 4];
      int cross_product = (p2.x - p1.x) * (targetY - p1.y) - (p2.y - p1.y) * (targetX - p1.x);
      if (cross_product < 0) has_neg = true;
      if (cross_product > 0) has_pos = true;
    }
    if (!(has_neg && has_pos)) return true;
  }
  return false;
}

void loadSettings() {
  Serial.println("\n--- loadSettings() begins ---");
  EEPROM.begin(EEPROM_SIZE);
  int addr = 0;
  byte checkVal;
  EEPROM.get(addr, checkVal);
  addr += sizeof(checkVal); 
  if (checkVal != EEPROM_VERSION) { 
    Serial.printf("!!! EEPROM version mismatch. Resetting to defaults.\n");
    EEPROM.end();
    saveSettings();
    return;
  }
  
  EEPROM.get(addr, ledOffDelay); addr += sizeof(ledOffDelay);
  EEPROM.get(addr, movingIntensity); addr += sizeof(movingIntensity);
  EEPROM.get(addr, stationaryIntensity); addr += sizeof(stationaryIntensity);
  EEPROM.get(addr, movingLength); addr += sizeof(movingLength);
  EEPROM.get(addr, centerShift); addr += sizeof(centerShift);
  EEPROM.get(addr, additionalLEDs); addr += sizeof(additionalLEDs);
  EEPROM.get(addr, targetSettings[0].color); addr += sizeof(targetSettings[0].color);
  EEPROM.get(addr, gradientSoftness); addr += sizeof(gradientSoftness);
  EEPROM.get(addr, startHour); addr += sizeof(startHour);
  EEPROM.get(addr, startMinute); addr += sizeof(startMinute);
  EEPROM.get(addr, endHour); addr += sizeof(endHour);
  EEPROM.get(addr, endMinute); addr += sizeof(endMinute);
  int tempTz; EEPROM.get(addr, tempTz); clientTimezoneOffsetMinutes = tempTz; addr += sizeof(clientTimezoneOffsetMinutes);
  EEPROM.get(addr, backgroundModeActive); addr += sizeof(backgroundModeActive);
  EEPROM.get(addr, currentLedDensity); addr += sizeof(currentLedDensity);
  currentNumLeds = constrain(currentLedDensity * STRIP_LENGTH, 1, MAX_NUM_LEDS);

  if (addr + sizeof(inactiveZones) <= EEPROM_SIZE) {
    EEPROM.get(addr, inactiveZones);
    addr += sizeof(inactiveZones);
  }
  
  // Load Network config
  if (addr + sizeof(NetworkConfig) <= EEPROM_SIZE) {
    EEPROM.get(addr, networkConfig);
    addr += sizeof(NetworkConfig);
  }
  
  // Load MIDI config
  if (addr + sizeof(MidiConfig) <= EEPROM_SIZE) {
    EEPROM.get(addr, midiConfig);
    addr += sizeof(MidiConfig);
  }
  
  // Load OSC config
  if (addr + sizeof(OscConfig) <= EEPROM_SIZE) {
    EEPROM.get(addr, oscConfig);
    addr += sizeof(OscConfig);
  }
  
  Serial.printf("[Config] Network: %d, MIDI: %d, OSC: %d\n", 
                networkConfig.enabled, midiConfig.enabled, oscConfig.enabled);
  
  EEPROM.end();
}

void saveSettings() {
  Serial.println("\n--- saveSettings() begins ---");
  EEPROM.begin(EEPROM_SIZE);
  int addr = 0;
  byte checkVal = EEPROM_VERSION;
  EEPROM.put(addr, checkVal); addr += sizeof(checkVal);
  
  EEPROM.put(addr, ledOffDelay); addr += sizeof(ledOffDelay);
  EEPROM.put(addr, movingIntensity); addr += sizeof(movingIntensity);
  EEPROM.put(addr, stationaryIntensity); addr += sizeof(stationaryIntensity);
  EEPROM.put(addr, movingLength); addr += sizeof(movingLength);
  EEPROM.put(addr, centerShift); addr += sizeof(centerShift);
  EEPROM.put(addr, additionalLEDs); addr += sizeof(additionalLEDs);
  EEPROM.put(addr, targetSettings[0].color); addr += sizeof(targetSettings[0].color);
  EEPROM.put(addr, gradientSoftness); addr += sizeof(gradientSoftness);
  EEPROM.put(addr, startHour); addr += sizeof(startHour);
  EEPROM.put(addr, startMinute); addr += sizeof(startMinute);
  EEPROM.put(addr, endHour); addr += sizeof(endHour);
  EEPROM.put(addr, endMinute); addr += sizeof(endMinute);
  int tempTz = clientTimezoneOffsetMinutes; EEPROM.put(addr, tempTz); addr += sizeof(clientTimezoneOffsetMinutes);
  EEPROM.put(addr, backgroundModeActive); addr += sizeof(backgroundModeActive);
  EEPROM.put(addr, currentLedDensity); addr += sizeof(currentLedDensity);

  if (addr + sizeof(inactiveZones) <= EEPROM_SIZE) {
    EEPROM.put(addr, inactiveZones);
    addr += sizeof(inactiveZones);
  }
  
  if (addr + sizeof(NetworkConfig) <= EEPROM_SIZE) {
    EEPROM.put(addr, networkConfig);
    addr += sizeof(NetworkConfig);
  }
  
  if (addr + sizeof(MidiConfig) <= EEPROM_SIZE) {
    EEPROM.put(addr, midiConfig);
    addr += sizeof(MidiConfig);
  }
  
  if (addr + sizeof(OscConfig) <= EEPROM_SIZE) {
    EEPROM.put(addr, oscConfig);
    addr += sizeof(OscConfig);
  }
  
  boolean result = EEPROM.commit();
  EEPROM.end();
  Serial.printf("EEPROM save: %s (%d bytes)\n", result ? "OK" : "ERROR", addr);
}

bool initializeStrip() {
  Serial.println("Initializing LED Strip...");
  try {
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, MAX_NUM_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(255);
    FastLED.clear(true);
    return true;
  } catch (...) {
    Serial.println("!!! CRITICAL ERROR in FastLED!");
    return false;
  }
}

// --- RTOS TASKS ---
void sensorTask(void* parameter) {
  Serial.println("Sensor Task started.");
  unsigned long lastSensorDebug = 0;
  
  // Give radar a moment to stabilize after task start
  vTaskDelay(pdMS_TO_TICKS(1000));
  
  for (;;) {
    int readResult = radar.read();
    
    // Debug output every 2 seconds
    unsigned long now = millis();
    if (now - lastSensorDebug > 2000) {
      lastSensorDebug = now;
      Serial.printf("[RADAR DEBUG] read()=%d, Serial1.available()=%d, supportedTargets=%d\n", 
                    readResult, Serial1.available(), radar.getSensorSupportedTargetCount());
      
      if (readResult > 0) {
        for (int i = 0; i < radar.getSensorSupportedTargetCount(); i++) {
          LD2450::RadarTarget rt = radar.getTarget(i);
          Serial.printf("  radar[%d] valid:%d x:%d y:%d dist:%d speed:%d\n",
                        i, rt.valid, rt.x, rt.y, rt.distance, rt.speed);
        }
        // Show targets array status
        Serial.printf("  targets[]: ");
        for (int i = 0; i < MAX_TARGETS; i++) {
          if (targets[i].present) {
            Serial.printf("T%d(x:%d,y:%d) ", i, targets[i].x, targets[i].y);
          }
        }
        bool anyPresent = false;
        for (int i = 0; i < MAX_TARGETS; i++) if (targets[i].present) anyPresent = true;
        if (!anyPresent) Serial.printf("(none)");
        Serial.println();
      } else {
        // Show raw bytes if we're getting nothing
        if (Serial1.available() > 0) {
          Serial.print("  Raw bytes available: ");
          int count = 0;
          while (Serial1.available() > 0 && count < 20) {
            Serial.printf("%02X ", Serial1.read());
            count++;
          }
          Serial.println();
        } else {
          Serial.println("  No data in Serial1 buffer - check wiring!");
        }
      }
    }
    
    if (readResult > 0) {
      unsigned long currentMillis = millis();
      
      // Get smoothing factor
      float alpha = max(midiConfig.smoothingFactor, oscConfig.smoothingFactor);
      if (alpha < 0.01f) alpha = 0.15f;
      
      // Collect all valid radar targets first
      struct ValidTarget {
        int x, y, distance, speed;
        bool valid;
      };
      ValidTarget validTargets[3] = {{0,0,0,0,false}, {0,0,0,0,false}, {0,0,0,0,false}};
      int validCount = 0;
      
      int radarTargetCount = min((int)radar.getSensorSupportedTargetCount(), 3);
      for (int i = 0; i < radarTargetCount; i++) {
        LD2450::RadarTarget rt = radar.getTarget(i);
        if (rt.valid && rt.distance > 0) {
          validTargets[validCount].x = rt.x;
          validTargets[validCount].y = rt.y;
          validTargets[validCount].distance = rt.distance;
          validTargets[validCount].speed = rt.speed;
          validTargets[validCount].valid = true;
          validCount++;
        }
      }
      
      // Sort by distance (closest first) using simple bubble sort
      for (int i = 0; i < validCount - 1; i++) {
        for (int j = 0; j < validCount - i - 1; j++) {
          if (validTargets[j].distance > validTargets[j+1].distance) {
            ValidTarget temp = validTargets[j];
            validTargets[j] = validTargets[j+1];
            validTargets[j+1] = temp;
          }
        }
      }
      
      // Track which target slots we're updating this cycle
      bool targetFoundThisCycle[MAX_TARGETS] = {false, false, false};
      
      // Assign sorted targets to our targets array (closest = index 0)
      for (int i = 0; i < MAX_TARGETS; i++) {
        if (i < validCount && validTargets[i].valid) {
          // Check for ghost target (stationary too long)
          if (targets[i].present && (currentMillis - targets[i].lastMovementTime > GHOST_REMOVAL_TIMEOUT)) {
            // Ghost target - skip but don't mark as found
            continue;
          }
          
          targetFoundThisCycle[i] = true;
          if (!targets[i].present) { 
            targets[i].lastMovementTime = currentMillis; 
          }
          
          // Store raw values
          targets[i].x = validTargets[i].x;
          targets[i].y = validTargets[i].y;
          targets[i].distance = validTargets[i].distance;
          targets[i].speed = validTargets[i].speed;
          targets[i].present = true;
          targets[i].lastSeenTime = currentMillis;
          
          // Calculate smoothed values using exponential moving average
          if (!targets[i].smoothedInitialized) {
            targets[i].smoothedX = (float)validTargets[i].x;
            targets[i].smoothedY = (float)validTargets[i].y;
            targets[i].smoothedDistance = (float)validTargets[i].distance;
            targets[i].smoothedSpeed = (float)validTargets[i].speed;
            targets[i].smoothedInitialized = true;
          } else {
            targets[i].smoothedX = alpha * (float)validTargets[i].x + (1.0f - alpha) * targets[i].smoothedX;
            targets[i].smoothedY = alpha * (float)validTargets[i].y + (1.0f - alpha) * targets[i].smoothedY;
            targets[i].smoothedDistance = alpha * (float)validTargets[i].distance + (1.0f - alpha) * targets[i].smoothedDistance;
            targets[i].smoothedSpeed = alpha * (float)validTargets[i].speed + (1.0f - alpha) * targets[i].smoothedSpeed;
          }
          
          if (abs(validTargets[i].speed) >= movementSensitivity) {
            targets[i].lastMovementTime = currentMillis;
            if (validTargets[i].speed > movementSensitivity) targets[i].lastMovementDirection = 1;
            else if (validTargets[i].speed < -movementSensitivity) targets[i].lastMovementDirection = -1;
          }
        }
      }
      
      // Mark targets not found this cycle as not present
      for (int i = 0; i < MAX_TARGETS; i++) {
        if (!targetFoundThisCycle[i]) {
          targets[i].present = false;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void ledTask(void* parameter) {
  unsigned long lastCycleTimeMicros = micros();
  while (true) {
    unsigned long currentMicros = micros();
    float deltaTime = (currentMicros - lastCycleTimeMicros) / 1000000.0f;
    lastCycleTimeMicros = currentMicros;
    if (deltaTime <= 0.0f || deltaTime > 0.1f) deltaTime = (float)ledUpdateInterval / 1000.0f;

    if (!lightOn && !smarthomeOverride) {
      fill_solid(leds, MAX_NUM_LEDS, CRGB::Black);
      FastLED.show();
      vTaskDelay(pdMS_TO_TICKS(ledUpdateInterval * 2));
      continue;
    }

    if (backgroundModeActive) {
      CRGB bgColor = targetSettings[0].color;
      bgColor.nscale8_video(stationaryIntensity * 255);
      fill_solid(leds, currentNumLeds, bgColor);
    } else {
      fill_solid(leds, currentNumLeds, CRGB::Black);
    }
    if (currentNumLeds < MAX_NUM_LEDS) {
      fill_solid(leds + currentNumLeds, MAX_NUM_LEDS - currentNumLeds, CRGB::Black);
    }

    Target& target = targets[0];
    const float q_pos = 1.0f * deltaTime, q_vel = 5.0f * deltaTime, r_measure = 10.0f;

    if (target.present) {
      float measuredPosition = (float)mapDistanceToLED(target.distance);
      if (!target.isInitialized) {
        target.smoothedLEDPosition = measuredPosition;
        target.velocityLED = 0;
        target.p_pos = 1000.0f;
        target.p_vel = 1000.0f;
        target.isInitialized = true;
      }
      float predicted_pos = target.smoothedLEDPosition + target.velocityLED * deltaTime;
      target.p_pos += q_pos;
      target.p_vel += q_vel;
      float innovation = measuredPosition - predicted_pos;
      float s_inv = 1.0f / (target.p_pos + r_measure);
      float k_pos = target.p_pos * s_inv;
      float k_vel = target.p_vel * s_inv;
      target.smoothedLEDPosition = predicted_pos + k_pos * innovation;
      target.velocityLED += k_vel * innovation;
      target.p_pos *= (1.0f - k_pos);
      target.p_vel *= (1.0f - k_pos);
    } else if (target.isInitialized) {
      float decayRate = 15.0f;
      target.velocityLED *= (1.0f - constrain(decayRate * deltaTime, 0.0f, 1.0f));
      target.p_pos = min(target.p_pos * 1.5f, 1000.0f);
      target.p_vel = min(target.p_vel * 1.5f, 1000.0f);
      if (millis() - target.lastSeenTime > ((unsigned long)ledOffDelay * 1000 + 500)) {
        target.isInitialized = false;
      }
    }

    float maxVel = 500.0f * ((float)currentNumLeds / MAX_NUM_LEDS);
    target.velocityLED = constrain(target.velocityLED, -maxVel, maxVel);
    target.smoothedLEDPosition = constrain(target.smoothedLEDPosition, -1.0f, (float)currentNumLeds);

    float targetBrightness = 0.0f;
    bool shouldBeLit = (target.isInitialized && (target.present || (ledOffDelay > 0 && millis() - target.lastSeenTime < (unsigned long)ledOffDelay * 1000)));
    if (shouldBeLit && !isTargetInInactiveZone(target.x, target.y)) {
      bool isMoving = target.present && (abs(target.speed) >= movementSensitivity);
      if (isMoving) targetBrightness = movingIntensity;
      else if (!backgroundModeActive) targetBrightness = movingIntensity;
      else if (target.present && (millis() - target.lastMovementTime < GHOST_FILTER_TIMEOUT)) targetBrightness = stationaryIntensity;
      else if (!target.present) targetBrightness = movingIntensity;
    }
    if (ledOffDelay == 0 && !target.present) target.currentBrightness = 0.0f;

    float fadeRate = (target.currentBrightness < targetBrightness) ? 8.0f : 4.0f;
    target.currentBrightness += (targetBrightness - target.currentBrightness) * constrain(fadeRate * deltaTime, 0.0f, 1.0f);
    if (target.currentBrightness < 0.005f) target.currentBrightness = 0.0f;

    if (target.currentBrightness > 0.005f && target.smoothedLEDPosition >= 0) {
      int centerLED = round(target.smoothedLEDPosition) + centerShift;
      int direction = target.lastMovementDirection;
      if (abs(target.velocityLED) > 2.0f) direction = (target.velocityLED > 0) ? 1 : -1;
      if (direction == 0) direction = 1;

      int leftEdge, rightEdge;
      if (direction > 0) {
        leftEdge = centerLED - (movingLength / 2);
        rightEdge = leftEdge + movingLength - 1 + additionalLEDs;
      } else {
        rightEdge = centerLED + (movingLength / 2);
        leftEdge = rightEdge - (movingLength - 1) - additionalLEDs;
      }

      int totalLightLength = rightEdge - leftEdge + 1;
      if (totalLightLength <= 0) totalLightLength = 1;

      int fadeWidth = map(gradientSoftness, 0, 10, 0, max(1, totalLightLength / 3));
      float fadeExp = 1.0 + (gradientSoftness / 10.0) * 1.5;

      for (int i = max(0, leftEdge); i <= min(currentNumLeds - 1, rightEdge); ++i) {
        int pos = (direction > 0) ? (i - leftEdge) : (rightEdge - i);
        float intensity = 1.0f;
        if (gradientSoftness > 0 && fadeWidth > 0 && totalLightLength > 1) {
          if (pos < fadeWidth) intensity = powf((float)pos / fadeWidth, fadeExp);
          else if (pos >= (totalLightLength - fadeWidth)) intensity = powf((float)(totalLightLength - 1 - pos) / fadeWidth, fadeExp);
        }
        if (intensity > 0.01f) {
          CRGB drawColor = targetSettings[0].color;
          drawColor.nscale8_video(scale8(target.currentBrightness * 255, (uint8_t)(intensity * 255)));
          leds[i] += drawColor;
        }
      }
    }

    FastLED.show();
    long delayMicros = (ledUpdateInterval * 1000) - (micros() - currentMicros);
    vTaskDelay(pdMS_TO_TICKS(max(0L, delayMicros / 1000)));
  }
}

// --- WEB & NETWORK FUNCTIONS ---
void setupWiFi() {
  WiFi.mode(WIFI_AP_STA);
  esp_wifi_set_max_tx_power(8);
  IPAddress local_IP(192, 168, 4, 1);
  WiFi.softAPConfig(local_IP, local_IP, IPAddress(255, 255, 255, 0));

  uint8_t mac[6] = {0};
  esp_wifi_get_mac(WIFI_IF_AP, mac);
  char deviceName[25];
  sprintf(deviceName, "LightTrack-%02X%02X%02X", mac[3], mac[4], mac[5]);
  WiFi.softAP(deviceName, "12345678");

  server.on("/", HTTP_GET, handleRoot);
  server.on("/network", HTTP_GET, handleNetworkSettings);
  server.on("/setNetworkConfig", HTTP_GET, handleSetNetworkConfig);
  server.on("/getNetworkStatus", HTTP_GET, handleGetNetworkStatus);
  server.on("/setMovingIntensity", HTTP_GET, handleSetMovingIntensity);
  server.on("/setStationaryIntensity", HTTP_GET, handleSetStationaryIntensity);
  server.on("/setMovingLength", HTTP_GET, handleSetMovingLength);
  server.on("/setAdditionalLEDs", HTTP_GET, handleSetAdditionalLEDs);
  server.on("/setCenterShift", HTTP_GET, handleSetCenterShift);
  server.on("/setLedDensity", HTTP_GET, handleSetLedDensity);
  server.on("/setGradientSoftness", HTTP_GET, handleSetGradientSoftness);
  server.on("/setLedOffDelay", HTTP_GET, handleSetLedOffDelay);
  server.on("/setTime", HTTP_GET, handleSetTime);
  server.on("/setSchedule", HTTP_GET, handleSetSchedule);
  server.on("/smarthome/on", HTTP_GET, handleSmartHomeOn);
  server.on("/smarthome/off", HTTP_GET, handleSmartHomeOff);
  server.on("/smarthome/clear", HTTP_GET, handleSmartHomeClear);
  server.on("/toggleBackgroundMode", HTTP_GET, handleToggleBackgroundMode);
  server.on("/getCurrentTime", HTTP_GET, handleGetCurrentTime);
  server.on("/radarview", HTTP_GET, handleRadarView);
  server.on("/setTarget", HTTP_GET, handleSetTargetSettings);
  server.onNotFound(handleNotFound);
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void checkWiFiTimeout() {
  if (networkConfig.enabled) return;
  
  if (wifiActive && wifiTimeoutMinutes > 0 && millis() - wifiStartTime > (unsigned long)wifiTimeoutMinutes * 60 * 1000) {
    WiFi.mode(WIFI_OFF);
    wifiActive = false;
  }
}

void webServerTask(void* p) {
  unsigned long lastRadarBroadcast = 0;
  for (;;) {
    if (wifiActive) {
      server.handleClient();
      webSocket.loop();
      ArduinoOTA.handle();
      if (webSocket.connectedClients() > 0 && millis() - lastRadarBroadcast > 100) {
        broadcastRadarData();
        lastRadarBroadcast = millis();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void setupOTA() {
  ArduinoOTA.onStart([]() {});
  ArduinoOTA.onEnd([]() {});
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {});
  ArduinoOTA.onError([](ota_error_t e) {});

  uint8_t m[6] = {0};
  esp_wifi_get_mac(WIFI_IF_AP, m);
  char d[25];
  sprintf(d, "LightTrack-%02X%02X%02X", m[3], m[4], m[5]);
  ArduinoOTA.setHostname(d);
  ArduinoOTA.begin();
}

// --- NETWORK/MIDI/OSC WEB HANDLERS ---
void handleNetworkSettings() {
  String html = R"rawliteral(
<!DOCTYPE html><html><head><title>Network Settings - LightTrack</title>
<meta name='viewport' content='width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no'>
<meta charset='UTF-8'>
<style>
body{background-color:#282c34;color:#abb2bf;font-family:sans-serif;margin:0;padding:15px}
.container{max-width:700px;margin:auto;background-color:#3a3f4b;padding:20px;border-radius:8px}
h1,h2{color:#61afef;text-align:center;border-bottom:1px solid #4b5263;padding-bottom:10px;margin-top:0}
h2{margin-top:20px;font-size:1.1em;border-bottom:none;color:#c678dd}
h3{color:#98c379;margin:15px 0 10px 0;font-size:1em}
label{display:block;margin-top:10px;margin-bottom:4px;color:#98c379;font-weight:bold;font-size:0.85em}
input[type=text],input[type=password],input[type=number]{width:100%;padding:8px;border-radius:4px;border:1px solid #4b5263;background-color:#282c34;color:#abb2bf;box-sizing:border-box;font-size:1em}
input[type=number]{width:80px}
.row{display:flex;gap:10px;flex-wrap:wrap;align-items:flex-end}
.row>div{flex:1;min-width:100px}
button{display:block;width:100%;font-size:1em;margin-top:20px;padding:12px;border:none;border-radius:5px;background-color:#98c379;color:#282c34;cursor:pointer;font-weight:bold}
button:hover{background-color:#7cb668}
.toggle{display:flex;align-items:center;gap:10px;margin:10px 0;padding:10px;background:#2c313a;border-radius:5px}
.toggle input{width:20px;height:20px;cursor:pointer}
.toggle label{margin:0;display:inline;cursor:pointer}
.status{margin-top:15px;padding:12px;border-radius:5px;background-color:#2c313a;font-size:0.9em}
.status.connected{border-left:4px solid #98c379}
.status.disconnected{border-left:4px solid #e06c75}
a.back{display:inline-block;margin-top:15px;color:#61afef;text-decoration:none}
a.back:hover{text-decoration:underline}
.hint{font-size:0.8em;color:#5c6370;margin-top:2px}
.section{background-color:#2c313a;padding:15px;border-radius:5px;margin-top:15px}
.tabs{display:flex;gap:5px;margin-bottom:15px}
.tab{flex:1;padding:10px;text-align:center;background:#4b5263;border-radius:5px 5px 0 0;cursor:pointer;font-weight:bold}
.tab.active{background:#61afef;color:#282c34}
.tab-content{display:none}
.tab-content.active{display:block}
.protocol-header{display:flex;justify-content:space-between;align-items:center}
</style>
<script>
function showTab(id){
  document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active'));
  document.querySelectorAll('.tab-content').forEach(t=>t.classList.remove('active'));
  document.querySelector(`[data-tab="${id}"]`).classList.add('active');
  document.getElementById(id).classList.add('active');
}
function save(){
  const params=new URLSearchParams();
  // Network
  params.append('netEnabled',document.getElementById('netEnabled').checked?1:0);
  params.append('ssid',document.getElementById('ssid').value);
  params.append('password',document.getElementById('password').value);
  // MIDI
  params.append('midiEnabled',document.getElementById('midiEnabled').checked?1:0);
  params.append('midiSession',document.getElementById('midiSession').value);
  params.append('midiChannel',document.getElementById('midiChannel').value);
  params.append('midiInterval',document.getElementById('midiInterval').value);
  params.append('midiSmoothed',document.getElementById('midiSmoothed').checked?1:0);
  params.append('midiSmoothFactor',document.getElementById('midiSmoothFactor').value);
  params.append('ccX',document.getElementById('ccX').value);
  params.append('ccY',document.getElementById('ccY').value);
  params.append('ccDist',document.getElementById('ccDist').value);
  params.append('ccSpeed',document.getElementById('ccSpeed').value);
  params.append('midiXMin',document.getElementById('midiXMin').value);
  params.append('midiXMax',document.getElementById('midiXMax').value);
  params.append('midiYMin',document.getElementById('midiYMin').value);
  params.append('midiYMax',document.getElementById('midiYMax').value);
  params.append('midiDistMin',document.getElementById('midiDistMin').value);
  params.append('midiDistMax',document.getElementById('midiDistMax').value);
  params.append('midiSpeedMin',document.getElementById('midiSpeedMin').value);
  params.append('midiSpeedMax',document.getElementById('midiSpeedMax').value);
  // OSC
  params.append('oscEnabled',document.getElementById('oscEnabled').checked?1:0);
  params.append('oscHost',document.getElementById('oscHost').value);
  params.append('oscPort',document.getElementById('oscPort').value);
  params.append('oscLocalPort',document.getElementById('oscLocalPort').value);
  params.append('oscPrefix',document.getElementById('oscPrefix').value);
  params.append('oscBundle',document.getElementById('oscBundle').checked?1:0);
  params.append('oscNormalize',document.getElementById('oscNormalize').checked?1:0);
  params.append('oscSmoothed',document.getElementById('oscSmoothed').checked?1:0);
  params.append('oscSmoothFactor',document.getElementById('oscSmoothFactor').value);
  params.append('oscInterval',document.getElementById('oscInterval').value);
  params.append('oscXMin',document.getElementById('oscXMin').value);
  params.append('oscXMax',document.getElementById('oscXMax').value);
  params.append('oscYMin',document.getElementById('oscYMin').value);
  params.append('oscYMax',document.getElementById('oscYMax').value);
  params.append('oscDistMin',document.getElementById('oscDistMin').value);
  params.append('oscDistMax',document.getElementById('oscDistMax').value);
  params.append('oscSpeedMin',document.getElementById('oscSpeedMin').value);
  params.append('oscSpeedMax',document.getElementById('oscSpeedMax').value);
  
  fetch('/setNetworkConfig?'+params.toString())
    .then(r=>r.text())
    .then(d=>{alert('Settings saved! Device will restart...');setTimeout(()=>location.reload(),3000);})
    .catch(e=>alert('Error saving settings'));
}
function updateStatus(){
  fetch('/getNetworkStatus').then(r=>r.json()).then(d=>{
    const el=document.getElementById('status');
    let html='<b>WiFi:</b> '+(d.wifiConnected?'✅ Connected ('+d.ip+')':'❌ Disconnected');
    html+='<br><b>rtpMIDI:</b> '+(d.midiEnabled?(d.midiConnected?'✅ Session Active':'⏳ Waiting...'):'⚪ Disabled');
    html+='<br><b>OSC:</b> '+(d.oscEnabled?(d.oscResolved?'✅ Sending to '+d.oscTarget:'❌ Target not resolved'):'⚪ Disabled');
    html+='<br><b>Targets:</b> '+d.targetCount+'/3'+(d.smoothed?' (smoothed)':'');
    if(d.targets){
      d.targets.forEach(t=>{
        if(t.present){
          html+='<br>&nbsp;&nbsp;🎯 T'+t.id+': X:'+t.x+' Y:'+t.y+' D:'+t.dist+' S:'+t.speed;
        }
      });
    }
    if(d.targetCount===0) html+='<br>&nbsp;&nbsp;⚪ No targets detected';
    el.innerHTML=html;
    el.className='status '+(d.wifiConnected?'connected':'disconnected');
  });
}
setInterval(updateStatus,1000);
window.onload=()=>{updateStatus();showTab('tab-wifi');};
</script>
</head><body><div class='container'>
<h1>🌐 Network & Protocols</h1>
<div id='status' class='status disconnected'>Loading...</div>

<div class='tabs'>
  <div class='tab active' data-tab='tab-wifi' onclick="showTab('tab-wifi')">📶 WiFi</div>
  <div class='tab' data-tab='tab-midi' onclick="showTab('tab-midi')">🎹 MIDI</div>
  <div class='tab' data-tab='tab-osc' onclick="showTab('tab-osc')">📡 OSC</div>
</div>

<!-- WiFi Tab -->
<div id='tab-wifi' class='tab-content active'>
<div class='section'>
<div class='toggle'>
  <input type='checkbox' id='netEnabled' )rawliteral";
  html += networkConfig.enabled ? "checked" : "";
  html += R"rawliteral(>
  <label for='netEnabled'>Enable WiFi Connection</label>
</div>
<label>Network SSID</label>
<input type='text' id='ssid' value=')rawliteral";
  html += String(networkConfig.wifiSSID);
  html += R"rawliteral(' maxlength='32' placeholder='Your WiFi network name'>
<label>Password</label>
<input type='password' id='password' value=')rawliteral";
  html += String(networkConfig.wifiPassword);
  html += R"rawliteral(' maxlength='64' placeholder='WiFi password'>
<p class='hint'>Connect to your studio/home WiFi. The config AP (192.168.4.1) stays active.</p>
</div>
</div>

<!-- MIDI Tab -->
<div id='tab-midi' class='tab-content'>
<div class='section'>
<div class='protocol-header'>
<h3>🎹 rtpMIDI (AppleMIDI)</h3>
<div class='toggle' style='margin:0;padding:5px 10px'>
  <input type='checkbox' id='midiEnabled' )rawliteral";
  html += midiConfig.enabled ? "checked" : "";
  html += R"rawliteral(>
  <label for='midiEnabled'>Enable</label>
</div>
</div>
<p class='hint'>Works with macOS Audio MIDI Setup, iOS, and Windows rtpMIDI driver.<br>
3 targets send on consecutive channels (base, base+1, base+2).</p>

<div class='row'>
<div><label>Session Name</label>
<input type='text' id='midiSession' value=')rawliteral";
  html += String(midiConfig.sessionName);
  html += R"rawliteral(' maxlength='32'></div>
<div><label>Channel (1-16)</label>
<input type='number' id='midiChannel' min='1' max='16' value=')rawliteral";
  html += String(midiConfig.midiChannel);
  html += R"rawliteral('></div>
<div><label>Interval (ms)</label>
<input type='number' id='midiInterval' min='10' max='500' value=')rawliteral";
  html += String(midiConfig.sendInterval);
  html += R"rawliteral('></div>
</div>

<div class='row' style='margin-top:10px'>
<div class='toggle' style='flex:1'>
  <input type='checkbox' id='midiSmoothed' )rawliteral";
  html += midiConfig.sendSmoothed ? "checked" : "";
  html += R"rawliteral(>
  <label for='midiSmoothed'>Send Smoothed Values</label>
</div>
<div><label>Smoothing (0.05-0.5)</label>
<input type='number' id='midiSmoothFactor' min='0.05' max='0.5' step='0.05' value=')rawliteral";
  html += String(midiConfig.smoothingFactor, 2);
  html += R"rawliteral('></div>
</div>
<p class='hint'>Smoothed values use exponential moving average. Lower = smoother but more latency.</p>

<h3>CC Assignments</h3>
<div class='row'>
<div><label>X → CC#</label><input type='number' id='ccX' min='0' max='127' value=')rawliteral";
  html += String(midiConfig.ccX);
  html += R"rawliteral('></div>
<div><label>Y → CC#</label><input type='number' id='ccY' min='0' max='127' value=')rawliteral";
  html += String(midiConfig.ccY);
  html += R"rawliteral('></div>
<div><label>Dist → CC#</label><input type='number' id='ccDist' min='0' max='127' value=')rawliteral";
  html += String(midiConfig.ccDistance);
  html += R"rawliteral('></div>
<div><label>Speed → CC#</label><input type='number' id='ccSpeed' min='0' max='127' value=')rawliteral";
  html += String(midiConfig.ccSpeed);
  html += R"rawliteral('></div>
</div>

<h3>Value Ranges (mm → 0-127)</h3>
<div class='row'>
<div><label>X Min</label><input type='number' id='midiXMin' value=')rawliteral";
  html += String(midiConfig.xMin);
  html += R"rawliteral('></div>
<div><label>X Max</label><input type='number' id='midiXMax' value=')rawliteral";
  html += String(midiConfig.xMax);
  html += R"rawliteral('></div>
<div><label>Y Min</label><input type='number' id='midiYMin' value=')rawliteral";
  html += String(midiConfig.yMin);
  html += R"rawliteral('></div>
<div><label>Y Max</label><input type='number' id='midiYMax' value=')rawliteral";
  html += String(midiConfig.yMax);
  html += R"rawliteral('></div>
</div>
<div class='row'>
<div><label>Dist Min</label><input type='number' id='midiDistMin' value=')rawliteral";
  html += String(midiConfig.distMin);
  html += R"rawliteral('></div>
<div><label>Dist Max</label><input type='number' id='midiDistMax' value=')rawliteral";
  html += String(midiConfig.distMax);
  html += R"rawliteral('></div>
<div><label>Speed Min</label><input type='number' id='midiSpeedMin' value=')rawliteral";
  html += String(midiConfig.speedMin);
  html += R"rawliteral('></div>
<div><label>Speed Max</label><input type='number' id='midiSpeedMax' value=')rawliteral";
  html += String(midiConfig.speedMax);
  html += R"rawliteral('></div>
</div>
</div>
</div>

<!-- OSC Tab -->
<div id='tab-osc' class='tab-content'>
<div class='section'>
<div class='protocol-header'>
<h3>📡 OSC (Open Sound Control)</h3>
<div class='toggle' style='margin:0;padding:5px 10px'>
  <input type='checkbox' id='oscEnabled' )rawliteral";
  html += oscConfig.enabled ? "checked" : "";
  html += R"rawliteral(>
  <label for='oscEnabled'>Enable</label>
</div>
</div>
<p class='hint'>Works with TouchDesigner, Max/MSP, SuperCollider, Ableton (via plugins), etc.</p>

<div class='row'>
<div style='flex:2'><label>Target Host (IP or hostname)</label>
<input type='text' id='oscHost' value=')rawliteral";
  html += String(oscConfig.targetHost);
  html += R"rawliteral(' maxlength='63' placeholder='192.168.1.100'></div>
<div><label>Port</label>
<input type='number' id='oscPort' min='1' max='65535' value=')rawliteral";
  html += String(oscConfig.targetPort);
  html += R"rawliteral('></div>
</div>

<div class='row'>
<div><label>Address Prefix</label>
<input type='text' id='oscPrefix' value=')rawliteral";
  html += String(oscConfig.addressPrefix);
  html += R"rawliteral(' maxlength='31' placeholder='/radar'></div>
<div><label>Local Port (recv)</label>
<input type='number' id='oscLocalPort' min='0' max='65535' value=')rawliteral";
  html += String(oscConfig.localPort);
  html += R"rawliteral('></div>
<div><label>Interval (ms)</label>
<input type='number' id='oscInterval' min='5' max='500' value=')rawliteral";
  html += String(oscConfig.sendInterval);
  html += R"rawliteral('></div>
</div>

<div class='row' style='margin-top:10px'>
<div class='toggle' style='flex:1'>
  <input type='checkbox' id='oscBundle' )rawliteral";
  html += oscConfig.sendBundle ? "checked" : "";
  html += R"rawliteral(>
  <label for='oscBundle'>Send as Bundle</label>
</div>
<div class='toggle' style='flex:1'>
  <input type='checkbox' id='oscNormalize' )rawliteral";
  html += oscConfig.normalizeValues ? "checked" : "";
  html += R"rawliteral(>
  <label for='oscNormalize'>Normalize (0.0-1.0)</label>
</div>
</div>

<div class='row' style='margin-top:10px'>
<div class='toggle' style='flex:1'>
  <input type='checkbox' id='oscSmoothed' )rawliteral";
  html += oscConfig.sendSmoothed ? "checked" : "";
  html += R"rawliteral(>
  <label for='oscSmoothed'>Send Smoothed Values</label>
</div>
<div><label>Smoothing (0.05-0.5)</label>
<input type='number' id='oscSmoothFactor' min='0.05' max='0.5' step='0.05' value=')rawliteral";
  html += String(oscConfig.smoothingFactor, 2);
  html += R"rawliteral('></div>
</div>
<p class='hint'>Smoothed = exponential moving average like LED position. Lower value = smoother but more latency.</p>

<h3>Value Ranges (for normalization)</h3>
<div class='row'>
<div><label>X Min</label><input type='number' id='oscXMin' value=')rawliteral";
  html += String(oscConfig.xMin);
  html += R"rawliteral('></div>
<div><label>X Max</label><input type='number' id='oscXMax' value=')rawliteral";
  html += String(oscConfig.xMax);
  html += R"rawliteral('></div>
<div><label>Y Min</label><input type='number' id='oscYMin' value=')rawliteral";
  html += String(oscConfig.yMin);
  html += R"rawliteral('></div>
<div><label>Y Max</label><input type='number' id='oscYMax' value=')rawliteral";
  html += String(oscConfig.yMax);
  html += R"rawliteral('></div>
</div>
<div class='row'>
<div><label>Dist Min</label><input type='number' id='oscDistMin' value=')rawliteral";
  html += String(oscConfig.distMin);
  html += R"rawliteral('></div>
<div><label>Dist Max</label><input type='number' id='oscDistMax' value=')rawliteral";
  html += String(oscConfig.distMax);
  html += R"rawliteral('></div>
<div><label>Speed Min</label><input type='number' id='oscSpeedMin' value=')rawliteral";
  html += String(oscConfig.speedMin);
  html += R"rawliteral('></div>
<div><label>Speed Max</label><input type='number' id='oscSpeedMax' value=')rawliteral";
  html += String(oscConfig.speedMax);
  html += R"rawliteral('></div>
</div>

<div class='section' style='margin-top:15px;background:#1e2127'>
<h3>📋 OSC Addresses Sent (3 targets)</h3>
<code style='color:#98c379;font-size:0.85em;line-height:1.6'>
)rawliteral";
  html += String(oscConfig.addressPrefix);
  html += R"rawliteral(/count (int: 0-3)<br>
)rawliteral";
  html += String(oscConfig.addressPrefix);
  html += R"rawliteral(/1/present, /2/present, /3/present<br>
)rawliteral";
  html += String(oscConfig.addressPrefix);
  html += R"rawliteral(/1/x, /1/y, /1/distance, /1/speed<br>
)rawliteral";
  html += String(oscConfig.addressPrefix);
  html += R"rawliteral(/2/x, /2/y, /2/distance, /2/speed<br>
)rawliteral";
  html += String(oscConfig.addressPrefix);
  html += R"rawliteral(/3/x, /3/y, /3/distance, /3/speed<br>
)rawliteral";
  html += String(oscConfig.addressPrefix);
  html += R"rawliteral(/1/all, /2/all, /3/all (x,y,dist,speed)
</code>
<p class='hint' style='margin-top:8px'>MIDI: Each target sends on consecutive channels (Ch1, Ch2, Ch3)</p>
</div>
</div>
</div>

<button onclick='save()'>💾 Save & Restart</button>
<a href='/' class='back'>← Back to Main Settings</a>
</div></body></html>
)rawliteral";
  
  server.send(200, "text/html; charset=utf-8", html);
}

void handleSetNetworkConfig() {
  // Network
  if (server.hasArg("netEnabled")) networkConfig.enabled = server.arg("netEnabled").toInt();
  if (server.hasArg("ssid")) strncpy(networkConfig.wifiSSID, server.arg("ssid").c_str(), 32);
  if (server.hasArg("password")) strncpy(networkConfig.wifiPassword, server.arg("password").c_str(), 64);
  
  // MIDI
  if (server.hasArg("midiEnabled")) midiConfig.enabled = server.arg("midiEnabled").toInt();
  if (server.hasArg("midiSession")) strncpy(midiConfig.sessionName, server.arg("midiSession").c_str(), 32);
  if (server.hasArg("midiChannel")) midiConfig.midiChannel = constrain(server.arg("midiChannel").toInt(), 1, 16);
  if (server.hasArg("midiInterval")) midiConfig.sendInterval = constrain(server.arg("midiInterval").toInt(), 10, 500);
  if (server.hasArg("midiSmoothed")) midiConfig.sendSmoothed = server.arg("midiSmoothed").toInt();
  if (server.hasArg("midiSmoothFactor")) midiConfig.smoothingFactor = constrain(server.arg("midiSmoothFactor").toFloat(), 0.05f, 0.5f);
  if (server.hasArg("ccX")) midiConfig.ccX = constrain(server.arg("ccX").toInt(), 0, 127);
  if (server.hasArg("ccY")) midiConfig.ccY = constrain(server.arg("ccY").toInt(), 0, 127);
  if (server.hasArg("ccDist")) midiConfig.ccDistance = constrain(server.arg("ccDist").toInt(), 0, 127);
  if (server.hasArg("ccSpeed")) midiConfig.ccSpeed = constrain(server.arg("ccSpeed").toInt(), 0, 127);
  if (server.hasArg("midiXMin")) midiConfig.xMin = server.arg("midiXMin").toInt();
  if (server.hasArg("midiXMax")) midiConfig.xMax = server.arg("midiXMax").toInt();
  if (server.hasArg("midiYMin")) midiConfig.yMin = server.arg("midiYMin").toInt();
  if (server.hasArg("midiYMax")) midiConfig.yMax = server.arg("midiYMax").toInt();
  if (server.hasArg("midiDistMin")) midiConfig.distMin = server.arg("midiDistMin").toInt();
  if (server.hasArg("midiDistMax")) midiConfig.distMax = server.arg("midiDistMax").toInt();
  if (server.hasArg("midiSpeedMin")) midiConfig.speedMin = server.arg("midiSpeedMin").toInt();
  if (server.hasArg("midiSpeedMax")) midiConfig.speedMax = server.arg("midiSpeedMax").toInt();
  
  // OSC
  if (server.hasArg("oscEnabled")) oscConfig.enabled = server.arg("oscEnabled").toInt();
  if (server.hasArg("oscHost")) strncpy(oscConfig.targetHost, server.arg("oscHost").c_str(), 63);
  if (server.hasArg("oscPort")) oscConfig.targetPort = constrain(server.arg("oscPort").toInt(), 1, 65535);
  if (server.hasArg("oscLocalPort")) oscConfig.localPort = constrain(server.arg("oscLocalPort").toInt(), 0, 65535);
  if (server.hasArg("oscPrefix")) strncpy(oscConfig.addressPrefix, server.arg("oscPrefix").c_str(), 31);
  if (server.hasArg("oscBundle")) oscConfig.sendBundle = server.arg("oscBundle").toInt();
  if (server.hasArg("oscNormalize")) oscConfig.normalizeValues = server.arg("oscNormalize").toInt();
  if (server.hasArg("oscSmoothed")) oscConfig.sendSmoothed = server.arg("oscSmoothed").toInt();
  if (server.hasArg("oscSmoothFactor")) oscConfig.smoothingFactor = constrain(server.arg("oscSmoothFactor").toFloat(), 0.05f, 0.5f);
  if (server.hasArg("oscInterval")) oscConfig.sendInterval = constrain(server.arg("oscInterval").toInt(), 5, 500);
  if (server.hasArg("oscXMin")) oscConfig.xMin = server.arg("oscXMin").toInt();
  if (server.hasArg("oscXMax")) oscConfig.xMax = server.arg("oscXMax").toInt();
  if (server.hasArg("oscYMin")) oscConfig.yMin = server.arg("oscYMin").toInt();
  if (server.hasArg("oscYMax")) oscConfig.yMax = server.arg("oscYMax").toInt();
  if (server.hasArg("oscDistMin")) oscConfig.distMin = server.arg("oscDistMin").toInt();
  if (server.hasArg("oscDistMax")) oscConfig.distMax = server.arg("oscDistMax").toInt();
  if (server.hasArg("oscSpeedMin")) oscConfig.speedMin = server.arg("oscSpeedMin").toInt();
  if (server.hasArg("oscSpeedMax")) oscConfig.speedMax = server.arg("oscSpeedMax").toInt();
  
  saveSettings();
  server.send(200, "text/plain", "OK");
  
  delay(500);
  ESP.restart();
}

void handleGetNetworkStatus() {
  JsonDocument doc;
  doc["wifiConnected"] = wifiSTAConnected;
  doc["ip"] = wifiSTAConnected ? WiFi.localIP().toString() : "";
  doc["midiEnabled"] = midiConfig.enabled;
  doc["midiConnected"] = midiConnected;
  doc["oscEnabled"] = oscConfig.enabled;
  doc["oscResolved"] = oscTargetResolved;
  doc["oscTarget"] = oscTargetResolved ? (oscTargetIP.toString() + ":" + String(oscConfig.targetPort)) : "";
  doc["smoothed"] = oscConfig.sendSmoothed || midiConfig.sendSmoothed;
  
  // Count present targets
  int targetCount = 0;
  for (int t = 0; t < MAX_TARGETS; t++) {
    if (targets[t].present) targetCount++;
  }
  doc["targetCount"] = targetCount;
  
  // Add array of targets
  JsonArray targetsArr = doc["targets"].to<JsonArray>();
  for (int t = 0; t < MAX_TARGETS; t++) {
    JsonObject tObj = targetsArr.add<JsonObject>();
    tObj["id"] = t + 1;
    tObj["present"] = targets[t].present;
    if (targets[t].present) {
      if ((oscConfig.sendSmoothed || midiConfig.sendSmoothed) && targets[t].smoothedInitialized) {
        tObj["x"] = (int)targets[t].smoothedX;
        tObj["y"] = (int)targets[t].smoothedY;
        tObj["dist"] = (int)targets[t].smoothedDistance;
        tObj["speed"] = (int)targets[t].smoothedSpeed;
      } else {
        tObj["x"] = targets[t].x;
        tObj["y"] = targets[t].y;
        tObj["dist"] = targets[t].distance;
        tObj["speed"] = targets[t].speed;
      }
    }
  }
  
  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}

// --- OTHER WEB HANDLERS ---
void handleSetTime() {
  if (server.hasArg("tz")) { clientTimezoneOffsetMinutes = server.arg("tz").toInt(); isTimeOffsetSet = true; }
  if (server.hasArg("epoch")) {
    time_t e = strtoul(server.arg("epoch").c_str(), NULL, 10);
    struct timeval tv = {.tv_sec = e, .tv_usec = 0};
    settimeofday(&tv, NULL);
  }
  updateTime();
  server.send(200, "text/plain", "OK");
}

void updateTime() {
  unsigned long c = millis();
  if (c - lastTimeCheck < 1000) return;
  lastTimeCheck = c;
  time_t n = time(nullptr);
  if (n < 1609459200UL || !isTimeOffsetSet) return;
  time_t l = n + (clientTimezoneOffsetMinutes * 60);
  struct tm t;
  gmtime_r(&l, &t);
  int cur = t.tm_hour * 60 + t.tm_min;
  int s = startHour * 60 + startMinute;
  int e = endHour * 60 + endMinute;
  bool on = (s <= e) ? (cur >= s && cur < e) : (cur >= s || cur < e);
  if (!smarthomeOverride && (lightOn != on)) { lightOn = on; }
}

void handleGetCurrentTime() {
  char t[20] = "N/A";
  time_t n = time(nullptr);
  if (n > 1609459200UL && isTimeOffsetSet) {
    time_t l = n + (clientTimezoneOffsetMinutes * 60);
    strftime(t, sizeof(t), "%H:%M:%S", gmtime(&l));
  } else if (n > 1609459200UL) {
    strcpy(t, "No TZ");
  } else {
    strcpy(t, "Syncing...");
  }
  server.send(200, "application/json", String("{\"time\":\"") + t + "\"}");
}

void handleRoot() {
  char sS[6], sE[6];
  sprintf(sS, "%02d:%02d", startHour, startMinute);
  sprintf(sE, "%02d:%02d", endHour, endMinute);
  String h = "";
  h.reserve(8000);
  char c[8];
  sprintf(c, "#%02x%02x%02x", targetSettings[0].color.r, targetSettings[0].color.g, targetSettings[0].color.b);
  int dMI = round((movingIntensity / MAX_BRIGHTNESS_SCALER) * 100.0f);
  float dSI = (stationaryIntensity / MAX_BRIGHTNESS_SCALER) * 100.0f;

  h += R"rawliteral(
<!DOCTYPE html><html><head><title>LightTrack VISION</title><meta name='viewport' content='width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no'><meta charset='UTF-8'><style>body{background-color:#282c34;color:#abb2bf;font-family:sans-serif;margin:0;padding:15px}.container{max-width:700px;margin:auto;background-color:#3a3f4b;padding:20px;border-radius:8px;box-shadow:0 4px 8px #00000033}h1,h2{color:#61afef;text-align:center;border-bottom:1px solid #4b5263;padding-bottom:10px;margin-top:0}h2{margin-top:25px;border-bottom:none}label{display:block;margin-top:15px;margin-bottom:5px;color:#98c379;font-weight:bold}input[type=color]{width:40px;height:40px;border:none;border-radius:5px;padding:0;cursor:pointer;background-color:transparent;vertical-align:middle;margin-left:10px}input[type=time]{font-size:1em;padding:5px;border-radius:4px;border:1px solid #4b5263;background-color:#282c34;color:#abb2bf;margin:0 5px}input[type=radio]{margin:0 5px 0 10px;vertical-align:middle;transform:scale(1.2)}button,a.button-link{display:inline-block;text-decoration:none;font-size:1em;margin:10px 5px;padding:10px 15px;border:none;border-radius:5px;background-color:#61afef;color:#282c34!important;cursor:pointer;transition:background-color .2s;text-align:center}button:hover,a.button-link:hover{background-color:#5295cc}.button-off{background-color:#e06c75}.button-off:hover{background-color:#be5046}.button-nav{background-color:#c678dd}.button-nav:hover{background-color:#a968bd}.button-network{background-color:#e5c07b}.button-network:hover{background-color:#d4a84a}hr{border:none;height:1px;background:#4b5263;margin:25px 0}.value-display{color:#e5c07b;font-weight:normal;display:inline}.time-container{display:flex;justify-content:center;align-items:center;gap:15px;margin-top:5px;flex-wrap:wrap}.current-time{font-size:.9em;color:#5c6370;text-align:center;margin-top:15px}.radio-group label{display:inline;font-weight:normal;color:#abb2bf}.footer{font-size:.8em;color:#5c6370;text-align:center;margin-top:20px}input[type=range]{width:100%;-webkit-appearance:none;background:#4b5263;height:10px;border-radius:5px;margin-bottom:5px;pointer-events:none}input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:22px;height:22px;background:#61afef;border-radius:50%;cursor:pointer;border:3px solid #282c34;pointer-events:auto}details{border:1px solid #4b5263;border-radius:5px;padding:10px;margin-top:10px;background-color:#2c313a}summary{font-weight:bold;color:#c678dd;cursor:pointer;padding:5px;list-style-position:inside}.color-label{display:flex;align-items:center;justify-content:space-between}.nav-buttons{display:flex;flex-wrap:wrap;justify-content:center;gap:10px;margin-bottom:15px}</style><script>const STRIP_LENGTH=)rawliteral";
  h += String(STRIP_LENGTH);
  h += R"rawliteral(;function s(u,cb){fetch(u).then(r=>r.text()).then(d=>{if(cb)cb(d)}).catch(e=>console.error(u,e))}function u(i,v){document.getElementById(i).innerText=v}function t(){var d=new Date(),e=Math.floor(d.getTime()/1000),z=-d.getTimezoneOffset();fetch(`/setTime?epoch=${e}&tz=${z}`).then(()=>c())}function c(){fetch('/getCurrentTime').then(r=>r.json()).then(d=>u('currentTimeDisplay',d.time))}function updateDensity(val){s('/setLedDensity?value='+val,()=>{document.getElementById('led_count_span').innerText=`(${val*STRIP_LENGTH} LEDs)`;});}window.onload=t;setInterval(c,5000);setInterval(t,36e5);</script></head><body><div class='container'><h1>LightTrack VISION</h1>
<div class='nav-buttons'>
<a href='/network' class='button-link button-network'>🌐 Network / MIDI / OSC</a>
<a href='/radarview' class='button-link button-nav'>📡 Radar View</a>
</div>
<hr><h2>Main Light Settings</h2><div class='color-label'><label for='target_color'>Color:</label><input type='color' id='target_color' value=')rawliteral";
  h += String(c);
  h += R"rawliteral(' onchange="s('/setTarget?id=0&r='+parseInt(this.value.substring(1,3),16)+'&g='+parseInt(this.value.substring(3,5),16)+'&b='+parseInt(this.value.substring(5,7),16))"></div><label>Brightness (Moving): <span class='value-display' id='v_mi'>)rawliteral";
  h += String(dMI);
  h += R"rawliteral(</span>%</label><input type='range' min='0' max='100' value=')rawliteral";
  h += String(dMI);
  h += R"rawliteral(' oninput='u("v_mi",this.value)' onchange='s("/setMovingIntensity?value="+this.value)'><label>Length: <span class='value-display' id='v_ml'>)rawliteral";
  h += String(movingLength);
  h += R"rawliteral(</span> LEDs</label><input type='range' min='1' max=')rawliteral";
  h += String(currentNumLeds);
  h += R"rawliteral(' value=')rawliteral";
  h += String(movingLength);
  h += R"rawliteral(' oninput='u("v_ml",this.value)' onchange='s("/setMovingLength?value="+this.value)'><label>Additional Length: <span class='value-display' id='v_al'>)rawliteral";
  h += String(additionalLEDs);
  h += R"rawliteral(</span></label><input type='range' min='0' max=')rawliteral";
  h += String(currentNumLeds / 2);
  h += R"rawliteral(' value=')rawliteral";
  h += String(additionalLEDs);
  h += R"rawliteral(' oninput='u("v_al",this.value)' onchange='s("/setAdditionalLEDs?value="+this.value)'><label>Center Shift: <span class='value-display' id='v_cs'>)rawliteral";
  h += String(centerShift);
  h += R"rawliteral(</span></label><input type='range' min='-)rawliteral";
  h += String(currentNumLeds / 2);
  h += R"rawliteral(' max=')rawliteral";
  h += String(currentNumLeds / 2);
  h += R"rawliteral(' value=')rawliteral";
  h += String(centerShift);
  h += R"rawliteral(' oninput='u("v_cs",this.value)' onchange='s("/setCenterShift?value="+this.value)'><label>Gradient: <span class='value-display' id='v_gs'>)rawliteral";
  h += String(gradientSoftness);
  h += R"rawliteral(</span></label><input type='range' min='0' max='10' value=')rawliteral";
  h += String(gradientSoftness);
  h += R"rawliteral(' oninput='u("v_gs",this.value)' onchange='s("/setGradientSoftness?value="+this.value)'><label>Off Delay: <span class='value-display' id='v_od'>)rawliteral";
  h += String(ledOffDelay);
  h += R"rawliteral(</span>s</label><input type='range' min='0' max='60' value=')rawliteral";
  h += String(ledOffDelay);
  h += R"rawliteral(' oninput='u("v_od",this.value)' onchange='s("/setLedOffDelay?value="+this.value)'><hr><h2>Background Mode</h2><button onclick="s('/toggleBackgroundMode');setTimeout(()=>location.reload(),200)" class=')rawliteral";
  h += (backgroundModeActive ? "button-off" : "");
  h += "'>";
  h += (backgroundModeActive ? "Turn Off" : "Turn On");
  h += R"rawliteral(</button><label style='margin-top:10px'>Background Brightness: <span class='value-display' id='v_si'>)rawliteral";
  h += String(dSI, 1);
  h += R"rawliteral(</span>%</label><input type='range' min='0' max='10' step='0.1' value=')rawliteral";
  h += String(dSI, 1);
  h += R"rawliteral(' oninput='u("v_si",parseFloat(this.value).toFixed(1))' onchange='s("/setStationaryIntensity?value="+this.value)'><hr><h2>Schedule</h2><div class='time-container'><input type='time' id='s_start' value=')rawliteral";
  h += String(sS);
  h += R"rawliteral(' onchange="s('/setSchedule?startHour='+this.value.split(':')[0]+'&startMinute='+this.value.split(':')[1]+'&endHour='+document.getElementById('s_end').value.split(':')[0]+'&endMinute='+document.getElementById('s_end').value.split(':')[1])"><span>-</span><input type='time' id='s_end' value=')rawliteral";
  h += String(sE);
  h += R"rawliteral(' onchange="s('/setSchedule?startHour='+document.getElementById('s_start').value.split(':')[0]+'&startMinute='+document.getElementById('s_start').value.split(':')[1]+'&endHour='+this.value.split(':')[0]+'&endMinute='+this.value.split(':')[1])"></div><div class='current-time'>Time: <span id='currentTimeDisplay'>...</span></div><hr><details><summary>Strip Settings</summary><label>Density: <span id="led_count_span" class='value-display'>(</span></label><div class='radio-group'><input type='radio' id='density30' name='ledDensity' value='30' )rawliteral";
  h += (currentLedDensity == 30 ? "checked" : "");
  h += R"rawliteral( onchange="updateDensity(30)"><label for='density30'>30/m</label><input type='radio' id='density60' name='ledDensity' value='60' )rawliteral";
  h += (currentLedDensity == 60 ? "checked" : "");
  h += R"rawliteral( onchange="updateDensity(60)"><label for='density60'>60/m</label></div></details><div class='footer'>LightTrack VISION + MIDI + OSC</div></div></body></html>)rawliteral";

  server.send(200, "text/html; charset=utf-8", h);
}

void handleSetTargetSettings() {
  if (server.hasArg("id")) {
    int id = server.arg("id").toInt();
    if (id == 0) {
      if (server.hasArg("r")) {
        targetSettings[0].color = CRGB(server.arg("r").toInt(), server.arg("g").toInt(), server.arg("b").toInt());
      }
      saveSettings();
      server.send(200, "text/plain", "OK");
      return;
    }
  }
  server.send(400, "text/plain", "Bad Request");
}

void handleSetMovingIntensity() { if (server.hasArg("value")) { float v = server.arg("value").toFloat() / 100.0f; movingIntensity = constrain(v, 0.0f, 1.0f) * MAX_BRIGHTNESS_SCALER; saveSettings(); server.send(200, "text/plain", "OK"); } }
void handleSetStationaryIntensity() { if (server.hasArg("value")) { float v = server.arg("value").toFloat() / 100.0f; stationaryIntensity = constrain(v, 0.0f, 0.1f) * MAX_BRIGHTNESS_SCALER; saveSettings(); server.send(200, "text/plain", "OK"); } }
void handleSetMovingLength() { if (server.hasArg("value")) { movingLength = constrain(server.arg("value").toInt(), 1, currentNumLeds); saveSettings(); server.send(200, "text/plain", "OK"); } }
void handleSetAdditionalLEDs() { if (server.hasArg("value")) { additionalLEDs = constrain(server.arg("value").toInt(), 0, currentNumLeds / 2); saveSettings(); server.send(200, "text/plain", "OK"); } }
void handleSetCenterShift() { if (server.hasArg("value")) { centerShift = constrain(server.arg("value").toInt(), -currentNumLeds / 2, currentNumLeds / 2); saveSettings(); server.send(200, "text/plain", "OK"); } }
void handleSetLedDensity() { if (server.hasArg("value")) { int n = server.arg("value").toInt(); if ((n == 30 || n == 60) && n != currentLedDensity) { currentLedDensity = n; currentNumLeds = constrain(n * STRIP_LENGTH, 1, MAX_NUM_LEDS); movingLength = constrain(movingLength, 1, currentNumLeds); additionalLEDs = constrain(additionalLEDs, 0, currentNumLeds / 2); centerShift = constrain(centerShift, -currentNumLeds / 2, currentNumLeds / 2); saveSettings(); fill_solid(leds, MAX_NUM_LEDS, CRGB::Black); FastLED.show(); } server.send(200, "text/plain", "OK"); } }
void handleSetGradientSoftness() { if (server.hasArg("value")) { gradientSoftness = constrain(server.arg("value").toInt(), 0, 10); saveSettings(); server.send(200, "text/plain", "OK"); } }
void handleSetLedOffDelay() { if (server.hasArg("value")) { ledOffDelay = constrain(server.arg("value").toInt(), 0, 600); saveSettings(); server.send(200, "text/plain", "OK"); } }
void handleSetSchedule() { if (server.hasArg("startHour")) { startHour = constrain(server.arg("startHour").toInt(), 0, 23); startMinute = constrain(server.arg("startMinute").toInt(), 0, 59); endHour = constrain(server.arg("endHour").toInt(), 0, 23); endMinute = constrain(server.arg("endMinute").toInt(), 0, 59); saveSettings(); updateTime(); server.send(200, "text/plain", "OK"); } }
void handleToggleBackgroundMode() { backgroundModeActive = !backgroundModeActive; saveSettings(); server.send(200, "text/plain", "OK"); }
void handleSmartHomeOn() { lightOn = true; smarthomeOverride = true; server.send(200, "text/plain", "ON"); }
void handleSmartHomeOff() { lightOn = false; smarthomeOverride = true; server.send(200, "text/plain", "OFF"); }
void handleSmartHomeClear() { smarthomeOverride = false; updateTime(); server.send(200, "text/plain", "CLEARED"); }
void handleNotFound() { server.send(404, "text/plain", "404: Not found"); }

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    JsonDocument doc;
    JsonArray zonesArray = doc["inactiveZones"].to<JsonArray>();
    for (int i = 0; i < MAX_INACTIVE_ZONES; ++i) {
      JsonObject zoneObj = zonesArray.add<JsonObject>();
      zoneObj["id"] = i;
      zoneObj["enabled"] = inactiveZones[i].enabled;
      JsonArray cornersArray = zoneObj["corners"].to<JsonArray>();
      for (int j = 0; j < 4; ++j) {
        JsonObject cornerObj = cornersArray.add<JsonObject>();
        cornerObj["x"] = inactiveZones[i].corners[j].x;
        cornerObj["y"] = inactiveZones[i].corners[j].y;
      }
    }
    String jsonOutput;
    serializeJson(doc, jsonOutput);
    webSocket.sendTXT(num, jsonOutput);
  } else if (type == WStype_TEXT) {
    JsonDocument doc;
    if (deserializeJson(doc, payload, length) == DeserializationError::Ok) {
      bool shouldSave = !doc["saveInactiveZones"].isNull();
      if (shouldSave || !doc["setInactiveZones"].isNull()) {
        JsonArray zonesArray = doc[shouldSave ? "saveInactiveZones" : "setInactiveZones"];
        for (JsonObject zoneDataObj : zonesArray) {
          int id = zoneDataObj["id"] | -1;
          if (id >= 0 && id < MAX_INACTIVE_ZONES) {
            inactiveZones[id].enabled = zoneDataObj["enabled"];
            JsonArray cornersArray = zoneDataObj["corners"];
            if (cornersArray.size() == 4) {
              for (int j = 0; j < 4; ++j) {
                inactiveZones[id].corners[j].x = cornersArray[j]["x"];
                inactiveZones[id].corners[j].y = cornersArray[j]["y"];
              }
            }
          }
        }
        if (shouldSave) {
          saveSettings();
          webSocket.sendTXT(num, "{\"zonesSaved\":true}");
        }
      }
    }
  }
}

void broadcastRadarData() {
  if (webSocket.connectedClients() == 0) return;
  
  // Debug: count present targets
  static unsigned long lastBroadcastDebug = 0;
  int presentCount = 0;
  for (int t = 0; t < MAX_TARGETS; t++) {
    if (targets[t].present) presentCount++;
  }
  if (millis() - lastBroadcastDebug > 2000) {
    lastBroadcastDebug = millis();
    Serial.printf("[WS DEBUG] clients:%d, presentTargets:%d\n", 
                  webSocket.connectedClients(), presentCount);
    for (int t = 0; t < MAX_TARGETS; t++) {
      Serial.printf("  targets[%d]: present=%d enabled=%d x=%d y=%d\n",
                    t, targets[t].present, targetSettings[t].enabled, targets[t].x, targets[t].y);
    }
  }
  
  JsonDocument doc;
  JsonArray targetsArray = doc["targets"].to<JsonArray>();
  
  // Send all present targets
  for (int t = 0; t < MAX_TARGETS; t++) {
    if (targets[t].present && targetSettings[t].enabled) {
      JsonObject targetObj = targetsArray.add<JsonObject>();
      targetObj["id"] = t;
      targetObj["x"] = targets[t].x;
      targetObj["y"] = targets[t].y;
      targetObj["s"] = targets[t].speed;
    }
  }
  
  String jsonOutput;
  serializeJson(doc, jsonOutput);
  
  // Debug: show what we're sending (every 2 seconds)
  static unsigned long lastJsonDebug = 0;
  if (millis() - lastJsonDebug > 2000) {
    lastJsonDebug = millis();
    Serial.printf("[WS SEND] clients:%d targets:%d json:%s\n", 
                  webSocket.connectedClients(), presentCount, jsonOutput.c_str());
  }
  
  webSocket.broadcastTXT(jsonOutput);
}

void handleRadarView() {
  String html = R"rawliteral(
<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Radar View</title><style>
body{margin:0;font-family:sans-serif;background-color:#282c34;color:#abb2bf;display:flex;flex-direction:column;align-items:center;padding-top:5px;box-sizing:border-box;overflow:hidden}
h1{color:#61afef;margin-top:0;margin-bottom:5px;text-align:center;font-size:1.2em}
#canvasContainer{position:relative;width:95%;max-width:500px;flex-grow:1;display:flex;justify-content:center;align-items:center;margin-bottom:5px}
#radarCanvas{border:1px solid #4b5263;background-color:#3a3f4b;touch-action:none;max-width:100%;max-height:100%}
#status{margin-top:3px;color:#5c6370;font-size:.8em;text-align:center}
#debug{font-size:0.7em;color:#e5c07b;margin:5px;max-height:60px;overflow:auto;background:#1e2127;padding:5px;border-radius:4px;width:90%;max-width:480px}
.controls{margin-top:5px;display:flex;flex-wrap:wrap;justify-content:center;gap:8px}
.controls label{margin:0 5px;font-size:.85em}
.controls button{padding:5px 8px;font-size:.8em;border:none;border-radius:4px;cursor:pointer;background-color:#61afef;color:#282c34}
button.del-btn{background-color:#e06c75}
button:disabled{opacity:.5;cursor:not-allowed}
.zoom-value{color:#e5c07b;font-weight:bold}
a.back{font-size:.8em;margin-top:5px;color:#61afef}
</style></head><body>
<h1>Radar / Zone Editor</h1>
<div id="canvasContainer"><canvas id="radarCanvas"></canvas></div>
<div id="status">Connecting...</div>
<div id="debug">Debug: waiting...</div>
<div class="controls">
<label>Zoom: <input type="range" id="zoomSlider" min="40" max="180" value="60"> <span id="zoomValue" class="zoom-value">60</span></label>
<button id="addZoneBtn">Add</button><button id="deleteZoneBtn" class="del-btn" disabled>Delete</button><button id="saveZonesBtn">Save</button>
</div>
<a href="/" class="back">← Back</a>
<script>
const canvas = document.getElementById('radarCanvas');
const ctx = canvas.getContext('2d');
const statusEl = document.getElementById('status');
const debugEl = document.getElementById('debug');
const zoomSlider = document.getElementById('zoomSlider');
const zoomValueEl = document.getElementById('zoomValue');
const saveBtn = document.getElementById('saveZonesBtn');
const addBtn = document.getElementById('addZoneBtn');
const delBtn = document.getElementById('deleteZoneBtn');

let targets = [];
let scale = zoomSlider.value / 1000;
let ws = null;
let zoneData = [];
let selectedZone = -1;
let dragInfo = {active: false, handleIndex: -1, startX: 0, startY: 0, originalZone: null};
let needsRedraw = true;
let msgCount = 0;
let lastTargetData = '';

function log(msg) {
  console.log('[Radar]', msg);
  debugEl.textContent = msg;
}

const getOrigin = () => ({x: canvas.width / 2, y: canvas.height * 0.9});
const realToPixel = p => ({x: getOrigin().x + p.x * scale, y: getOrigin().y - p.y * scale});
const pixelToReal = p => ({x: (p.x - getOrigin().x) / scale, y: (getOrigin().y - p.y) / scale});

function resizeCanvas() {
  const cw = document.getElementById("canvasContainer").clientWidth;
  const ah = window.innerHeight - 150;
  const mw = Math.min(cw, 500);
  let nw = mw, nh = mw;
  if (nh > ah) { nh = ah; nw *= (ah / mw); }
  nw = Math.max(nw, 150);
  nh = Math.max(nh, 150);
  if (canvas.width !== Math.floor(nw) || canvas.height !== Math.floor(nh)) {
    canvas.width = Math.floor(nw);
    canvas.height = Math.floor(nh);
    needsRedraw = true;
  }
}

function isPointInPolygon(pt, poly) {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const xi = poly[i].x, yi = poly[i].y, xj = poly[j].x, yj = poly[j].y;
    if (((yi > pt.y) !== (yj > pt.y)) && (pt.x < (xj - xi) * (pt.y - yi) / (yj - yi) + xi)) inside = !inside;
  }
  return inside;
}

function getHandlesForZone(z) {
  if (!z || z.corners.length < 4) return [];
  const pc = z.corners.map(realToPixel);
  const h = [...pc];
  for (let i = 0; i < 4; i++) {
    const p1 = pc[i], p2 = pc[(i + 1) % 4];
    h.push({x: (p1.x + p2.x) / 2, y: (p1.y + p2.y) / 2});
  }
  return h;
}

function draw() {
  const w = canvas.width, h = canvas.height;
  if (w <= 0) return;
  const o = getOrigin();
  
  // Background
  ctx.fillStyle = "#3a3f4b";
  ctx.fillRect(0, 0, w, h);
  
  // Grid lines
  ctx.strokeStyle = "#4b5263";
  ctx.lineWidth = 0.5;
  ctx.beginPath(); ctx.moveTo(o.x, o.y); ctx.lineTo(o.x, 0); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(0, o.y); ctx.lineTo(w, o.y); ctx.stroke();
  
  // Distance arcs
  ctx.textAlign = "center";
  ctx.fillStyle = "#5c6370";
  const fs = Math.max(8, Math.floor(w / 40));
  ctx.font = `${fs}px sans-serif`;
  const dy = Math.ceil(pixelToReal({x: 0, y: 0}).y / 1000);
  for (let i = 1; i <= dy; i++) {
    const r = i * 1000 * scale;
    if (r < 5) continue;
    ctx.beginPath();
    ctx.arc(o.x, o.y, r, Math.PI * 1.02, Math.PI * 1.98);
    ctx.stroke();
    if (o.y - r > fs + 2) ctx.fillText(`${i}m`, o.x, o.y - r - 3);
  }
  
  // Zones
  const zc = ["rgba(224,108,117,0.3)","rgba(229,192,123,0.3)","rgba(152,195,121,0.3)","rgba(97,175,239,0.3)","rgba(198,120,221,0.3)","rgba(86,182,194,0.3)"];
  const bc = ["#e06c75","#e5c07b","#98c379","#61afef","#c678dd","#56b6c2"];
  zoneData.forEach((z, idx) => {
    if (!z.enabled || z.corners.length < 3) return;
    const pc = z.corners.map(realToPixel);
    ctx.save();
    const sel = (selectedZone === idx), ci = idx % 6;
    ctx.strokeStyle = sel ? "#FFF" : bc[ci];
    ctx.fillStyle = zc[ci];
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(pc[0].x, pc[0].y);
    for (let i = 1; i < pc.length; i++) ctx.lineTo(pc[i].x, pc[i].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    if (sel) {
      ctx.fillStyle = "#fff";
      ctx.strokeStyle = "#333";
      ctx.lineWidth = 1;
      getHandlesForZone(z).forEach((p, hi) => {
        ctx.beginPath();
        ctx.arc(p.x, p.y, hi < 4 ? 7 : 4, 0, 2 * Math.PI);
        ctx.fill();
        ctx.stroke();
      });
    }
    ctx.restore();
  });
  
  // TARGETS - Draw them!
  const tr = Math.max(6, Math.floor(w / 50));  // Increased size
  targets.forEach((t, idx) => {
    const p = realToPixel(t);
    let col = "#e06c75";  // Default red
    if (t.s > 3) col = "#98c379";  // Green if moving away
    else if (t.s < -3) col = "#61afef";  // Blue if approaching
    
    // Check if in inactive zone
    const inZone = zoneData.some(z => z.enabled && isPointInPolygon(t, z.corners));
    ctx.globalAlpha = inZone ? 0.4 : 1;
    
    ctx.fillStyle = col;
    ctx.beginPath();
    ctx.arc(p.x, p.y, tr, 0, Math.PI * 2);
    ctx.fill();
    
    // Draw target ID
    ctx.fillStyle = "#fff";
    ctx.font = "10px sans-serif";
    ctx.fillText(`T${idx+1}`, p.x, p.y - tr - 2);
    
    ctx.globalAlpha = 1;
  });
  
  updateUI();
  needsRedraw = false;
}

function animate() {
  if (needsRedraw) draw();
  requestAnimationFrame(animate);
}

function setupWS() {
  const url = `ws://${location.hostname}:81/`;
  log(`Connecting to ${url}...`);
  ws = new WebSocket(url);
  
  ws.onopen = () => {
    log('WebSocket connected!');
    needsRedraw = true;
  };
  
  ws.onclose = () => {
    log('WebSocket disconnected');
    targets = [];
    zoneData = [];
    selectedZone = -1;
    needsRedraw = true;
    setTimeout(setupWS, 3000);
  };
  
  ws.onerror = (e) => {
    log('WebSocket error: ' + e.type);
  };
  
  ws.onmessage = (e) => {
    msgCount++;
    try {
      const d = JSON.parse(e.data);
      
      // Handle targets
      if (d.targets !== undefined) {
        targets = d.targets;
        lastTargetData = JSON.stringify(d.targets);
        if (targets.length > 0) {
          log(`MSG#${msgCount}: ${targets.length} targets - ${lastTargetData}`);
        }
        needsRedraw = true;
      }
      
      // Handle zones
      if (d.inactiveZones !== undefined) {
        zoneData = d.inactiveZones;
        if (selectedZone >= zoneData.length) selectedZone = -1;
        log(`MSG#${msgCount}: Got ${zoneData.length} zones`);
        needsRedraw = true;
      }
      
      if (d.zonesSaved !== undefined) {
        alert(d.zonesSaved ? "Saved!" : "Error");
      }
    } catch (x) {
      log(`MSG#${msgCount}: Parse error - ${e.data.substring(0,50)}`);
    }
  };
}

function updateUI() {
  delBtn.disabled = (selectedZone === -1);
  addBtn.disabled = zoneData.filter(z => z.enabled).length >= zoneData.length;
  statusEl.textContent = (ws && ws.readyState === 1 ? "Connected" : "Offline") + 
    ` | ${targets.length} target(s)` + 
    (selectedZone !== -1 ? ` | Zone ${selectedZone + 1}` : "");
}

// Zone editing handlers
function getZoneAt(pt) {
  for (let i = zoneData.length - 1; i >= 0; i--) {
    if (zoneData[i].enabled && isPointInPolygon(pixelToReal(pt), zoneData[i].corners)) return i;
  }
  return -1;
}

function getHandleAt(pt) {
  if (selectedZone === -1) return -1;
  const h = getHandlesForZone(zoneData[selectedZone]);
  for (let i = 0; i < h.length; i++) {
    const dx = pt.x - h[i].x, dy = pt.y - h[i].y;
    if (dx * dx + dy * dy <= 400) return i;
  }
  return -1;
}

function sendZones() {
  if (!ws || ws.readyState !== 1) return;
  ws.send(JSON.stringify({setInactiveZones: zoneData.map(z => ({id: z.id, enabled: z.enabled, corners: z.corners}))}));
}

function getEventCoords(e, el) {
  const r = el.getBoundingClientRect();
  const cx = e.clientX ?? e.touches?.[0]?.clientX;
  const cy = e.clientY ?? e.touches?.[0]?.clientY;
  return (cx === undefined) ? null : {x: cx - r.left, y: cy - r.top};
}

function onDown(e) {
  const c = getEventCoords(e, canvas);
  if (!c) return;
  const hi = getHandleAt(c);
  let zi = -1;
  if (hi !== -1) {
    zi = selectedZone;
    dragInfo.handleIndex = hi;
    e.preventDefault();
  } else {
    zi = getZoneAt(c);
    dragInfo.handleIndex = -1;
  }
  if (selectedZone !== zi) {
    selectedZone = zi;
    needsRedraw = true;
  }
  if (selectedZone !== -1) {
    dragInfo.active = true;
    dragInfo.startX = c.x;
    dragInfo.startY = c.y;
    dragInfo.originalZone = JSON.parse(JSON.stringify(zoneData[selectedZone]));
  }
  updateUI();
}

function onMove(e) {
  const c = getEventCoords(e, canvas);
  if (!c) return;
  if (dragInfo.active && selectedZone !== -1) {
    const z = zoneData[selectedZone];
    if (dragInfo.handleIndex !== -1) {
      if (dragInfo.handleIndex < 4) {
        z.corners[dragInfo.handleIndex] = pixelToReal(c);
      } else {
        const ei = dragInfo.handleIndex - 4, c1 = ei, c2 = (ei + 1) % 4;
        const oc1 = realToPixel(dragInfo.originalZone.corners[c1]);
        const oc2 = realToPixel(dragInfo.originalZone.corners[c2]);
        const dx = c.x - dragInfo.startX, dy = c.y - dragInfo.startY;
        z.corners[c1] = pixelToReal({x: oc1.x + dx, y: oc1.y + dy});
        z.corners[c2] = pixelToReal({x: oc2.x + dx, y: oc2.y + dy});
      }
    } else {
      const dx = c.x - dragInfo.startX, dy = c.y - dragInfo.startY;
      for (let i = 0; i < z.corners.length; i++) {
        const p = realToPixel(dragInfo.originalZone.corners[i]);
        z.corners[i] = pixelToReal({x: p.x + dx, y: p.y + dy});
      }
    }
    needsRedraw = true;
    sendZones();
  }
}

function onUp() {
  dragInfo.active = false;
  if (selectedZone !== -1) sendZones();
  updateUI();
}

// Event listeners
canvas.addEventListener("mousedown", onDown);
canvas.addEventListener("mousemove", onMove);
window.addEventListener("mouseup", onUp);
canvas.addEventListener("touchstart", onDown, {passive: false});
canvas.addEventListener("touchmove", onMove, {passive: false});
window.addEventListener("touchend", onUp);

saveBtn.onclick = () => {
  if (ws && ws.readyState === 1) {
    ws.send(JSON.stringify({saveInactiveZones: zoneData.map(z => ({id: z.id, enabled: z.enabled, corners: z.corners}))}));
  }
};

zoomSlider.oninput = () => {
  scale = zoomSlider.value / 1000;
  zoomValueEl.textContent = zoomSlider.value;
  needsRedraw = true;
};

addBtn.onclick = () => {
  const fd = zoneData.find(z => !z.enabled);
  if (fd) {
    fd.enabled = true;
    fd.corners = [{x: -200, y: 2200}, {x: 200, y: 2200}, {x: 200, y: 1800}, {x: -200, y: 1800}];
    selectedZone = fd.id;
    sendZones();
    needsRedraw = true;
    updateUI();
  }
};

delBtn.onclick = () => {
  if (selectedZone === -1) return;
  zoneData[selectedZone].enabled = false;
  selectedZone = -1;
  sendZones();
  needsRedraw = true;
  updateUI();
};

window.addEventListener("resize", resizeCanvas);
window.onload = () => {
  resizeCanvas();
  setupWS();
  animate();
};
</script></body></html>
)rawliteral";
  server.send(200, "text/html; charset=utf-8", html);
}
