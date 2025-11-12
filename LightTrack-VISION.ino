#include <Arduino.h>
#include <LD2450.h>
#include <FastLED.h>
#include <EEPROM.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <time.h>
#include <ArduinoOTA.h>
#include <stdlib.h>
#include "esp_wifi.h"
#include <math.h>
#include <utility> // For std::swap

// --- PIN CONFIGURATION ---
const int RX1_PIN = 8;
const int TX1_PIN = 9;
const int LED_PIN = 7;

// --- LED STRIP CONFIGURATION ---
#define MAX_LED_DENSITY 60
#define STRIP_LENGTH    10
#define MAX_NUM_LEDS    (MAX_LED_DENSITY * STRIP_LENGTH)
#define LED_TYPE        WS2812B
#define COLOR_ORDER     GRB
CRGB leds[MAX_NUM_LEDS];

// --- CURRENT STRIP VARIABLES ---
int currentLedDensity = 60;
int currentNumLeds = (currentLedDensity * STRIP_LENGTH);

// --- RADAR CONFIGURATION ---
LD2450 ld2450;

// --- TARGET TRACKING CONFIGURATION ---
#define MAX_TARGETS 1
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

// --- INACTIVE ZONES (NOW 12 ZONES) ---
#define MAX_INACTIVE_ZONES 12
struct Point {
  int x;
  int y;
};
struct InactiveZone {
  bool enabled;
  Point corners[4]; // 4 corners: top-left, top-right, bottom-right, bottom-left
};
InactiveZone inactiveZones[MAX_INACTIVE_ZONES];

// --- Individual Target Settings Structure ---
struct TargetSetting {
  bool enabled;
  CRGB color;
};

// --- GLOBAL SETTINGS ---
const float MAX_BRIGHTNESS_SCALER = 0.7f;
int ledUpdateInterval = 20;
int movementSensitivity = 1;
int wifiTimeoutMinutes = 7;
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
  {true, CRGB(255, 200, 50)}
};

// --- EEPROM ---
#define EEPROM_SIZE (1024 + sizeof(int) + (8 * sizeof(InactiveZone))) // Increased for 12 zones
#define EEPROM_VERSION 0xAF // EEPROM Version changed due to zone count modification

// --- TARGET STRUCTURE with Kalman Filter variables ---
struct Target {
  bool present;
  int x, y, distance, speed;
  unsigned long lastSeenTime, lastMovementTime;
  float smoothedLEDPosition, velocityLED;
  float p_pos, p_vel;
  float currentBrightness;
  int lastMovementDirection;
  bool isInitialized;
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
TaskHandle_t sensorTaskHandle = NULL, ledTaskHandle = NULL, webServerTaskHandle = NULL;
unsigned long wifiStartTime = 0;
bool wifiActive = true;

// --- FUNCTION PROTOTYPES ---
void sensorTask(void* parameter); void ledTask(void* parameter); void webServerTask(void* parameter);
void loadSettings(); void saveSettings(); bool initializeStrip(); int mapDistanceToLED(int rawDistance);
bool isTargetInInactiveZone(int targetX, int targetY); void setupWiFi(); void setupOTA();
void handleRoot(); void handleSetMovingIntensity(); void handleSetStationaryIntensity();
void handleSetMovingLength(); void handleSetAdditionalLEDs(); void handleSetCenterShift();
void handleSetLedDensity(); void handleSetGradientSoftness(); void handleSetLedOffDelay();
void handleSetTime(); void handleSetSchedule(); void handleNotFound(); void handleSmartHomeOn();
void handleSmartHomeOff(); void handleSmartHomeClear(); void handleToggleBackgroundMode();
void handleGetCurrentTime(); void handleRadarView(); void webSocketEvent(uint8_t, WStype_t, uint8_t*, size_t);
void broadcastRadarData(); void updateTime();
void handleSetTargetSettings();
void checkWiFiTimeout();

// ------------------------- Setup -------------------------
void setup() {
  setCpuFrequencyMhz(80);
  
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n--- setup() begins ---");
  Serial.println("LightTrack VISION (12 Polygon Zones)");
  Serial.printf("CPU Frequency: %d MHz\n", getCpuFrequencyMhz());
  Serial.println("=======================================================");

  randomSeed(ESP.getEfuseMac());

  Serial.println("Initializing Target variables...");
  for (auto & target : targets) {
    target.present = false;
    target.isInitialized = false;
    target.smoothedLEDPosition = 0;
    target.velocityLED = 0.0f;
    target.p_pos = 1000.0f;
    target.p_vel = 1000.0f;
    target.lastMovementTime = 0;
  }

  Serial.println("Setting all INACTIVE zones to DISABLED by default...");
  for (auto & zone : inactiveZones) {
    zone.enabled = false;
  }
  
  if (!SPIFFS.begin(true)) { Serial.println("!!! SPIFFS mount failed"); }

  Serial.println("--- Calling loadSettings() ---");
  loadSettings();

  Serial.println("Initializing LD2450 Radar...");
  Serial1.begin(LD2450_SERIAL_SPEED, SERIAL_8N1, RX1_PIN, TX1_PIN);
  delay(500); ld2450.begin(Serial1); delay(100);
  Serial.println("Radar initialized.");

  setupWiFi();
  wifiStartTime = millis();
  configTzTime("UTC0", "pool.ntp.org", "time.nist.gov");
  setupOTA();
  stripInitialized = initializeStrip();

  Serial.println("Starting RTOS tasks...");
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 5, &sensorTaskHandle, 0);
  xTaskCreatePinnedToCore(ledTask, "LEDTask", 8192, NULL, 4, &ledTaskHandle, 1);
  xTaskCreatePinnedToCore(webServerTask, "WebServerTask", 6144, NULL, 3, &webServerTaskHandle, 0);

  Serial.println("--------------------------------------");
  Serial.println("Setup complete. System is running.");
  Serial.print("Web UI: http://"); Serial.println(WiFi.softAPIP());
  Serial.print("Radar View: http://"); Serial.print(WiFi.softAPIP()); Serial.println("/radarview");
  Serial.printf("Wi-Fi will turn off automatically in %d minutes.\n", wifiTimeoutMinutes);
  Serial.println("--------------------------------------");
}

void loop() {
  updateTime();
  checkWiFiTimeout();
  vTaskDelay(pdMS_TO_TICKS(5000));
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
        bool has_neg = false;
        bool has_pos = false;
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
      Serial.printf("!!! EEPROM version mismatch (is %X, expected %X). Resetting to defaults.\n", checkVal, EEPROM_VERSION);
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
      Serial.println("Loaded Quadrilateral zones from EEPROM.");
  }
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
  
  boolean result = EEPROM.commit();
  EEPROM.end();
  Serial.print("EEPROM save finished. Result: "); Serial.println(result ? "OK" : "ERROR");
  Serial.printf("Total EEPROM bytes used: %d / %d\n", addr, EEPROM_SIZE);
}

bool initializeStrip() {
  Serial.println("Initializing LED Strip (FastLED)...");
  try {
      FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, MAX_NUM_LEDS).setCorrection(TypicalLEDStrip);
      FastLED.setBrightness(255);
      FastLED.clear(true);
      return true;
  } catch (...) {
      Serial.println("!!! CRITICAL ERROR in FastLED.addLeds()!");
      return false;
  }
}

// --- RTOS TASKS ---
void sensorTask(void * parameter) {
  Serial.println("Sensor Task started.");
  for (;;) {
    if (ld2450.read() > 0) {
        unsigned long currentMillis = millis();
        bool targetFoundThisCycle = false;
        int closestTargetIndex = -1;
        int minDistance = 99999;
        for (int i = 0; i < ld2450.getSensorSupportedTargetCount(); i++) {
            LD2450::RadarTarget radarTarget = ld2450.getTarget(i);
            if (radarTarget.valid && radarTarget.distance < minDistance) {
                minDistance = radarTarget.distance;
                closestTargetIndex = i;
            }
        }
        if (closestTargetIndex != -1) {
            LD2450::RadarTarget radarTarget = ld2450.getTarget(closestTargetIndex);
            if (targets[0].present && (millis() - targets[0].lastMovementTime > GHOST_REMOVAL_TIMEOUT)) {
                Serial.printf("Ghost target detected and ignored.\n");
            } else {
                targetFoundThisCycle = true;
                if (!targets[0].present) { targets[0].lastMovementTime = currentMillis; }
                targets[0].x = radarTarget.x;
                targets[0].y = radarTarget.y;
                targets[0].distance = radarTarget.distance;
                targets[0].speed = radarTarget.speed;
                targets[0].present = true;
                targets[0].lastSeenTime = currentMillis;
                if (abs(radarTarget.speed) >= movementSensitivity) {
                    targets[0].lastMovementTime = currentMillis;
                    if (radarTarget.speed > movementSensitivity) targets[0].lastMovementDirection = 1;
                    else if (radarTarget.speed < -movementSensitivity) targets[0].lastMovementDirection = -1;
                }
            }
        }
        if (!targetFoundThisCycle) {
            targets[0].present = false;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void ledTask(void * parameter) {
  unsigned long lastCycleTimeMicros = micros();
  while (true) {
    unsigned long currentMicros = micros();
    float deltaTime = (currentMicros - lastCycleTimeMicros) / 1000000.0f;
    lastCycleTimeMicros = currentMicros;
    if (deltaTime <= 0.0f || deltaTime > 0.1f) deltaTime = (float)ledUpdateInterval / 1000.0f;
    if (!lightOn && !smarthomeOverride) {
        fill_solid(leds, MAX_NUM_LEDS, CRGB::Black);
        FastLED.show();
        vTaskDelay(pdMS_TO_TICKS(ledUpdateInterval * 2)); continue;
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
            target.p_pos = 1000.0f; target.p_vel = 1000.0f; 
            target.isInitialized = true;
        }
        float predicted_pos = target.smoothedLEDPosition + target.velocityLED * deltaTime;
        target.p_pos += q_pos; target.p_vel += q_vel;
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
        if(isMoving) targetBrightness = movingIntensity;
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
      if (direction > 0) { leftEdge = centerLED - (movingLength / 2); rightEdge = leftEdge + movingLength - 1 + additionalLEDs; } 
      else { rightEdge = centerLED + (movingLength / 2); leftEdge = rightEdge - (movingLength - 1) - additionalLEDs; }
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
    vTaskDelay(pdMS_TO_TICKS(max(0L, delayMicros/1000)));
  }
}

// --- WEB & NETWORK FUNCTIONS ---
void setupWiFi() {
  WiFi.mode(WIFI_AP);
  esp_wifi_set_max_tx_power(8); 
  IPAddress local_IP(192, 168, 4, 1);
  WiFi.softAPConfig(local_IP, local_IP, IPAddress(255, 255, 255, 0));
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_AP);
  char deviceName[25];
  sprintf(deviceName, "LightTrack-VISION-%02X%02X%02X", mac[3], mac[4], mac[5]);
  WiFi.softAP(deviceName, "12345678");
  server.on("/", HTTP_GET, handleRoot);
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
void checkWiFiTimeout() {if(wifiActive&&wifiTimeoutMinutes>0&&millis()-wifiStartTime>(unsigned long)wifiTimeoutMinutes*60*1000){WiFi.mode(WIFI_OFF);wifiActive=false;}}
void webServerTask(void*p){unsigned long lastRadarBroadcast=0;for(;;){if(wifiActive){server.handleClient();webSocket.loop();ArduinoOTA.handle();if(webSocket.connectedClients()>0&&millis()-lastRadarBroadcast>100){broadcastRadarData();lastRadarBroadcast=millis();}}vTaskDelay(pdMS_TO_TICKS(2));}}
void setupOTA(){ArduinoOTA.onStart([](){});ArduinoOTA.onEnd([](){});ArduinoOTA.onProgress([](unsigned int p,unsigned int t){});ArduinoOTA.onError([](ota_error_t e){});uint8_t m[6];esp_read_mac(m,ESP_MAC_WIFI_AP);char d[25];sprintf(d,"LightTrack-VISION-%02X%02X%02X",m[3],m[4],m[5]);ArduinoOTA.setHostname(d);ArduinoOTA.begin();}
void handleSetTime(){if(server.hasArg("tz")){clientTimezoneOffsetMinutes=server.arg("tz").toInt();isTimeOffsetSet=true;}if(server.hasArg("epoch")){time_t e=strtoul(server.arg("epoch").c_str(),NULL,10);struct timeval tv={.tv_sec=e,.tv_usec=0};settimeofday(&tv,NULL);}updateTime();server.send(200,"text/plain","OK");}
void updateTime(){unsigned long c=millis();if(c-lastTimeCheck<1000)return;lastTimeCheck=c;time_t n=time(nullptr);if(n<1609459200UL||!isTimeOffsetSet)return;time_t l=n+(clientTimezoneOffsetMinutes*60);struct tm t;gmtime_r(&l,&t);int cur=t.tm_hour*60+t.tm_min;int s=startHour*60+startMinute;int e=endHour*60+endMinute;bool on=(s<=e)?(cur>=s&&cur<e):(cur>=s||cur<e);if(!smarthomeOverride&&(lightOn!=on)){lightOn=on;}}
void handleGetCurrentTime(){char t[20]="N/A";time_t n=time(nullptr);if(n>1609459200UL&&isTimeOffsetSet){time_t l=n+(clientTimezoneOffsetMinutes*60);strftime(t,sizeof(t),"%H:%M:%S",gmtime(&l));}else if(n>1609459200UL){strcpy(t,"No TZ");}else{strcpy(t,"Syncing...");}server.send(200,"application/json",String("{\"time\":\"")+t+"\"}");}

void handleRoot(){
  char sS[6],sE[6];
  sprintf(sS,"%02d:%02d",startHour,startMinute);
  sprintf(sE,"%02d:%02d",endHour,endMinute);
  String h="";
  h.reserve(7000);
  char c[8];
  sprintf(c,"#%02x%02x%02x",targetSettings[0].color.r,targetSettings[0].color.g,targetSettings[0].color.b);
  int dMI=round((movingIntensity/MAX_BRIGHTNESS_SCALER)*100.0f);
  float dSI=(stationaryIntensity/MAX_BRIGHTNESS_SCALER)*100.0f;
  
  h += R"rawliteral(
<!DOCTYPE html><html><head><title>LightTrack VISION</title><meta name='viewport' content='width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no'><meta charset='UTF-8'><style>body{background-color:#282c34;color:#abb2bf;font-family:sans-serif;margin:0;padding:15px}.container{max-width:700px;margin:auto;background-color:#3a3f4b;padding:20px;border-radius:8px;box-shadow:0 4px 8px #00000033}h1,h2{color:#61afef;text-align:center;border-bottom:1px solid #4b5263;padding-bottom:10px;margin-top:0}h2{margin-top:25px;border-bottom:none}label{display:block;margin-top:15px;margin-bottom:5px;color:#98c379;font-weight:bold}input[type=color]{width:40px;height:40px;border:none;border-radius:5px;padding:0;cursor:pointer;background-color:transparent;vertical-align:middle;margin-left:10px}input[type=time]{font-size:1em;padding:5px;border-radius:4px;border:1px solid #4b5263;background-color:#282c34;color:#abb2bf;margin:0 5px}input[type=radio]{margin:0 5px 0 10px;vertical-align:middle;transform:scale(1.2)}button,a.button-link{display:inline-block;text-decoration:none;font-size:1em;margin:10px 5px;padding:10px 15px;border:none;border-radius:5px;background-color:#61afef;color:#282c34!important;cursor:pointer;transition:background-color .2s;text-align:center}button:hover,a.button-link:hover{background-color:#5295cc}.button-off{background-color:#e06c75}.button-off:hover{background-color:#be5046}.button-nav{background-color:#c678dd}.button-nav:hover{background-color:#a968bd}hr{border:none;height:1px;background:#4b5263;margin:25px 0}.value-display{color:#e5c07b;font-weight:normal;display:inline}.time-container{display:flex;justify-content:center;align-items:center;gap:15px;margin-top:5px;flex-wrap:wrap}.current-time{font-size:.9em;color:#5c6370;text-align:center;margin-top:15px}.radio-group label{display:inline;font-weight:normal;color:#abb2bf}.footer{font-size:.8em;color:#5c6370;text-align:center;margin-top:20px}input[type=range]{width:100%;-webkit-appearance:none;background:#4b5263;height:10px;border-radius:5px;margin-bottom:5px;pointer-events:none}input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:22px;height:22px;background:#61afef;border-radius:50%;cursor:pointer;border:3px solid #282c34;pointer-events:auto}details{border:1px solid #4b5263;border-radius:5px;padding:10px;margin-top:10px;background-color:#2c313a}summary{font-weight:bold;color:#c678dd;cursor:pointer;padding:5px;list-style-position:inside}.color-label{display:flex;align-items:center;justify-content:space-between}</style><script>const STRIP_LENGTH=)rawliteral";
  h += String(STRIP_LENGTH);
  h += R"rawliteral(;function s(u,cb){fetch(u).then(r=>r.text()).then(d=>{if(cb)cb(d)}).catch(e=>console.error(u,e))}function u(i,v){document.getElementById(i).innerText=v}function t(){var d=new Date(),e=Math.floor(d.getTime()/1000),z=-d.getTimezoneOffset();fetch(`/setTime?epoch=${e}&tz=${z}`).then(()=>c())}function c(){fetch('/getCurrentTime').then(r=>r.json()).then(d=>u('currentTimeDisplay',d.time))}function updateDensity(val){s('/setLedDensity?value='+val,()=>{const numLeds=val*STRIP_LENGTH,halfLeds=Math.floor(numLeds/2);document.getElementById('v_ml').parentElement.nextElementSibling.max=numLeds;document.getElementById('v_al').parentElement.nextElementSibling.max=halfLeds;const cs=document.getElementById('v_cs').parentElement.nextElementSibling;cs.max=halfLeds;cs.min=-halfLeds;document.getElementById('led_count_span').innerText=`(${numLeds} LEDs)`;});}window.onload=t;setInterval(c,5000);setInterval(t,36e5);</script></head><body><div class='container'><h1>LightTrack VISION</h1><div style='text-align:center;margin-bottom:20px'><p><b>Inactive Zone Setup (Polygon Masks):</b></p><a href='/radarview' class='button-link button-nav'>Visualization & Zone Setup</a></div><hr><h2>Main Light Settings</h2><div class='color-label'><label for='target_color'>Color:</label><input type='color' id='target_color' value=')rawliteral";
  h += String(c);
  h += R"rawliteral(' onchange="s('/setTarget?id=0&r='+parseInt(this.value.substring(1,3),16)+'&g='+parseInt(this.value.substring(3,5),16)+'&b='+parseInt(this.value.substring(5,7),16))"></div><label>Brightness (Moving): <span class='value-display' id='v_mi'>)rawliteral";
  h += String(dMI);
  h += R"rawliteral(</span>%</label><input type='range' min='0' max='100' value=')rawliteral";
  h += String(dMI);
  h += R"rawliteral(' oninput='u("v_mi",this.value)' onchange='s("/setMovingIntensity?value="+this.value)'><label>Length (Moving): <span class='value-display' id='v_ml'>)rawliteral";
  h += String(movingLength);
  h += R"rawliteral(</span> LEDs</label><input type='range' min='1' max=')rawliteral";
  h += String(currentNumLeds);
  h += R"rawliteral(' value=')rawliteral";
  h += String(movingLength);
  h += R"rawliteral(' oninput='u("v_ml",this.value)' onchange='s("/setMovingLength?value="+this.value)'><label>Additional Length (Directional): <span class='value-display' id='v_al'>)rawliteral";
  h += String(additionalLEDs);
  h += R"rawliteral(</span> LEDs</label><input type='range' min='0' max=')rawliteral";
  h += String(currentNumLeds/2);
  h += R"rawliteral(' value=')rawliteral";
  h += String(additionalLEDs);
  h += R"rawliteral(' oninput='u("v_al",this.value)' onchange='s("/setAdditionalLEDs?value="+this.value)'><label>Center Shift: <span class='value-display' id='v_cs'>)rawliteral";
  h += String(centerShift);
  h += R"rawliteral(</span> LEDs</label><input type='range' min='-)rawliteral";
  h += String(currentNumLeds/2);
  h += R"rawliteral(' max=')rawliteral";
  h += String(currentNumLeds/2);
  h += R"rawliteral(' value=')rawliteral";
  h += String(centerShift);
  h += R"rawliteral(' oninput='u("v_cs",this.value)' onchange='s("/setCenterShift?value="+this.value)'><label>Gradient Softness: <span class='value-display' id='v_gs'>)rawliteral";
  h += String(gradientSoftness);
  h += R"rawliteral(</span> (0-10)</label><input type='range' min='0' max='10' value=')rawliteral";
  h += String(gradientSoftness);
  h += R"rawliteral(' oninput='u("v_gs",this.value)' onchange='s("/setGradientSoftness?value="+this.value)'><label>Off Delay: <span class='value-display' id='v_od'>)rawliteral";
  h += String(ledOffDelay);
  h += R"rawliteral(</span> sec</label><input type='range' min='0' max='60' value=')rawliteral";
  h += String(ledOffDelay);
  h += R"rawliteral(' oninput='u("v_od",this.value)' onchange='s("/setLedOffDelay?value="+this.value)'><hr><h2>Background Mode</h2><button onclick="s('/toggleBackgroundMode');setTimeout(()=>location.reload(),200)" class=')rawliteral";
  h += (backgroundModeActive ? "button-off" : "");
  h += "'>";
  h += (backgroundModeActive ? "Turn Off Background" : "Turn On Background");
  h += R"rawliteral(</button><label style='margin-top:10px'>Brightness (Background/Still): <span class='value-display' id='v_si'>)rawliteral";
  h += String(dSI,1);
  h += R"rawliteral(</span>%</label><input type='range' min='0' max='10' step='0.1' value=')rawliteral";
  h += String(dSI,1);
  h += R"rawliteral(' oninput='u("v_si",parseFloat(this.value).toFixed(1))' onchange='s("/setStationaryIntensity?value="+this.value)'><hr><h2>Schedule</h2><div class='time-container'><input type='time' id='s_start' value=')rawliteral";
  h += String(sS);
  h += R"rawliteral(' onchange="s('/setSchedule?startHour='+this.value.split(':')[0]+'&startMinute='+this.value.split(':')[1]+'&endHour='+document.getElementById('s_end').value.split(':')[0]+'&endMinute='+document.getElementById('s_end').value.split(':')[1])"><span>-</span><input type='time' id='s_end' value=')rawliteral";
  h += String(sE);
  h += R"rawliteral(' onchange="s('/setSchedule?startHour='+document.getElementById('s_start').value.split(':')[0]+'&startMinute='+document.getElementById('s_start').value.split(':')[1]+'&endHour='+this.value.split(':')[0]+'&endMinute='+this.value.split(':')[1])"></div><div class='current-time'>Local Time: <span id='currentTimeDisplay'>Syncing...</span></div><hr><details><summary>Strip Settings</summary><label>LED Density: <span id="led_count_span" class='value-display'>( )rawliteral";
  h += String(currentNumLeds);
  h += R"rawliteral( LEDs)</span></label><div class='radio-group'><input type='radio' id='density30' name='ledDensity' value='30' )rawliteral";
  h += (currentLedDensity==30 ? "checked" : "");
  h += R"rawliteral( onchange="updateDensity(30)"><label for='density30'>30 LEDs/m</label><input type='radio' id='density60' name='ledDensity' value='60' )rawliteral";
  h += (currentLedDensity==60 ? "checked" : "");
  h += R"rawliteral( onchange="updateDensity(60)"><label for='density60'>60 LEDs/m</label></div></details><hr><div class='footer'>LightTrack VISION</div></div></body></html>)rawliteral";
  
  server.send(200, "text/html; charset=utf-8", h);
}

void handleSetTargetSettings(){if(server.hasArg("id")){int id=server.arg("id").toInt();if(id==0){if(server.hasArg("r")){targetSettings[0].color=CRGB(server.arg("r").toInt(),server.arg("g").toInt(),server.arg("b").toInt());}saveSettings();server.send(200,"text/plain","OK");return;}}server.send(400,"text/plain","Bad Request");}
void handleSetMovingIntensity(){if(server.hasArg("value")){float v=server.arg("value").toFloat()/100.0f;movingIntensity=constrain(v,0.0f,1.0f)*MAX_BRIGHTNESS_SCALER;saveSettings();server.send(200,"text/plain","OK");}}
void handleSetStationaryIntensity(){if(server.hasArg("value")){float v=server.arg("value").toFloat()/100.0f;stationaryIntensity=constrain(v,0.0f,0.1f)*MAX_BRIGHTNESS_SCALER;saveSettings();server.send(200,"text/plain","OK");}}
void handleSetMovingLength(){if(server.hasArg("value")){movingLength=constrain(server.arg("value").toInt(),1,currentNumLeds);saveSettings();server.send(200,"text/plain","OK");}}
void handleSetAdditionalLEDs(){if(server.hasArg("value")){additionalLEDs=constrain(server.arg("value").toInt(),0,currentNumLeds/2);saveSettings();server.send(200,"text/plain","OK");}}
void handleSetCenterShift(){if(server.hasArg("value")){centerShift=constrain(server.arg("value").toInt(),-currentNumLeds/2,currentNumLeds/2);saveSettings();server.send(200,"text/plain","OK");}}
void handleSetLedDensity(){if(server.hasArg("value")){int n=server.arg("value").toInt();if((n==30||n==60)&&n!=currentLedDensity){currentLedDensity=n;currentNumLeds=constrain(n*STRIP_LENGTH,1,MAX_NUM_LEDS);movingLength=constrain(movingLength,1,currentNumLeds);additionalLEDs=constrain(additionalLEDs,0,currentNumLeds/2);centerShift=constrain(centerShift,-currentNumLeds/2,currentNumLeds/2);saveSettings();fill_solid(leds,MAX_NUM_LEDS,CRGB::Black);FastLED.show();}server.send(200,"text/plain","OK");}}
void handleSetGradientSoftness(){if(server.hasArg("value")){gradientSoftness=constrain(server.arg("value").toInt(),0,10);saveSettings();server.send(200,"text/plain","OK");}}
void handleSetLedOffDelay(){if(server.hasArg("value")){ledOffDelay=constrain(server.arg("value").toInt(),0,600);saveSettings();server.send(200,"text/plain","OK");}}
void handleSetSchedule(){if(server.hasArg("startHour")){startHour=constrain(server.arg("startHour").toInt(),0,23);startMinute=constrain(server.arg("startMinute").toInt(),0,59);endHour=constrain(server.arg("endHour").toInt(),0,23);endMinute=constrain(server.arg("endMinute").toInt(),0,59);saveSettings();updateTime();server.send(200,"text/plain","OK");}}
void handleToggleBackgroundMode(){backgroundModeActive=!backgroundModeActive;saveSettings();server.send(200,"text/plain","OK");}
void handleSmartHomeOn(){lightOn=true;smarthomeOverride=true;server.send(200,"text/plain","ON");}
void handleSmartHomeOff(){lightOn=false;smarthomeOverride=true;server.send(200,"text/plain","OFF");}
void handleSmartHomeClear(){smarthomeOverride=false;updateTime();server.send(200,"text/plain","CLEARED");}
void handleNotFound(){server.send(404,"text/plain","404: Not found");}
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {if(type==WStype_CONNECTED){IPAddress ip=webSocket.remoteIP(num);JsonDocument doc;JsonArray zonesArray=doc["inactiveZones"].to<JsonArray>();for(int i=0;i<MAX_INACTIVE_ZONES;++i){JsonObject zoneObj=zonesArray.add<JsonObject>();zoneObj["id"]=i;zoneObj["enabled"]=inactiveZones[i].enabled;JsonArray cornersArray=zoneObj["corners"].to<JsonArray>();for(int j=0;j<4;++j){JsonObject cornerObj=cornersArray.add<JsonObject>();cornerObj["x"]=inactiveZones[i].corners[j].x;cornerObj["y"]=inactiveZones[i].corners[j].y;}}String jsonOutput;serializeJson(doc,jsonOutput);webSocket.sendTXT(num,jsonOutput);}else if(type==WStype_TEXT){JsonDocument doc;if(deserializeJson(doc,payload,length)==DeserializationError::Ok){bool shouldSave=!doc["saveInactiveZones"].isNull();if(shouldSave||!doc["setInactiveZones"].isNull()){JsonArray zonesArray=doc[shouldSave?"saveInactiveZones":"setInactiveZones"];for(JsonObject zoneData:zonesArray){int id=zoneData["id"]|-1;if(id>=0&&id<MAX_INACTIVE_ZONES){inactiveZones[id].enabled=zoneData["enabled"];JsonArray cornersArray=zoneData["corners"];if(cornersArray.size()==4){for(int j=0;j<4;++j){inactiveZones[id].corners[j].x=cornersArray[j]["x"];inactiveZones[id].corners[j].y=cornersArray[j]["y"];}}}}if(shouldSave){saveSettings();webSocket.sendTXT(num,"{\"zonesSaved\":true}");}}}}}
void broadcastRadarData(){if(webSocket.connectedClients()==0)return;JsonDocument doc;JsonArray targetsArray=doc["targets"].to<JsonArray>();if(targets[0].present&&targetSettings[0].enabled){JsonObject targetObj=targetsArray.add<JsonObject>();targetObj["id"]=0;targetObj["x"]=targets[0].x;targetObj["y"]=targets[0].y;targetObj["s"]=targets[0].speed;}String jsonOutput;serializeJson(doc,jsonOutput);webSocket.broadcastTXT(jsonOutput);}

void handleRadarView() {
  String html = R"rawliteral(
<!DOCTYPE html><html lang="en"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Radar Visualization & Zone Setup</title><style>body{margin:0;font-family:sans-serif;background-color:#282c34;color:#abb2bf;display:flex;flex-direction:column;align-items:center;padding-top:5px;box-sizing:border-box;overflow:hidden}h1{color:#61afef;margin-top:0;margin-bottom:5px;text-align:center;flex-shrink:0;font-size:1.3em}#canvasContainer{position:relative;width:95%;max-width:500px;flex-grow:1;display:flex;justify-content:center;align-items:center;margin-bottom:5px}#radarCanvas{border:1px solid #4b5263;background-color:#3a3f4b;touch-action:none;max-width:100%;max-height:100%;object-fit:contain}#status{margin-top:3px;color:#5c6370;font-size:.85em;flex-shrink:0;text-align:center;height:1.1em}.controls{margin-top:5px;display:flex;flex-wrap:wrap;justify-content:center;align-items:center;gap:8px;flex-shrink:0}.controls label{margin:0 5px;white-space:nowrap;font-size:.9em}.controls input[type=range]{vertical-align:middle;cursor:pointer;height:8px}.controls button{padding:6px 10px;font-size:.85em;border:none;border-radius:4px;cursor:pointer;background-color:#61afef;color:#282c34}button.del-btn{background-color:#e06c75}button:disabled{background-color:#4b5263;color:#5c6370;cursor:not-allowed;opacity:.7}.zoom-value{color:#e5c07b;font-weight:bold}a.button-link{display:inline-block;text-decoration:none;font-size:.85em;margin:5px;padding:6px 10px;border:none;border-radius:4px;background-color:#98c379;color:#282c34!important;cursor:pointer;text-align:center;flex-shrink:0}
</style>
</head><body><h1>Radar Map / Polygon Zone Editor</h1>
<div id="canvasContainer"><canvas id="radarCanvas"></canvas></div>
<div id="status">Connecting...</div><div class="controls">
<label>Zoom: <input type="range" id="zoomSlider" min="40" max="180" value="60" step="1"> <span id="zoomValue" class="zoom-value">60</span> px/m</label>
<button id="addZoneBtn">Add Zone</button>
<button id="deleteZoneBtn" class="del-btn" disabled>Delete Selected</button>
<button id="saveZonesBtn">Save Zones</button></div>
<a href="/" class="button-link">Back to Main Settings</a>
<script>
const canvas=document.getElementById('radarCanvas'),ctx=canvas.getContext('2d'),statusEl=document.getElementById('status'),zoomSlider=document.getElementById('zoomSlider'),zoomValue=document.getElementById('zoomValue'),saveBtn=document.getElementById('saveZonesBtn'),addBtn=document.getElementById('addZoneBtn'),delBtn=document.getElementById('deleteZoneBtn');
let targets=[],scale=zoomSlider.value/1000,ws,animationFrame,zoneData=[],selectedZone=-1,dragInfo={active:!1,handleIndex:-1,startX:0,startY:0,originalZone:null},needsRedraw=!0;
const getOrigin=()=>({x:canvas.width/2,y:canvas.height*.9});
const realToPixel=p=>({x:getOrigin().x+p.x*scale,y:getOrigin().y-p.y*scale});
const pixelToReal=p=>({x:(p.x-getOrigin().x)/scale,y:(getOrigin().y-p.y)/scale});
const getEventCoords=(evt,element)=>{const rect=element.getBoundingClientRect();const clientX=evt.clientX??evt.touches?.[0]?.clientX;const clientY=evt.clientY??evt.touches?.[0]?.clientY;if(typeof clientX==='undefined'||typeof clientY==='undefined')return null;return{x:clientX-rect.left,y:clientY-rect.top}};
function resizeCanvas(){const containerW=document.getElementById("canvasContainer").clientWidth;const availableH=window.innerHeight-(document.querySelector(".controls").offsetHeight+statusEl.offsetHeight+document.querySelector("h1").offsetHeight+document.querySelector(".button-link").offsetHeight+20);const maxW=Math.min(containerW,500);let newW=maxW,newH=maxW;if(newH>availableH){newH=availableH;newW*=(availableH/newW)}newW=Math.max(newW,150);newH=Math.max(newH,150);if(canvas.width!==Math.floor(newW)||canvas.height!==Math.floor(newH)){canvas.width=Math.floor(newW);canvas.height=Math.floor(newH);needsRedraw=!0}}
function isPointInPolygon(point,polygon){let isInside=!1;for(let i=0,j=polygon.length-1;i<polygon.length;j=i++){const xi=polygon[i].x,yi=polygon[i].y;const xj=polygon[j].x,yj=polygon[j].y;const intersect=((yi>point.y)!==(yj>point.y))&&(point.x<(xj-xi)*(point.y-yi)/(yj-yi)+xi);if(intersect)isInside=!isInside}return isInside}
function getHandlesForZone(zone){if(!zone||zone.corners.length<4)return[];const pc=zone.corners.map(realToPixel);const handles=[...pc];for(let i=0;i<4;i++){const p1=pc[i];const p2=pc[(i+1)%4];handles.push({x:(p1.x+p2.x)/2,y:(p1.y+p2.y)/2})}return handles}
function draw(){const w=canvas.width,h=canvas.height;if(w<=0||h<=0)return;const origin=getOrigin();ctx.fillStyle="#3a3f4b";ctx.fillRect(0,0,w,h);ctx.strokeStyle="#4b5263";ctx.lineWidth=.5;ctx.beginPath();ctx.moveTo(origin.x,origin.y);ctx.lineTo(origin.x,0);ctx.stroke();ctx.beginPath();ctx.moveTo(0,origin.y);ctx.lineTo(w,origin.y);ctx.stroke();ctx.textAlign="center";ctx.fillStyle="#5c6370";const fontSize=Math.max(8,Math.min(12,Math.floor(w/35)));ctx.font=`${fontSize}px sans-serif`;const distY=Math.ceil(pixelToReal({x:0,y:0}).y/1000);for(let i=1;i<=distY;i++){const radius=i*1000*scale;if(radius<5)continue;ctx.beginPath();ctx.arc(origin.x,origin.y,radius,Math.PI*1.02,Math.PI*1.98);ctx.stroke();if(origin.y-radius>fontSize+2){ctx.fillText(`${i}m`,origin.x,origin.y-radius-3)}}if(origin.x+1000*scale<w-15){ctx.fillText("1m",origin.x+1000*scale,origin.y+fontSize+2)}if(origin.x>fontSize+5){ctx.fillText("Y",origin.x,fontSize)}
const zoneColors=["rgba(224,108,117,0.3)","rgba(229,192,123,0.3)","rgba(152,195,121,0.3)","rgba(97,175,239,0.3)","rgba(198,120,221,0.3)","rgba(86,182,194,0.3)","rgba(209,154,102,0.3)","rgba(229,99,156,0.3)","rgba(12,201,139,0.3)","rgba(235,203,139,0.3)","rgba(120,125,221,0.3)","rgba(182,86,98,0.3)"];
const borderColors=["#e06c75","#e5c07b","#98c379","#61afef","#c678dd","#56b6c2","#d19a66","#e5639c","#0cc98b","#ebcb8b","#787dd1","#b65662"];
zoneData.forEach((zone,idx)=>{if(!zone.enabled||zone.corners.length<3)return;const pixelCorners=zone.corners.map(realToPixel);ctx.save();const isSelected=(selectedZone===idx);const colorIdx=idx%12;ctx.strokeStyle=isSelected?"#FFFFFF":borderColors[colorIdx];ctx.fillStyle=zoneColors[colorIdx];ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(pixelCorners[0].x,pixelCorners[0].y);for(let i=1;i<pixelCorners.length;i++){ctx.lineTo(pixelCorners[i].x,pixelCorners[i].y)}ctx.closePath();ctx.fill();ctx.stroke();if(isSelected){ctx.fillStyle="#fff";ctx.strokeStyle="#333";ctx.lineWidth=1;const handles=getHandlesForZone(zone);handles.forEach((p,handleIdx)=>{const radius=handleIdx<4?8:5;ctx.beginPath();ctx.arc(p.x,p.y,radius,0,2*Math.PI);ctx.fill();ctx.stroke()})}ctx.fillStyle=isSelected?"#FFFFFF":borderColors[colorIdx];ctx.textAlign="left";ctx.fillText(`${zone.id+1}`,pixelCorners[0].x+4,pixelCorners[0].y+fontSize+4);ctx.restore()});ctx.font=`${Math.max(9,Math.min(13,Math.floor(w/30)))}px sans-serif`;const targetRadius=Math.max(4,Math.min(8,Math.floor(w/50)));targets.forEach(t=>{const p=realToPixel(t);let color="#e06c75";if(t.s>3)color="#98c379";else if(t.s<-3)color="#61afef";let isInZone=zoneData.some(z=>z.enabled&&isPointInPolygon(t,z.corners));ctx.globalAlpha=isInZone?.4:1;ctx.fillStyle=color;ctx.beginPath();ctx.arc(p.x,p.y,targetRadius,0,Math.PI*2);ctx.fill();ctx.globalAlpha=1;ctx.fillStyle="#abb2bf";ctx.textAlign="left";ctx.fillText(`${t.id+1}`,p.x+targetRadius+2,p.y+targetRadius/2)});updateUI();needsRedraw=!1}
function animationLoop(){if(needsRedraw){draw()}animationFrame=requestAnimationFrame(animationLoop)}
function setupWebSocket(){const url=`ws://${window.location.hostname}:81/`;statusEl.textContent=`Connecting to ${url}...`;if(ws&&ws.readyState!==WebSocket.CLOSED){ws.onclose=null;ws.close()}ws=new WebSocket(url);ws.onopen=()=>{console.log("[WebSocket] OPEN.");if(!animationFrame)animationLoop();needsRedraw=!0};ws.onclose=()=>{console.log("[WebSocket] CLOSED.");statusEl.textContent="Disconnected. Reconnecting in 5s...";targets=[];zoneData=[];selectedZone=-1;updateUI();if(animationFrame){cancelAnimationFrame(animationFrame);animationFrame=null}needsRedraw=!0;draw();setTimeout(setupWebSocket,5000)};ws.onerror=error=>{console.error("[WebSocket] Error:",error);statusEl.textContent="WebSocket Error"};ws.onmessage=event=>{try{const data=JSON.parse(event.data);let hasChanged=!1;if(typeof data.targets!=='undefined'){targets=data.targets;hasChanged=!0}if(typeof data.inactiveZones!=='undefined'){zoneData=data.inactiveZones;if(selectedZone>=zoneData.length)selectedZone=-1;updateUI();hasChanged=!0}if(typeof data.zonesSaved!=='undefined'){alert(data.zonesSaved?"Zones saved to device memory!":"Error saving zones.")}if(hasChanged){needsRedraw=!0}}catch(e){console.error("[WebSocket] JSON Parse Error:",e,event.data);statusEl.textContent="Data Error"}}}
function getZoneAt(point){for(let i=zoneData.length-1;i>=0;i--){const zone=zoneData[i];if(zone.enabled&&isPointInPolygon(pixelToReal(point),zone.corners)){return i}}return -1}
function getHandleAt(point){if(selectedZone===-1)return-1;const handles=getHandlesForZone(zoneData[selectedZone]);const hitRadius=24;for(let i=0;i<handles.length;i++){const p=handles[i];const dx=point.x-p.x,dy=point.y-p.y;if(dx*dx+dy*dy<=hitRadius*hitRadius){return i}}return-1}
function sendZonesUpdate(){if(!ws||ws.readyState!==WebSocket.OPEN)return;const payload={setInactiveZones:zoneData.map(z=>({id:z.id,enabled:z.enabled,corners:z.corners}))};ws.send(JSON.stringify(payload))}
function updateUI(){const isZoneSelected=(selectedZone!==-1);delBtn.disabled=!isZoneSelected;addBtn.disabled=zoneData.filter(z=>z.enabled).length>=zoneData.length;const editStatus=isZoneSelected?` | Editing Zone ${selectedZone+1}`:"";let wsStatus="Connecting...";if(ws){wsStatus=ws.readyState===WebSocket.OPEN?"Connected":ws.readyState===WebSocket.CONNECTING?"Connecting...":"Offline"}statusEl.textContent=`${wsStatus} | Targets: ${targets.length}${editStatus}`}
const handleCursors=['nwse-resize','nesw-resize','nwse-resize','nesw-resize','ns-resize','ew-resize','ns-resize','ew-resize','move'];
function handleMouseDown(evt){const coords=getEventCoords(evt,canvas);if(!coords)return;let zoneIdx=-1;const handleIndex=getHandleAt(coords);if(handleIndex!==-1){zoneIdx=selectedZone;dragInfo.handleIndex=handleIndex;evt.preventDefault()}else{zoneIdx=getZoneAt(coords);dragInfo.handleIndex=-1}if(selectedZone!==zoneIdx){selectedZone=zoneIdx;needsRedraw=!0}if(selectedZone!==-1){dragInfo.active=!0;dragInfo.startX=coords.x;dragInfo.startY=coords.y;dragInfo.originalZone=JSON.parse(JSON.stringify(zoneData[selectedZone]));canvas.style.cursor=handleIndex!==-1?handleCursors[handleIndex]:'move'}updateUI()}
function handleMouseMove(evt){const coords=getEventCoords(evt,canvas);if(!coords)return;if(dragInfo.active&&selectedZone!==-1){const zone=zoneData[selectedZone];if(dragInfo.handleIndex!==-1){if(dragInfo.handleIndex<4){const newPos=pixelToReal(coords);zone.corners[dragInfo.handleIndex]=newPos}else{const edgeIndex=dragInfo.handleIndex-4;const c1=edgeIndex;const c2=(edgeIndex+1)%4;const orig_c1=realToPixel(dragInfo.originalZone.corners[c1]);const orig_c2=realToPixel(dragInfo.originalZone.corners[c2]);const dx=coords.x-dragInfo.startX,dy=coords.y-dragInfo.startY;zone.corners[c1]=pixelToReal({x:orig_c1.x+dx,y:orig_c1.y+dy});zone.corners[c2]=pixelToReal({x:orig_c2.x+dx,y:orig_c2.y+dy})}}else{const dx=coords.x-dragInfo.startX,dy=coords.y-dragInfo.startY;for(let i=0;i<zone.corners.length;i++){const p=realToPixel(dragInfo.originalZone.corners[i]);zone.corners[i]=pixelToReal({x:p.x+dx,y:p.y+dy})}}needsRedraw=!0;sendZonesUpdate()}else{const handleIndex=getHandleAt(coords);canvas.style.cursor=handleIndex!==-1?handleCursors[handleIndex]:'default'}}
function handleMouseUp(evt){dragInfo.active=!1;canvas.style.cursor='default';if(selectedZone!==-1){sendZonesUpdate()}updateUI()}
canvas.addEventListener("mousedown",handleMouseDown);canvas.addEventListener("mousemove",handleMouseMove);window.addEventListener("mouseup",handleMouseUp);canvas.addEventListener("mouseleave",handleMouseUp);canvas.addEventListener("touchstart",handleMouseDown,{passive:!1});canvas.addEventListener("touchmove",handleMouseMove,{passive:!1});window.addEventListener("touchend",handleMouseUp);window.addEventListener("touchcancel",handleMouseUp);
saveBtn.onclick=()=>{if(!ws||ws.readyState!==WebSocket.OPEN)return;const payload={saveInactiveZones:zoneData.map(z=>({id:z.id,enabled:z.enabled,corners:z.corners}))};ws.send(JSON.stringify(payload))};
zoomSlider.oninput=()=>{scale=zoomSlider.value/1000;zoomValue.textContent=zoomSlider.value;needsRedraw=!0};
addBtn.onclick=()=>{const firstDisabled=zoneData.find(z=>!z.enabled);if(firstDisabled){firstDisabled.enabled=!0;firstDisabled.corners=[{x:-200,y:2200},{x:200,y:2200},{x:200,y:1800},{x:-200,y:1800}];selectedZone=firstDisabled.id;sendZonesUpdate();needsRedraw=!0;updateUI()}else{alert(`All ${zoneData.length} zones are in use. Delete one first.`)} };
delBtn.onclick=()=>{if(selectedZone===-1)return;zoneData[selectedZone].enabled=!1;selectedZone=-1;sendZonesUpdate();needsRedraw=!0;updateUI()};
window.addEventListener("resize",resizeCanvas);window.addEventListener("load",()=>{resizeCanvas();setupWebSocket()});
</script></body></html>
)rawliteral";
  server.send(200, "text/html; charset=utf-8", html);
}
