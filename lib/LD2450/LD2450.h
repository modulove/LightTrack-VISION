/*
 * LD2450.h - Lightweight parser for the Hi-Link LD2450 radar module
 * Tailored for LightTrack VISION sketch
 *
 * Processes raw byte streams, buffers frames, and provides access to up to
 * three tracked targets including their position, velocity, and calculated distance.
 */

#ifndef LD2450_H
#define LD2450_H

#include <Arduino.h>

#define SENSOR_MAX_TARGETS 3
#define SERIAL_BUFFER_SIZE 256
#define SERIAL_SPEED 256000
#define DEFAULT_RETRY_COUNT 1000

class LD2450 {
public:
    struct RadarTarget {
        uint8_t id;
        int16_t x;           // X coordinate in mm
        int16_t y;           // Y coordinate in mm
        int16_t speed;       // Speed in cm/s
        uint16_t resolution;
        uint16_t distance;   // Calculated distance in mm
        bool valid;
    };

    LD2450();

    // Initialization methods
    void begin(Stream &stream);
    void begin(HardwareSerial &serial, int8_t rxPin = -1, int8_t txPin = -1);

    // Main read method - call frequently in loop/task
    // Returns number of valid targets found
    int read();

    // Wait for sensor data with retry
    String waitForSensorMessage(int retryCount = DEFAULT_RETRY_COUNT);

    // Process and return string representation of radar data
    String ProcessSerialDataIntoRadarData();

    // Get target by index (0-2)
    RadarTarget getTarget(uint8_t index);

    // Set number of targets to track (max 3)
    void setNumberOfTargets(uint8_t count);

    // Get current target count setting
    uint8_t getSensorSupportedTargetCount();

private:
    Stream* _serial;
    RadarTarget _targets[SENSOR_MAX_TARGETS];
    uint8_t _targetCount;
    uint8_t _buffer[SERIAL_BUFFER_SIZE];
    uint16_t _bufferIndex;

    void parseFrame();
};

#endif
